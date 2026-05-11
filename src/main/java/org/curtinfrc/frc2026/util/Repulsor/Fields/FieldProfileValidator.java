/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package org.curtinfrc.frc2026.util.Repulsor.Fields;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointObjectiveRole;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;

/**
 * Provides field profile validator functionality for the Repulsor field/profile definition layer
 * used to tune Repulsor for a specific game. Use this type from robot code, field profiles, or
 * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public final class FieldProfileValidator {
  private FieldProfileValidator() {}

  /**
   * Runs require valid in the Repulsor runtime.
   *
   * @param cfg value used by this operation.
   * @param source value used by this operation.
   */
  public static void requireValid(FieldProfileConfig cfg, String source) {
    List<String> errors = validate(cfg);
    if (!errors.isEmpty()) {
      throw new IllegalArgumentException(
          "Invalid Repulsor profile " + source + ": " + String.join("; ", errors));
    }
  }

  /**
   * Returns the validate value maintained by this Repulsor component.
   *
   * @param cfg value used by this operation.
   * @return value produced by this operation.
   */
  public static List<String> validate(FieldProfileConfig cfg) {
    List<String> errors = new ArrayList<>();
    if (cfg == null) {
      errors.add("config is null");
      return errors;
    }

    if (cfg.id == null || cfg.id.isBlank()) errors.add("id must be non-empty");
    if (cfg.schemaVersion != null) {
      if (cfg.schemaVersion <= 0) errors.add("schemaVersion must be positive when provided");
      if (cfg.schemaVersion > FieldProfileConfig.CURRENT_SCHEMA_VERSION) {
        errors.add(
            "schemaVersion "
                + cfg.schemaVersion
                + " is newer than supported version "
                + FieldProfileConfig.CURRENT_SCHEMA_VERSION);
      }
    }
    if (cfg.gameName == null || cfg.gameName.isBlank()) errors.add("gameName must be non-empty");
    if (cfg.gameYear == null || cfg.gameYear <= 0) errors.add("gameYear must be positive");

    if (cfg.geometry == null) {
      errors.add("geometry is required");
    } else {
      requirePositive(cfg.geometry.lengthMeters, "geometry.lengthMeters", errors);
      requirePositive(cfg.geometry.widthMeters, "geometry.widthMeters", errors);
    }

    if (cfg.resources != null) {
      for (Map.Entry<String, FieldProfileConfig.ResourceConfig> entry : cfg.resources.entrySet()) {
        String prefix = "resources." + entry.getKey();
        FieldProfileConfig.ResourceConfig resource = entry.getValue();
        if (entry.getKey() == null || entry.getKey().isBlank()) {
          errors.add("resource key must be non-empty");
        }
        if (resource == null) {
          errors.add(prefix + " is null");
          continue;
        }
        requirePositive(resource.radiusMeters, prefix + ".radiusMeters", errors);
        requirePositive(resource.unitValue, prefix + ".unitValue", errors);
        requirePositive(resource.sigmaMeters, prefix + ".sigmaMeters", errors);
      }
    }

    if (cfg.semanticRegions != null) {
      for (Map.Entry<String, FieldProfileConfig.SemanticRegionConfig> entry :
          cfg.semanticRegions.entrySet()) {
        validateSemanticRegion(entry.getKey(), entry.getValue(), errors);
      }
    }

    if (cfg.projectileShots != null) {
      for (Map.Entry<String, FieldProfileConfig.ProjectileShotConfig> entry :
          cfg.projectileShots.entrySet()) {
        validateProjectileShot(entry.getKey(), entry.getValue(), errors);
      }
    }

    validateRanking(cfg.predictiveRanking, errors);
    validateObjectiveSelection(cfg.objectiveSelection, errors);
    validateCollectPlanner(cfg.collectPlanner, errors);
    validateWaypointing(cfg.waypointing, errors);
    validatePlannerRuntime(cfg.plannerRuntime, errors);
    validateAutoPath(cfg.autoPath, errors);
    validateStrategyPresets(cfg, errors);

    if (cfg.shuttleShot != null && Boolean.TRUE.equals(cfg.shuttleShot.enabled)) {
      validateProjectileShot("legacyShuttleShot", cfg.shuttleShot, errors);
    }

    return errors;
  }

  private static void validateRanking(
      FieldProfileConfig.RankingConfig ranking, List<String> errors) {
    if (ranking == null) return;
    String prefix = "predictiveRanking";
    requireOptionalNonNegative(ranking.advantageGain, prefix + ".advantageGain", errors);
    requireOptionalNonNegative(ranking.distanceCost, prefix + ".distanceCost", errors);
    requireOptionalNonNegative(ranking.pressureCost, prefix + ".pressureCost", errors);
    requireOptionalNonNegative(ranking.congestionCost, prefix + ".congestionCost", errors);
    requireOptionalNonNegative(ranking.capacityGain, prefix + ".capacityGain", errors);
    requireOptionalNonNegative(ranking.headingGain, prefix + ".headingGain", errors);
    requireOptionalNonNegative(ranking.hysteresisBonus, prefix + ".hysteresisBonus", errors);
    requireOptionalNonNegative(
        ranking.hysteresisPersistSeconds, prefix + ".hysteresisPersistSeconds", errors);
  }

  private static void validateObjectiveSelection(
      FieldProfileConfig.ObjectiveSelectionProfileConfig objectiveSelection, List<String> errors) {
    if (objectiveSelection == null) return;
    String prefix = "objectiveSelection";
    requireOptionalPositive(objectiveSelection.candidateLimit, prefix + ".candidateLimit", errors);
    requireOptionalNonNegative(
        objectiveSelection.switchScoreMargin, prefix + ".switchScoreMargin", errors);
  }

  private static void validateCollectPlanner(
      FieldProfileConfig.CollectPlannerConfig collectPlanner, List<String> errors) {
    if (collectPlanner == null) return;
    String prefix = "collectPlanner";
    requireOptionalNonNegative(collectPlanner.groupCellMeters, prefix + ".groupCellMeters", errors);
    requireOptionalNonNegative(
        collectPlanner.nearbyRadiusMeters, prefix + ".nearbyRadiusMeters", errors);
    requireOptionalNonNegative(
        collectPlanner.liveObservationMaxAgeSeconds,
        prefix + ".liveObservationMaxAgeSeconds",
        errors);
    requireOptionalNonNegative(
        collectPlanner.predictorObservationMaxAgeSeconds,
        prefix + ".predictorObservationMaxAgeSeconds",
        errors);
    requireOptionalNonNegative(
        collectPlanner.stickyNoProgressSeconds, prefix + ".stickyNoProgressSeconds", errors);
    requireOptionalNonNegative(
        collectPlanner.switchCooldownSeconds, prefix + ".switchCooldownSeconds", errors);
    requireOptionalNonNegative(
        collectPlanner.collectCellMeters, prefix + ".collectCellMeters", errors);
    requireOptionalNonNegative(
        collectPlanner.resourceUnitGain, prefix + ".resourceUnitGain", errors);
    requireOptionalNonNegative(collectPlanner.etaCost, prefix + ".etaCost", errors);
    requireOptionalNonNegative(
        collectPlanner.hubFrontTrapPenalty, prefix + ".hubFrontTrapPenalty", errors);
    requireOptionalNonNegative(
        collectPlanner.canonicalScoreDropLimit, prefix + ".canonicalScoreDropLimit", errors);
    requireOptionalNonNegative(
        collectPlanner.richerUnitsAbsGain, prefix + ".richerUnitsAbsGain", errors);
    requireOptionalNonNegative(
        collectPlanner.richerUnitsRelGain, prefix + ".richerUnitsRelGain", errors);
    requireOptionalNonNegative(
        collectPlanner.richerEtaDeltaMaxSeconds, prefix + ".richerEtaDeltaMaxSeconds", errors);
    requireOptionalNonNegative(
        collectPlanner.richerScoreDropLimit, prefix + ".richerScoreDropLimit", errors);
    requireOptionalNonNegative(
        collectPlanner.liveFuelPreferScoreMargin, prefix + ".liveFuelPreferScoreMargin", errors);
    requireOptionalNonNegative(
        collectPlanner.hubFrontTrapEscapeScoreAllowDrop,
        prefix + ".hubFrontTrapEscapeScoreAllowDrop",
        errors);
    requireOptionalNonNegative(
        collectPlanner.nearbyCentroidScoreDropLimit,
        prefix + ".nearbyCentroidScoreDropLimit",
        errors);
    requireOptionalNonNegative(
        collectPlanner.liveRelockScoreDropLimit, prefix + ".liveRelockScoreDropLimit", errors);
    requireOptionalNonNegative(
        collectPlanner.stickyPreferRankedScoreMargin,
        prefix + ".stickyPreferRankedScoreMargin",
        errors);
    requireOptionalNonNegative(
        collectPlanner.farSwitchLockDistanceMeters,
        prefix + ".farSwitchLockDistanceMeters",
        errors);
    requireOptionalNonNegative(
        collectPlanner.farSwitchForceMultiplier, prefix + ".farSwitchForceMultiplier", errors);
    requireOptionalNonNegative(
        collectPlanner.closeSwitchEasyDistanceMeters,
        prefix + ".closeSwitchEasyDistanceMeters",
        errors);
    requireOptionalNonNegative(
        collectPlanner.closeSwitchMarginScale, prefix + ".closeSwitchMarginScale", errors);
  }

  private static void validateWaypointing(
      FieldProfileConfig.WaypointingConfig waypointing, List<String> errors) {
    if (waypointing == null) return;
    String prefix = "waypointing";
    requireOptionalNonNegative(waypointing.centerBandMeters, prefix + ".centerBandMeters", errors);
    requireOptionalNonNegative(
        waypointing.restageDistanceMeters, prefix + ".restageDistanceMeters", errors);
    requireOptionalNonNegative(
        waypointing.gatePaddingMeters, prefix + ".gatePaddingMeters", errors);
    requireOptionalNonNegative(waypointing.leadThroughScale, prefix + ".leadThroughScale", errors);
    requireOptionalNonNegative(
        waypointing.leadThroughMinMeters, prefix + ".leadThroughMinMeters", errors);
    requireOptionalNonNegative(
        waypointing.leadThroughMaxMeters, prefix + ".leadThroughMaxMeters", errors);
    requireOptionalNonNegative(
        waypointing.deepCenterBandMeters, prefix + ".deepCenterBandMeters", errors);
    requireOptionalNonNegative(
        waypointing.centerReturnStageTriggerMeters,
        prefix + ".centerReturnStageTriggerMeters",
        errors);
    requireOptionalNonNegative(
        waypointing.centerReturnIntersectionTriggerMeters,
        prefix + ".centerReturnIntersectionTriggerMeters",
        errors);
    requireOptionalNonNegative(
        waypointing.centerReturnExitMinMeters, prefix + ".centerReturnExitMinMeters", errors);
    requireOptionalNonNegative(
        waypointing.centerReturnExitMaxMeters, prefix + ".centerReturnExitMaxMeters", errors);
    requireOptionalNonNegative(
        waypointing.centerReturnGateMinOffsetMeters,
        prefix + ".centerReturnGateMinOffsetMeters",
        errors);
    requireOptionalNonNegative(
        waypointing.fieldEdgeMarginMeters, prefix + ".fieldEdgeMarginMeters", errors);
    if (waypointing.leadThroughMinMeters != null
        && waypointing.leadThroughMaxMeters != null
        && waypointing.leadThroughMaxMeters < waypointing.leadThroughMinMeters) {
      errors.add(prefix + ".leadThroughMaxMeters must be >= leadThroughMinMeters");
    }
    if (waypointing.centerReturnExitMinMeters != null
        && waypointing.centerReturnExitMaxMeters != null
        && waypointing.centerReturnExitMaxMeters < waypointing.centerReturnExitMinMeters) {
      errors.add(prefix + ".centerReturnExitMaxMeters must be >= centerReturnExitMinMeters");
    }
    validateWaypointZones(prefix, waypointing.zones, errors);
    validateWaypointRules(prefix, waypointing, errors);
  }

  private static void validateWaypointZones(
      String prefix, Map<String, FieldProfileConfig.ZoneConfig> zones, List<String> errors) {
    if (zones == null) return;
    for (Map.Entry<String, FieldProfileConfig.ZoneConfig> entry : zones.entrySet()) {
      String name = entry.getKey();
      FieldProfileConfig.ZoneConfig zone = entry.getValue();
      String zonePrefix = prefix + ".zones." + name;
      if (name == null || name.isBlank()) errors.add(prefix + ".zones key must be non-empty");
      if (zone == null) {
        errors.add(zonePrefix + " is null");
        continue;
      }
      requireOptionalNonNegative(zone.minXMeters, zonePrefix + ".minXMeters", errors);
      requireOptionalNonNegative(zone.maxXMeters, zonePrefix + ".maxXMeters", errors);
      requireOptionalNonNegative(zone.minYMeters, zonePrefix + ".minYMeters", errors);
      requireOptionalNonNegative(zone.maxYMeters, zonePrefix + ".maxYMeters", errors);
      if (zone.minXMeters != null && zone.maxXMeters != null && zone.maxXMeters < zone.minXMeters) {
        errors.add(zonePrefix + ".maxXMeters must be >= minXMeters");
      }
      if (zone.minYMeters != null && zone.maxYMeters != null && zone.maxYMeters < zone.minYMeters) {
        errors.add(zonePrefix + ".maxYMeters must be >= minYMeters");
      }
    }
  }

  private static void validateWaypointRules(
      String prefix, FieldProfileConfig.WaypointingConfig waypointing, List<String> errors) {
    if (waypointing == null || waypointing.rules == null) return;
    for (int i = 0; i < waypointing.rules.size(); i++) {
      FieldProfileConfig.WaypointRuleConfig rule = waypointing.rules.get(i);
      String rulePrefix = prefix + ".rules[" + i + "]";
      if (rule == null) {
        errors.add(rulePrefix + " is null");
        continue;
      }
      if (rule.name == null || rule.name.isBlank())
        errors.add(rulePrefix + ".name must be non-empty");
      validateWaypointObjectiveRole(rule.objectiveRole, rulePrefix + ".objectiveRole", errors);
      if (!zoneExists(rule.fromZone, waypointing.zones)) {
        errors.add(rulePrefix + ".fromZone must reference a defined waypointing.zones entry");
      }
      if (!zoneExists(rule.toZone, waypointing.zones)) {
        errors.add(rulePrefix + ".toZone must reference a defined waypointing.zones entry");
      }
      validateWaypointScoring(rule.scoring, rulePrefix + ".scoring", errors);
      if (!Boolean.TRUE.equals(rule.direct)
          && (rule.candidates == null || rule.candidates.isEmpty())) {
        errors.add(rulePrefix + ".candidates must be non-empty unless direct is true");
      }
      if (rule.candidates != null) {
        for (int c = 0; c < rule.candidates.size(); c++) {
          validateWaypointCandidate(
              rule.candidates.get(c), rulePrefix + ".candidates[" + c + "]", errors);
        }
      }
    }
  }

  private static boolean zoneExists(
      String zoneName, Map<String, FieldProfileConfig.ZoneConfig> zones) {
    return zoneName == null || zoneName.isBlank() || (zones != null && zones.containsKey(zoneName));
  }

  private static void validateWaypointObjectiveRole(String role, String name, List<String> errors) {
    if (role == null || role.isBlank()) return;
    try {
      FieldPlannerWaypointObjectiveRole.valueOf(role.trim().toUpperCase());
    } catch (IllegalArgumentException ex) {
      errors.add(name + " must be one of FieldPlannerWaypointObjectiveRole values");
    }
  }

  private static void validateWaypointScoring(
      FieldProfileConfig.WaypointScoringConfig scoring, String prefix, List<String> errors) {
    if (scoring == null) return;
    requireOptionalNonNegative(scoring.distanceCost, prefix + ".distanceCost", errors);
    requireOptionalNonNegative(scoring.goalAlignmentGain, prefix + ".goalAlignmentGain", errors);
    requireOptionalNonNegative(
        scoring.obstacleClearanceGain, prefix + ".obstacleClearanceGain", errors);
    requireOptionalNonNegative(scoring.preferenceGain, prefix + ".preferenceGain", errors);
  }

  private static void validateWaypointCandidate(
      FieldProfileConfig.WaypointCandidateConfig candidate, String prefix, List<String> errors) {
    if (candidate == null) {
      errors.add(prefix + " is null");
      return;
    }
    if (candidate.name == null || candidate.name.isBlank())
      errors.add(prefix + ".name must be non-empty");
    requireOptionalNonNegative(candidate.entryXMeters, prefix + ".entryXMeters", errors);
    requireOptionalNonNegative(candidate.entryYMeters, prefix + ".entryYMeters", errors);
    requireOptionalNonNegative(candidate.exitXMeters, prefix + ".exitXMeters", errors);
    requireOptionalNonNegative(candidate.exitYMeters, prefix + ".exitYMeters", errors);
    if (candidate.entryXMeters == null || candidate.entryYMeters == null) {
      errors.add(prefix + ".entryXMeters and entryYMeters are required");
    }
    if ((candidate.exitXMeters == null) != (candidate.exitYMeters == null)) {
      errors.add(prefix + ".exitXMeters and exitYMeters must be provided together");
    }
  }

  private static void validateAutoPath(
      FieldProfileConfig.AutoPathConfig autoPath, List<String> errors) {
    if (autoPath == null) return;
    String prefix = "autoPath";
    requireOptionalNonNegative(
        autoPath.episodeCooldownSeconds, prefix + ".episodeCooldownSeconds", errors);
    requireOptionalNonNegative(autoPath.pinnedFailSeconds, prefix + ".pinnedFailSeconds", errors);
    requireOptionalNonNegative(autoPath.stuckFailSeconds, prefix + ".stuckFailSeconds", errors);
    requireOptionalNonNegative(
        autoPath.progressEpsilonMeters, prefix + ".progressEpsilonMeters", errors);
    requireOptionalNonNegative(
        autoPath.pinnedProgressMinMeters, prefix + ".pinnedProgressMinMeters", errors);
    requireOptionalNonNegative(
        autoPath.stuckDistanceMinMeters, prefix + ".stuckDistanceMinMeters", errors);
    requireOptionalNonNegative(
        autoPath.successNearDistanceMeters, prefix + ".successNearDistanceMeters", errors);
    requireOptionalPositive(autoPath.collectGoalUnits, prefix + ".collectGoalUnits", errors);
    requireOptionalNonNegative(
        autoPath.shootLockEnterMeters, prefix + ".shootLockEnterMeters", errors);
    requireOptionalNonNegative(
        autoPath.shootLockExitMeters, prefix + ".shootLockExitMeters", errors);
    requireOptionalNonNegative(
        autoPath.shootLockMinRotationDegrees, prefix + ".shootLockMinRotationDegrees", errors);
    requireOptionalNonNegative(
        autoPath.shootReadyPositionToleranceMeters,
        prefix + ".shootReadyPositionToleranceMeters",
        errors);
    requireOptionalNonNegative(
        autoPath.shootReadyRotationToleranceDegrees,
        prefix + ".shootReadyRotationToleranceDegrees",
        errors);
    requireOptionalNonNegative(
        autoPath.collectHoldGoalNearMeters, prefix + ".collectHoldGoalNearMeters", errors);
    requireOptionalNonNegative(
        autoPath.collectFarResourceMinDistanceMeters,
        prefix + ".collectFarResourceMinDistanceMeters",
        errors);
    if (autoPath.shootLockEnterMeters != null
        && autoPath.shootLockExitMeters != null
        && autoPath.shootLockExitMeters < autoPath.shootLockEnterMeters) {
      errors.add(prefix + ".shootLockExitMeters must be >= shootLockEnterMeters");
    }
  }

  private static void validatePlannerRuntime(
      FieldProfileConfig.PlannerRuntimeConfig plannerRuntime, List<String> errors) {
    if (plannerRuntime == null) return;
    String prefix = "plannerRuntime";
    requireOptionalPositive(
        plannerRuntime.globalFallbackCellMeters, prefix + ".globalFallbackCellMeters", errors);
    requireOptionalPositive(
        plannerRuntime.globalFallbackLookaheadMeters,
        prefix + ".globalFallbackLookaheadMeters",
        errors);
    requireOptionalPositive(
        plannerRuntime.globalFallbackMaxExpandedNodes,
        prefix + ".globalFallbackMaxExpandedNodes",
        errors);
    requireOptionalPositive(
        plannerRuntime.globalFallbackMaxRuntimeSeconds,
        prefix + ".globalFallbackMaxRuntimeSeconds",
        errors);
    requireOptionalNonNegative(
        plannerRuntime.globalFallbackClearanceBufferMeters,
        prefix + ".globalFallbackClearanceBufferMeters",
        errors);
    requireOptionalNonNegative(
        plannerRuntime.globalFallbackDistanceCostWeight,
        prefix + ".globalFallbackDistanceCostWeight",
        errors);
    requireOptionalNonNegative(
        plannerRuntime.globalFallbackObstacleClearanceCostWeight,
        prefix + ".globalFallbackObstacleClearanceCostWeight",
        errors);
    requireOptionalNonNegative(
        plannerRuntime.globalFallbackWallClearanceCostWeight,
        prefix + ".globalFallbackWallClearanceCostWeight",
        errors);
    requireOptionalNonNegative(
        plannerRuntime.globalFallbackTurnCostWeight,
        prefix + ".globalFallbackTurnCostWeight",
        errors);
    requireOptionalNonNegative(
        plannerRuntime.globalFallbackCorridorPreferenceCostWeight,
        prefix + ".globalFallbackCorridorPreferenceCostWeight",
        errors);
    requireOptionalNonNegative(
        plannerRuntime.globalFallbackPartialRouteMinProgressMeters,
        prefix + ".globalFallbackPartialRouteMinProgressMeters",
        errors);
    requireOptionalNonNegative(
        plannerRuntime.globalFallbackPartialRouteMinClearanceMeters,
        prefix + ".globalFallbackPartialRouteMinClearanceMeters",
        errors);
    requireOptionalNonNegative(
        plannerRuntime.forceThroughGoalDistanceMeters,
        prefix + ".forceThroughGoalDistanceMeters",
        errors);
    requireOptionalNonNegative(
        plannerRuntime.forceThroughWallDistanceMeters,
        prefix + ".forceThroughWallDistanceMeters",
        errors);
  }

  private static void validateStrategyPresets(FieldProfileConfig cfg, List<String> errors) {
    if (cfg.strategyPresets == null) return;
    if (cfg.defaultStrategyPreset != null
        && !cfg.defaultStrategyPreset.isBlank()
        && !cfg.strategyPresets.containsKey(cfg.defaultStrategyPreset)) {
      errors.add("defaultStrategyPreset must reference a defined strategyPresets entry");
    }
    for (Map.Entry<String, FieldProfileConfig.StrategyPresetConfig> entry :
        cfg.strategyPresets.entrySet()) {
      String name = entry.getKey();
      if (name == null || name.isBlank()) {
        errors.add("strategy preset key must be non-empty");
        continue;
      }
      FieldProfileConfig.StrategyPresetConfig preset = entry.getValue();
      if (preset == null) {
        errors.add("strategyPresets." + name + " is null");
        continue;
      }
      validateRanking(preset.predictiveRanking, errors);
      validateObjectiveSelection(preset.objectiveSelection, errors);
      validateCollectPlanner(preset.collectPlanner, errors);
      FieldProfileConfig.WaypointingConfig mergedWaypointing =
          new FieldProfileConfig.WaypointingConfig();
      FieldProfileConfig.mergeWaypointingForValidation(
          mergedWaypointing, cfg.waypointing, preset.waypointing);
      validateWaypointing(mergedWaypointing, errors);
      validatePlannerRuntime(preset.plannerRuntime, errors);
      validateAutoPath(preset.autoPath, errors);
    }
  }

  private static void validateSemanticRegion(
      String id, FieldProfileConfig.SemanticRegionConfig region, List<String> errors) {
    String prefix = "semanticRegions." + id;
    if (id == null || id.isBlank()) errors.add("semantic region key must be non-empty");
    if (region == null) {
      errors.add(prefix + " is null");
      return;
    }
    if (region.shape != null
        && !region.shape.isBlank()
        && !"rectangle".equalsIgnoreCase(region.shape)) {
      errors.add(prefix + ".shape must be rectangle when provided");
    }
    requireOptionalNonNegative(region.minXMeters, prefix + ".minXMeters", errors);
    requireOptionalNonNegative(region.maxXMeters, prefix + ".maxXMeters", errors);
    requireOptionalNonNegative(region.minYMeters, prefix + ".minYMeters", errors);
    requireOptionalNonNegative(region.maxYMeters, prefix + ".maxYMeters", errors);
    requireOptionalNonNegative(region.collectPenalty, prefix + ".collectPenalty", errors);
    requireOptionalNonNegative(region.collectPreference, prefix + ".collectPreference", errors);
    if (region.minXMeters != null
        && region.maxXMeters != null
        && region.maxXMeters < region.minXMeters) {
      errors.add(prefix + ".maxXMeters must be >= minXMeters");
    }
    if (region.minYMeters != null
        && region.maxYMeters != null
        && region.maxYMeters < region.minYMeters) {
      errors.add(prefix + ".maxYMeters must be >= minYMeters");
    }
  }

  /**
   * Returns the validate value maintained by this Repulsor component.
   *
   * @param field value used by this operation.
   * @return value produced by this operation.
   */
  public static List<String> validate(FieldDefinition field) {
    List<String> errors = new ArrayList<>();
    if (field == null) {
      errors.add("field is null");
      return errors;
    }

    FieldGeometry geometry = field.geometry();
    for (Obstacle obstacle : field.fieldObstacles()) {
      if (obstacle == null) errors.add(field.gameName() + " has null field obstacle");
    }
    for (Obstacle wall : field.walls()) {
      if (wall == null) errors.add(field.gameName() + " has null wall obstacle");
    }
    if (field.defaultCollectSetpoint().isPresent()
        && !geometry.contains(
            field.defaultCollectSetpoint().get().getBlue(SetpointContext.EMPTY).getTranslation())) {
      errors.add("default collect setpoint is outside field geometry");
    }
    if (field.defaultScoreSetpoint().isPresent()
        && !geometry.contains(
            field.defaultScoreSetpoint().get().getBlue(SetpointContext.EMPTY).getTranslation())) {
      errors.add("default score setpoint is outside field geometry");
    }
    return errors;
  }

  private static void requirePositive(Double value, String name, List<String> errors) {
    if (value == null || !Double.isFinite(value) || value <= 0.0) {
      errors.add(name + " must be finite and > 0");
    }
  }

  private static void requireNonNegative(Double value, String name, List<String> errors) {
    if (value == null || !Double.isFinite(value) || value < 0.0) {
      errors.add(name + " must be finite and >= 0");
    }
  }

  private static void validateProjectileShot(
      String id, FieldProfileConfig.ProjectileShotConfig shot, List<String> errors) {
    String prefix = "projectileShots." + id;
    if (id == null || id.isBlank()) errors.add("projectile shot id must be non-empty");
    if (shot == null) {
      errors.add(prefix + " is null");
      return;
    }
    if (!Boolean.TRUE.equals(shot.enabled)) return;
    if (shot.target != null) {
      requireOptionalNonNegative(shot.target.blueXMeters, prefix + ".target.blueXMeters", errors);
      requireOptionalNonNegative(shot.target.blueYMeters, prefix + ".target.blueYMeters", errors);
    }
    requirePositive(shot.targetHeightMeters, prefix + ".targetHeightMeters", errors);
    validateShotConstraints(prefix + ".constraints", shot.constraints, errors);
    requireNonNegative(shot.behindTargetMeters, prefix + ".behindTargetMeters", errors);
    requireNonNegative(shot.fieldMarginMeters, prefix + ".fieldMarginMeters", errors);
    validateMovingShot(prefix + ".movingShot", shot.movingShot, errors);
    if (shot.lateralOffsetsMeters == null || shot.lateralOffsetsMeters.length == 0) {
      errors.add(prefix + ".lateralOffsetsMeters must have at least one value");
    }
    if (shot.fallbackGamePiece == null) {
      errors.add(prefix + ".fallbackGamePiece is required");
    } else {
      requirePositive(shot.fallbackGamePiece.massKg, prefix + ".fallbackGamePiece.massKg", errors);
      requirePositive(
          shot.fallbackGamePiece.crossSectionAreaM2,
          prefix + ".fallbackGamePiece.crossSectionAreaM2",
          errors);
      requirePositive(
          shot.fallbackGamePiece.dragCoefficient,
          prefix + ".fallbackGamePiece.dragCoefficient",
          errors);
    }
  }

  private static void validateShotConstraints(
      String prefix, FieldProfileConfig.ShotConstraintsConfig constraints, List<String> errors) {
    if (constraints == null) {
      return;
    }
    requireOptionalNonNegative(
        constraints.minLaunchSpeedMetersPerSecond,
        prefix + ".minLaunchSpeedMetersPerSecond",
        errors);
    requireOptionalNonNegative(
        constraints.maxLaunchSpeedMetersPerSecond,
        prefix + ".maxLaunchSpeedMetersPerSecond",
        errors);
    requireOptionalNonNegative(
        constraints.minLaunchAngleDegrees, prefix + ".minLaunchAngleDegrees", errors);
    requireOptionalNonNegative(
        constraints.maxLaunchAngleDegrees, prefix + ".maxLaunchAngleDegrees", errors);
    if (constraints.minLaunchSpeedMetersPerSecond != null
        && constraints.maxLaunchSpeedMetersPerSecond != null
        && constraints.maxLaunchSpeedMetersPerSecond < constraints.minLaunchSpeedMetersPerSecond) {
      errors.add(
          prefix + ".maxLaunchSpeedMetersPerSecond must be >= minLaunchSpeedMetersPerSecond");
    }
    if (constraints.minLaunchAngleDegrees != null
        && constraints.maxLaunchAngleDegrees != null
        && constraints.maxLaunchAngleDegrees < constraints.minLaunchAngleDegrees) {
      errors.add(prefix + ".maxLaunchAngleDegrees must be >= minLaunchAngleDegrees");
    }
    if (constraints.shotStyle != null && !constraints.shotStyle.isBlank()) {
      try {
        org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints.ShotStyle.valueOf(
            constraints.shotStyle.trim().toUpperCase());
      } catch (IllegalArgumentException ex) {
        errors.add(prefix + ".shotStyle must be one of ANY, DIRECT, ARC");
      }
    }
  }

  private static void validateMovingShot(
      String prefix, FieldProfileConfig.MovingShotConfig movingShot, List<String> errors) {
    if (movingShot == null || Boolean.FALSE.equals(movingShot.enabled)) {
      return;
    }
    requireOptionalNonNegative(
        movingShot.releaseLatencySeconds, prefix + ".releaseLatencySeconds", errors);
    requireOptionalNonNegative(
        movingShot.minFlightPredictionSeconds, prefix + ".minFlightPredictionSeconds", errors);
    requireOptionalNonNegative(
        movingShot.maxFlightPredictionSeconds, prefix + ".maxFlightPredictionSeconds", errors);
    requireOptionalNonNegative(
        movingShot.defaultFlightPredictionSeconds,
        prefix + ".defaultFlightPredictionSeconds",
        errors);
    requireOptionalNonNegative(
        movingShot.maxCompensatedSpeedMetersPerSecond,
        prefix + ".maxCompensatedSpeedMetersPerSecond",
        errors);
    requireOptionalNonNegative(
        movingShot.maxReleaseSpeedMetersPerSecond,
        prefix + ".maxReleaseSpeedMetersPerSecond",
        errors);
    requireOptionalNonNegative(
        movingShot.yawToleranceDegrees, prefix + ".yawToleranceDegrees", errors);
    requireOptionalNonNegative(
        movingShot.maxVerticalErrorMeters, prefix + ".maxVerticalErrorMeters", errors);
    if (movingShot.iterations != null && movingShot.iterations <= 0) {
      errors.add(prefix + ".iterations must be > 0 when provided");
    }
    if (movingShot.minFlightPredictionSeconds != null
        && movingShot.maxFlightPredictionSeconds != null
        && movingShot.maxFlightPredictionSeconds < movingShot.minFlightPredictionSeconds) {
      errors.add(prefix + ".maxFlightPredictionSeconds must be >= minFlightPredictionSeconds");
    }
  }

  private static void requireOptionalPositive(Integer value, String name, List<String> errors) {
    if (value != null && value <= 0) {
      errors.add(name + " must be > 0 when provided");
    }
  }

  private static void requireOptionalPositive(Double value, String name, List<String> errors) {
    if (value != null && (!Double.isFinite(value) || value <= 0.0)) {
      errors.add(name + " must be finite and > 0 when provided");
    }
  }

  private static void requireOptionalNonNegative(Double value, String name, List<String> errors) {
    if (value != null && (!Double.isFinite(value) || value < 0.0)) {
      errors.add(name + " must be finite and >= 0 when provided");
    }
  }
}
