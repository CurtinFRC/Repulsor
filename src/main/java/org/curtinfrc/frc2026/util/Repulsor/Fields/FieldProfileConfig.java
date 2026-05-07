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

import java.util.LinkedHashMap;
import java.util.Map;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.AutoPathRuntimeConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.CoarseGlobalPlannerConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlannerRuntimeConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointPolicyProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PredictiveRankingConfig;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective.ObjectiveSelectionConfig;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.MovingShotSolver;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.RepulsorStrategyPreset;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.CollectObjectiveSelectionConfig;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.CollectPlannerTuning;

/**
 * Provides field profile config functionality for the Repulsor field/profile definition layer used
 * to tune Repulsor for a specific game. Use this type from robot code, field profiles, or tests
 * when integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
 * method documents robot-relative motion.
 */
public class FieldProfileConfig {
  /**
   * Configuration value for id. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public String id;

  /**
   * Configuration value for game name. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public String gameName;

  /**
   * Configuration value for game year. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public Integer gameYear;

  public GeometryConfig geometry = new GeometryConfig();
  public Map<String, ResourceConfig> resources = new LinkedHashMap<>();
  public Map<String, ProjectileShotConfig> projectileShots = new LinkedHashMap<>();
  public RankingConfig predictiveRanking = new RankingConfig();
  public ObjectiveSelectionProfileConfig objectiveSelection = new ObjectiveSelectionProfileConfig();
  public CollectPlannerConfig collectPlanner = new CollectPlannerConfig();
  public WaypointingConfig waypointing = new WaypointingConfig();
  public PlannerRuntimeConfig plannerRuntime = new PlannerRuntimeConfig();
  public AutoPathConfig autoPath = new AutoPathConfig();
  public String defaultStrategyPreset;
  public Map<String, StrategyPresetConfig> strategyPresets = new LinkedHashMap<>();

  /** Compatibility input for older 2026-specific YAML. Prefer projectileShots. */
  @Deprecated(forRemoval = false)
  public ShuttleShotConfig shuttleShot = new ShuttleShotConfig();

  public RebuiltCorridorConfig rebuiltCorridor = new RebuiltCorridorConfig();

  /**
   * Returns the field geometry value maintained by this Repulsor component.
   *
   * @param fallbackLengthMeters distance or field-coordinate value in meters.
   * @param fallbackWidthMeters distance or field-coordinate value in meters.
   * @return field geometry result for field geometry.
   */
  public FieldGeometry fieldGeometry(double fallbackLengthMeters, double fallbackWidthMeters) {
    return new FieldGeometry(
        finitePositive(geometry.lengthMeters, fallbackLengthMeters),
        finitePositive(geometry.widthMeters, fallbackWidthMeters));
  }

  /**
   * Returns the merge value maintained by this Repulsor component.
   *
   * @param base value used by this operation.
   * @param overlay distance or field-coordinate value in meters.
   * @return field profile config result for merge.
   */
  public static FieldProfileConfig merge(FieldProfileConfig base, FieldProfileConfig overlay) {
    if (base == null) return overlay;
    if (overlay == null) return base;

    if (overlay.id != null) base.id = overlay.id;
    if (overlay.gameName != null) base.gameName = overlay.gameName;
    if (overlay.gameYear != null) base.gameYear = overlay.gameYear;

    if (overlay.geometry != null) {
      if (overlay.geometry.lengthMeters != null)
        base.geometry.lengthMeters = overlay.geometry.lengthMeters;
      if (overlay.geometry.widthMeters != null)
        base.geometry.widthMeters = overlay.geometry.widthMeters;
    }

    if (overlay.resources != null && !overlay.resources.isEmpty()) {
      overlay.resources.forEach(
          (type, resource) -> {
            ResourceConfig target =
                base.resources.computeIfAbsent(type, ignored -> new ResourceConfig());
            if (resource.radiusMeters != null) target.radiusMeters = resource.radiusMeters;
            if (resource.unitValue != null) target.unitValue = resource.unitValue;
            if (resource.sigmaMeters != null) target.sigmaMeters = resource.sigmaMeters;
          });
    }

    if (overlay.projectileShots != null && !overlay.projectileShots.isEmpty()) {
      overlay.projectileShots.forEach(
          (id, shot) -> {
            ProjectileShotConfig target =
                base.projectileShots.computeIfAbsent(id, ignored -> new ProjectileShotConfig());
            mergeProjectileShot(target, shot);
          });
    }

    mergeRanking(base.predictiveRanking, overlay.predictiveRanking);
    mergeObjectiveSelection(base.objectiveSelection, overlay.objectiveSelection);
    mergeCollectPlanner(base.collectPlanner, overlay.collectPlanner);
    mergeWaypointing(base.waypointing, overlay.waypointing);
    mergePlannerRuntime(base.plannerRuntime, overlay.plannerRuntime);
    mergeAutoPath(base.autoPath, overlay.autoPath);
    if (overlay.defaultStrategyPreset != null)
      base.defaultStrategyPreset = overlay.defaultStrategyPreset;
    mergeStrategyPresets(base.strategyPresets, overlay.strategyPresets);

    mergeProjectileShot(base.shuttleShot, overlay.shuttleShot);
    mergeCorridor(base.rebuiltCorridor, overlay.rebuiltCorridor);
    return base;
  }

  private static void mergeStrategyPresets(
      Map<String, StrategyPresetConfig> base, Map<String, StrategyPresetConfig> overlay) {
    if (base == null || overlay == null || overlay.isEmpty()) return;
    overlay.forEach(
        (name, preset) -> {
          if (name == null || name.isBlank() || preset == null) return;
          StrategyPresetConfig target =
              base.computeIfAbsent(name, ignored -> new StrategyPresetConfig());
          mergeRanking(target.predictiveRanking, preset.predictiveRanking);
          mergeObjectiveSelection(target.objectiveSelection, preset.objectiveSelection);
          mergeCollectPlanner(target.collectPlanner, preset.collectPlanner);
          mergeWaypointing(target.waypointing, preset.waypointing);
          mergePlannerRuntime(target.plannerRuntime, preset.plannerRuntime);
          mergeAutoPath(target.autoPath, preset.autoPath);
        });
  }

  public Map<String, RepulsorStrategyPreset> toStrategyPresets() {
    if (strategyPresets == null || strategyPresets.isEmpty()) return Map.of();
    LinkedHashMap<String, RepulsorStrategyPreset> presets = new LinkedHashMap<>();
    strategyPresets.forEach(
        (name, preset) -> {
          if (name == null || name.isBlank() || preset == null) return;
          presets.put(name, preset.toRepulsorStrategyPreset(name, this));
        });
    return Map.copyOf(presets);
  }

  private static void mergeWaypointing(WaypointingConfig base, WaypointingConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.bandTransitionStagingEnabled != null) {
      base.bandTransitionStagingEnabled = overlay.bandTransitionStagingEnabled;
    }
    if (overlay.occludingGateStagingEnabled != null) {
      base.occludingGateStagingEnabled = overlay.occludingGateStagingEnabled;
    }
    if (overlay.centerReturnStagingEnabled != null) {
      base.centerReturnStagingEnabled = overlay.centerReturnStagingEnabled;
    }
    if (overlay.centerBandMeters != null) base.centerBandMeters = overlay.centerBandMeters;
    if (overlay.restageDistanceMeters != null) {
      base.restageDistanceMeters = overlay.restageDistanceMeters;
    }
    if (overlay.gatePaddingMeters != null) base.gatePaddingMeters = overlay.gatePaddingMeters;
    if (overlay.leadThroughScale != null) base.leadThroughScale = overlay.leadThroughScale;
    if (overlay.leadThroughMinMeters != null) {
      base.leadThroughMinMeters = overlay.leadThroughMinMeters;
    }
    if (overlay.leadThroughMaxMeters != null) {
      base.leadThroughMaxMeters = overlay.leadThroughMaxMeters;
    }
    if (overlay.deepCenterBandMeters != null) {
      base.deepCenterBandMeters = overlay.deepCenterBandMeters;
    }
    if (overlay.centerReturnStageTriggerMeters != null) {
      base.centerReturnStageTriggerMeters = overlay.centerReturnStageTriggerMeters;
    }
    if (overlay.centerReturnIntersectionTriggerMeters != null) {
      base.centerReturnIntersectionTriggerMeters = overlay.centerReturnIntersectionTriggerMeters;
    }
    if (overlay.centerReturnExitMinMeters != null) {
      base.centerReturnExitMinMeters = overlay.centerReturnExitMinMeters;
    }
    if (overlay.centerReturnExitMaxMeters != null) {
      base.centerReturnExitMaxMeters = overlay.centerReturnExitMaxMeters;
    }
    if (overlay.centerReturnGateMinOffsetMeters != null) {
      base.centerReturnGateMinOffsetMeters = overlay.centerReturnGateMinOffsetMeters;
    }
    if (overlay.fieldEdgeMarginMeters != null) {
      base.fieldEdgeMarginMeters = overlay.fieldEdgeMarginMeters;
    }
  }

  private static void mergeRanking(RankingConfig base, RankingConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.advantageGain != null) base.advantageGain = overlay.advantageGain;
    if (overlay.distanceCost != null) base.distanceCost = overlay.distanceCost;
    if (overlay.pressureCost != null) base.pressureCost = overlay.pressureCost;
    if (overlay.congestionCost != null) base.congestionCost = overlay.congestionCost;
    if (overlay.capacityGain != null) base.capacityGain = overlay.capacityGain;
    if (overlay.headingGain != null) base.headingGain = overlay.headingGain;
    if (overlay.hysteresisBonus != null) base.hysteresisBonus = overlay.hysteresisBonus;
    if (overlay.hysteresisPersistSeconds != null) {
      base.hysteresisPersistSeconds = overlay.hysteresisPersistSeconds;
    }
  }

  private static void mergeObjectiveSelection(
      ObjectiveSelectionProfileConfig base, ObjectiveSelectionProfileConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.candidateLimit != null) base.candidateLimit = overlay.candidateLimit;
    if (overlay.switchScoreMargin != null) base.switchScoreMargin = overlay.switchScoreMargin;
    if (overlay.holdCurrentWhenRanked != null) {
      base.holdCurrentWhenRanked = overlay.holdCurrentWhenRanked;
    }
  }

  private static void mergeCollectPlanner(CollectPlannerConfig base, CollectPlannerConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.groupCellMeters != null) base.groupCellMeters = overlay.groupCellMeters;
    if (overlay.nearbyRadiusMeters != null) base.nearbyRadiusMeters = overlay.nearbyRadiusMeters;
    if (overlay.liveObservationMaxAgeSeconds != null) {
      base.liveObservationMaxAgeSeconds = overlay.liveObservationMaxAgeSeconds;
    }
    if (overlay.predictorObservationMaxAgeSeconds != null) {
      base.predictorObservationMaxAgeSeconds = overlay.predictorObservationMaxAgeSeconds;
    }
    if (overlay.stickyNoProgressSeconds != null) {
      base.stickyNoProgressSeconds = overlay.stickyNoProgressSeconds;
    }
    if (overlay.switchCooldownSeconds != null) {
      base.switchCooldownSeconds = overlay.switchCooldownSeconds;
    }
    if (overlay.collectCellMeters != null) base.collectCellMeters = overlay.collectCellMeters;
    if (overlay.resourceUnitGain != null) base.resourceUnitGain = overlay.resourceUnitGain;
    if (overlay.etaCost != null) base.etaCost = overlay.etaCost;
    if (overlay.hubFrontTrapPenalty != null) base.hubFrontTrapPenalty = overlay.hubFrontTrapPenalty;
    if (overlay.canonicalScoreDropLimit != null) {
      base.canonicalScoreDropLimit = overlay.canonicalScoreDropLimit;
    }
    if (overlay.richerUnitsAbsGain != null) base.richerUnitsAbsGain = overlay.richerUnitsAbsGain;
    if (overlay.richerUnitsRelGain != null) base.richerUnitsRelGain = overlay.richerUnitsRelGain;
    if (overlay.richerEtaDeltaMaxSeconds != null) {
      base.richerEtaDeltaMaxSeconds = overlay.richerEtaDeltaMaxSeconds;
    }
    if (overlay.richerScoreDropLimit != null)
      base.richerScoreDropLimit = overlay.richerScoreDropLimit;
    if (overlay.liveFuelPreferScoreMargin != null) {
      base.liveFuelPreferScoreMargin = overlay.liveFuelPreferScoreMargin;
    }
    if (overlay.hubFrontTrapEscapeScoreAllowDrop != null) {
      base.hubFrontTrapEscapeScoreAllowDrop = overlay.hubFrontTrapEscapeScoreAllowDrop;
    }
    if (overlay.nearbyCentroidScoreDropLimit != null) {
      base.nearbyCentroidScoreDropLimit = overlay.nearbyCentroidScoreDropLimit;
    }
    if (overlay.liveRelockScoreDropLimit != null) {
      base.liveRelockScoreDropLimit = overlay.liveRelockScoreDropLimit;
    }
    if (overlay.stickyPreferRankedScoreMargin != null) {
      base.stickyPreferRankedScoreMargin = overlay.stickyPreferRankedScoreMargin;
    }
    if (overlay.farSwitchLockDistanceMeters != null) {
      base.farSwitchLockDistanceMeters = overlay.farSwitchLockDistanceMeters;
    }
    if (overlay.farSwitchForceMultiplier != null) {
      base.farSwitchForceMultiplier = overlay.farSwitchForceMultiplier;
    }
    if (overlay.closeSwitchEasyDistanceMeters != null) {
      base.closeSwitchEasyDistanceMeters = overlay.closeSwitchEasyDistanceMeters;
    }
    if (overlay.closeSwitchMarginScale != null) {
      base.closeSwitchMarginScale = overlay.closeSwitchMarginScale;
    }
  }

  private static void mergePlannerRuntime(PlannerRuntimeConfig base, PlannerRuntimeConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.globalFallbackEnabled != null) {
      base.globalFallbackEnabled = overlay.globalFallbackEnabled;
    }
    if (overlay.globalFallbackCellMeters != null) {
      base.globalFallbackCellMeters = overlay.globalFallbackCellMeters;
    }
    if (overlay.globalFallbackLookaheadMeters != null) {
      base.globalFallbackLookaheadMeters = overlay.globalFallbackLookaheadMeters;
    }
    if (overlay.globalFallbackMaxExpandedNodes != null) {
      base.globalFallbackMaxExpandedNodes = overlay.globalFallbackMaxExpandedNodes;
    }
    if (overlay.globalFallbackMaxRuntimeSeconds != null) {
      base.globalFallbackMaxRuntimeSeconds = overlay.globalFallbackMaxRuntimeSeconds;
    }
    if (overlay.forceThroughGoalDistanceMeters != null) {
      base.forceThroughGoalDistanceMeters = overlay.forceThroughGoalDistanceMeters;
    }
    if (overlay.forceThroughWallDistanceMeters != null) {
      base.forceThroughWallDistanceMeters = overlay.forceThroughWallDistanceMeters;
    }
  }

  private static void mergeAutoPath(AutoPathConfig base, AutoPathConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.episodeCooldownSeconds != null) {
      base.episodeCooldownSeconds = overlay.episodeCooldownSeconds;
    }
    if (overlay.pinnedFailSeconds != null) base.pinnedFailSeconds = overlay.pinnedFailSeconds;
    if (overlay.stuckFailSeconds != null) base.stuckFailSeconds = overlay.stuckFailSeconds;
    if (overlay.progressEpsilonMeters != null) {
      base.progressEpsilonMeters = overlay.progressEpsilonMeters;
    }
    if (overlay.pinnedProgressMinMeters != null) {
      base.pinnedProgressMinMeters = overlay.pinnedProgressMinMeters;
    }
    if (overlay.stuckDistanceMinMeters != null) {
      base.stuckDistanceMinMeters = overlay.stuckDistanceMinMeters;
    }
    if (overlay.successNearDistanceMeters != null) {
      base.successNearDistanceMeters = overlay.successNearDistanceMeters;
    }
    if (overlay.collectGoalUnits != null) base.collectGoalUnits = overlay.collectGoalUnits;
    if (overlay.shootLockEnterMeters != null) {
      base.shootLockEnterMeters = overlay.shootLockEnterMeters;
    }
    if (overlay.shootLockExitMeters != null) base.shootLockExitMeters = overlay.shootLockExitMeters;
    if (overlay.shootLockMinRotationDegrees != null) {
      base.shootLockMinRotationDegrees = overlay.shootLockMinRotationDegrees;
    }
    if (overlay.shootReadyPositionToleranceMeters != null) {
      base.shootReadyPositionToleranceMeters = overlay.shootReadyPositionToleranceMeters;
    }
    if (overlay.shootReadyRotationToleranceDegrees != null) {
      base.shootReadyRotationToleranceDegrees = overlay.shootReadyRotationToleranceDegrees;
    }
    if (overlay.collectHoldGoalNearMeters != null) {
      base.collectHoldGoalNearMeters = overlay.collectHoldGoalNearMeters;
    }
    if (overlay.collectFarResourceMinDistanceMeters != null) {
      base.collectFarResourceMinDistanceMeters = overlay.collectFarResourceMinDistanceMeters;
    }
  }

  private static void mergeProjectileShot(ProjectileShotConfig base, ProjectileShotConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.enabled != null) base.enabled = overlay.enabled;
    if (overlay.role != null) base.role = overlay.role;
    if (overlay.gamePieceId != null) base.gamePieceId = overlay.gamePieceId;
    mergeTarget(base.target, overlay.target);
    if (overlay.targetHeightMeters != null) base.targetHeightMeters = overlay.targetHeightMeters;
    mergeConstraints(base.constraints, overlay.constraints);
    if (overlay.routeLevel != null) base.routeLevel = overlay.routeLevel;
    if (overlay.routeMechanismSetpoint != null)
      base.routeMechanismSetpoint = overlay.routeMechanismSetpoint;
    if (overlay.behindTargetMeters != null) base.behindTargetMeters = overlay.behindTargetMeters;
    if (overlay.lateralOffsetsMeters != null)
      base.lateralOffsetsMeters = overlay.lateralOffsetsMeters;
    if (overlay.fieldMarginMeters != null) base.fieldMarginMeters = overlay.fieldMarginMeters;
    mergeMovingShot(base.movingShot, overlay.movingShot);
    if (overlay.fallbackGamePiece != null) {
      if (overlay.fallbackGamePiece.massKg != null) {
        base.fallbackGamePiece.massKg = overlay.fallbackGamePiece.massKg;
      }
      if (overlay.fallbackGamePiece.crossSectionAreaM2 != null) {
        base.fallbackGamePiece.crossSectionAreaM2 = overlay.fallbackGamePiece.crossSectionAreaM2;
      }
      if (overlay.fallbackGamePiece.dragCoefficient != null) {
        base.fallbackGamePiece.dragCoefficient = overlay.fallbackGamePiece.dragCoefficient;
      }
    }
  }

  private static void mergeConstraints(ShotConstraintsConfig base, ShotConstraintsConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.minLaunchSpeedMetersPerSecond != null)
      base.minLaunchSpeedMetersPerSecond = overlay.minLaunchSpeedMetersPerSecond;
    if (overlay.maxLaunchSpeedMetersPerSecond != null)
      base.maxLaunchSpeedMetersPerSecond = overlay.maxLaunchSpeedMetersPerSecond;
    if (overlay.minLaunchAngleDegrees != null)
      base.minLaunchAngleDegrees = overlay.minLaunchAngleDegrees;
    if (overlay.maxLaunchAngleDegrees != null)
      base.maxLaunchAngleDegrees = overlay.maxLaunchAngleDegrees;
    if (overlay.shotStyle != null) base.shotStyle = overlay.shotStyle;
  }

  private static void mergeMovingShot(MovingShotConfig base, MovingShotConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.enabled != null) base.enabled = overlay.enabled;
    if (overlay.releaseLatencySeconds != null)
      base.releaseLatencySeconds = overlay.releaseLatencySeconds;
    if (overlay.minFlightPredictionSeconds != null)
      base.minFlightPredictionSeconds = overlay.minFlightPredictionSeconds;
    if (overlay.maxFlightPredictionSeconds != null)
      base.maxFlightPredictionSeconds = overlay.maxFlightPredictionSeconds;
    if (overlay.defaultFlightPredictionSeconds != null)
      base.defaultFlightPredictionSeconds = overlay.defaultFlightPredictionSeconds;
    if (overlay.maxCompensatedSpeedMetersPerSecond != null)
      base.maxCompensatedSpeedMetersPerSecond = overlay.maxCompensatedSpeedMetersPerSecond;
    if (overlay.maxReleaseSpeedMetersPerSecond != null)
      base.maxReleaseSpeedMetersPerSecond = overlay.maxReleaseSpeedMetersPerSecond;
    if (overlay.yawToleranceDegrees != null) base.yawToleranceDegrees = overlay.yawToleranceDegrees;
    if (overlay.maxVerticalErrorMeters != null)
      base.maxVerticalErrorMeters = overlay.maxVerticalErrorMeters;
    if (overlay.iterations != null) base.iterations = overlay.iterations;
  }

  private static void mergeTarget(TargetConfig base, TargetConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.kind != null) base.kind = overlay.kind;
    if (overlay.blueXMeters != null) base.blueXMeters = overlay.blueXMeters;
    if (overlay.blueYMeters != null) base.blueYMeters = overlay.blueYMeters;
  }

  private static void mergeCorridor(RebuiltCorridorConfig base, RebuiltCorridorConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.rectWidthMeters != null) base.rectWidthMeters = overlay.rectWidthMeters;
    if (overlay.rectHeightMeters != null) base.rectHeightMeters = overlay.rectHeightMeters;
    if (overlay.rectOffsetFromCenterMeters != null) {
      base.rectOffsetFromCenterMeters = overlay.rectOffsetFromCenterMeters;
    }
    if (overlay.edgeOffsetMeters != null) base.edgeOffsetMeters = overlay.edgeOffsetMeters;
    if (overlay.rectStrength != null) base.rectStrength = overlay.rectStrength;
    if (overlay.rectRangeXMeters != null) base.rectRangeXMeters = overlay.rectRangeXMeters;
    if (overlay.rectRangeYMeters != null) base.rectRangeYMeters = overlay.rectRangeYMeters;
    if (overlay.biasStrength != null) base.biasStrength = overlay.biasStrength;
    if (overlay.biasRangeMeters != null) base.biasRangeMeters = overlay.biasRangeMeters;
    if (overlay.bypassStrengthScale != null) base.bypassStrengthScale = overlay.bypassStrengthScale;
    if (overlay.bypassRangeMeters != null) base.bypassRangeMeters = overlay.bypassRangeMeters;
    if (overlay.sidePullDxMeters != null) base.sidePullDxMeters = overlay.sidePullDxMeters;
    if (overlay.sideBiasStrengthScale != null) {
      base.sideBiasStrengthScale = overlay.sideBiasStrengthScale;
    }
    if (overlay.sideBiasRangeScale != null) base.sideBiasRangeScale = overlay.sideBiasRangeScale;
    if (overlay.sideBypassStrengthScale != null) {
      base.sideBypassStrengthScale = overlay.sideBypassStrengthScale;
    }
    if (overlay.sideBypassRangeScale != null) {
      base.sideBypassRangeScale = overlay.sideBypassRangeScale;
    }
    if (overlay.railXWindowMeters != null) base.railXWindowMeters = overlay.railXWindowMeters;
    if (overlay.railMinHalfWidthMeters != null) {
      base.railMinHalfWidthMeters = overlay.railMinHalfWidthMeters;
    }
    if (overlay.railHalfWidthGapScale != null) {
      base.railHalfWidthGapScale = overlay.railHalfWidthGapScale;
    }
    if (overlay.railStrength != null) base.railStrength = overlay.railStrength;
    if (overlay.railMaxForce != null) base.railMaxForce = overlay.railMaxForce;
    if (overlay.centerRailMinWindowMeters != null) {
      base.centerRailMinWindowMeters = overlay.centerRailMinWindowMeters;
    }
    if (overlay.centerRailWindowScale != null) {
      base.centerRailWindowScale = overlay.centerRailWindowScale;
    }
    if (overlay.outerRailXOffsetScale != null) {
      base.outerRailXOffsetScale = overlay.outerRailXOffsetScale;
    }
    if (overlay.outerRailWindowMeters != null) {
      base.outerRailWindowMeters = overlay.outerRailWindowMeters;
    }
  }

  /**
   * Returns the finite positive value maintained by this Repulsor component.
   *
   * @param value value used by this operation.
   * @param fallback value used by this operation.
   * @return value produced by this operation.
   */
  static double finitePositive(Double value, double fallback) {
    return value != null && Double.isFinite(value) && value > 0.0 ? value : fallback;
  }

  /**
   * Returns the positive value maintained by this Repulsor component.
   *
   * @param value value used by this operation.
   * @param fallback value used by this operation.
   * @return value produced by this operation.
   */
  static int positive(Integer value, int fallback) {
    return value != null && value > 0 ? value : fallback;
  }

  static boolean boolOrDefault(Boolean value, boolean fallback) {
    return value != null ? value : fallback;
  }

  /**
   * Provides geometry config functionality for the Repulsor field/profile definition layer used to
   * tune Repulsor for a specific game. Use this type from robot code, field profiles, or tests when
   * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
   * method documents robot-relative motion.
   */
  public static class GeometryConfig {
    /**
     * Configuration value for length meters. Distances use meters in WPILib field coordinates and
     * should be treated as tunable when sourced from profiles.
     */
    public Double lengthMeters;

    /**
     * Configuration value for width meters. Distances use meters in WPILib field coordinates and
     * should be treated as tunable when sourced from profiles.
     */
    public Double widthMeters;
  }

  /**
   * Provides resource config functionality for the Repulsor field/profile definition layer used to
   * tune Repulsor for a specific game. Use this type from robot code, field profiles, or tests when
   * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
   * method documents robot-relative motion.
   */
  public static class ResourceConfig {
    /**
     * Configuration value for radius meters. Distances use meters in WPILib field coordinates and
     * should be treated as tunable when sourced from profiles.
     */
    public Double radiusMeters;

    /**
     * Configuration value for unit value. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public Double unitValue;

    /**
     * Configuration value for sigma meters. Distances use meters in WPILib field coordinates and
     * should be treated as tunable when sourced from profiles.
     */
    public Double sigmaMeters;
  }

  public static class RankingConfig {
    public Double advantageGain;
    public Double distanceCost;
    public Double pressureCost;
    public Double congestionCost;
    public Double capacityGain;
    public Double headingGain;
    public Double hysteresisBonus;
    public Double hysteresisPersistSeconds;

    public PredictiveRankingConfig toPredictiveRankingConfig() {
      PredictiveRankingConfig defaults = PredictiveRankingConfig.defaults();
      return new PredictiveRankingConfig(
          finiteNonNegative(advantageGain, defaults.advantageGain()),
          finiteNonNegative(distanceCost, defaults.distanceCost()),
          finiteNonNegative(pressureCost, defaults.pressureCost()),
          finiteNonNegative(congestionCost, defaults.congestionCost()),
          finiteNonNegative(capacityGain, defaults.capacityGain()),
          finiteNonNegative(headingGain, defaults.headingGain()),
          finiteNonNegative(hysteresisBonus, defaults.hysteresisBonus()),
          finiteNonNegative(hysteresisPersistSeconds, defaults.hysteresisPersistSeconds()));
    }
  }

  public static class ObjectiveSelectionProfileConfig {
    public Integer candidateLimit;
    public Double switchScoreMargin;
    public Boolean holdCurrentWhenRanked;

    public ObjectiveSelectionConfig toObjectiveSelectionConfig() {
      ObjectiveSelectionConfig defaults = ObjectiveSelectionConfig.defaults();
      return new ObjectiveSelectionConfig(
          positive(candidateLimit, defaults.candidateLimit()),
          finiteNonNegative(switchScoreMargin, defaults.switchScoreMargin()),
          boolOrDefault(holdCurrentWhenRanked, defaults.holdCurrentWhenRanked()));
    }
  }

  public static class CollectPlannerConfig {
    public Double groupCellMeters;
    public Double nearbyRadiusMeters;
    public Double liveObservationMaxAgeSeconds;
    public Double predictorObservationMaxAgeSeconds;
    public Double stickyNoProgressSeconds;
    public Double switchCooldownSeconds;
    public Double collectCellMeters;
    public Double resourceUnitGain;
    public Double etaCost;
    public Double hubFrontTrapPenalty;
    public Double canonicalScoreDropLimit;
    public Double richerUnitsAbsGain;
    public Double richerUnitsRelGain;
    public Double richerEtaDeltaMaxSeconds;
    public Double richerScoreDropLimit;
    public Double liveFuelPreferScoreMargin;
    public Double hubFrontTrapEscapeScoreAllowDrop;
    public Double nearbyCentroidScoreDropLimit;
    public Double liveRelockScoreDropLimit;
    public Double stickyPreferRankedScoreMargin;
    public Double farSwitchLockDistanceMeters;
    public Double farSwitchForceMultiplier;
    public Double closeSwitchEasyDistanceMeters;
    public Double closeSwitchMarginScale;

    public CollectPlannerTuning toCollectPlannerTuning() {
      CollectPlannerTuning defaults = CollectPlannerTuning.defaults();
      CollectObjectiveSelectionConfig selectionDefaults = defaults.selection();
      CollectObjectiveSelectionConfig selection =
          new CollectObjectiveSelectionConfig(
              finiteNonNegative(resourceUnitGain, selectionDefaults.resourceUnitGain()),
              finiteNonNegative(etaCost, selectionDefaults.etaCost()),
              finiteNonNegative(hubFrontTrapPenalty, selectionDefaults.hubFrontTrapPenalty()),
              finiteNonNegative(
                  canonicalScoreDropLimit, selectionDefaults.canonicalScoreDropLimit()),
              finiteNonNegative(richerUnitsAbsGain, selectionDefaults.richerUnitsAbsGain()),
              finiteNonNegative(richerUnitsRelGain, selectionDefaults.richerUnitsRelGain()),
              finiteNonNegative(
                  richerEtaDeltaMaxSeconds, selectionDefaults.richerEtaDeltaMaxSeconds()),
              finiteNonNegative(richerScoreDropLimit, selectionDefaults.richerScoreDropLimit()),
              finiteNonNegative(
                  liveFuelPreferScoreMargin, selectionDefaults.liveFuelPreferScoreMargin()),
              finiteNonNegative(
                  hubFrontTrapEscapeScoreAllowDrop,
                  selectionDefaults.hubFrontTrapEscapeScoreAllowDrop()),
              finiteNonNegative(
                  nearbyCentroidScoreDropLimit, selectionDefaults.nearbyCentroidScoreDropLimit()),
              finiteNonNegative(
                  liveRelockScoreDropLimit, selectionDefaults.liveRelockScoreDropLimit()),
              finiteNonNegative(
                  stickyPreferRankedScoreMargin, selectionDefaults.stickyPreferRankedScoreMargin()),
              finiteNonNegative(
                  farSwitchLockDistanceMeters, selectionDefaults.farSwitchLockDistanceMeters()),
              finiteNonNegative(
                  farSwitchForceMultiplier, selectionDefaults.farSwitchForceMultiplier()),
              finiteNonNegative(
                  closeSwitchEasyDistanceMeters, selectionDefaults.closeSwitchEasyDistanceMeters()),
              finiteNonNegative(
                  closeSwitchMarginScale, selectionDefaults.closeSwitchMarginScale()));
      return new CollectPlannerTuning(
          finitePositive(groupCellMeters, defaults.groupCellMeters()),
          finitePositive(nearbyRadiusMeters, defaults.nearbyRadiusMeters()),
          finiteNonNegative(liveObservationMaxAgeSeconds, defaults.liveObservationMaxAgeSeconds()),
          finiteNonNegative(
              predictorObservationMaxAgeSeconds, defaults.predictorObservationMaxAgeSeconds()),
          finiteNonNegative(stickyNoProgressSeconds, defaults.stickyNoProgressSeconds()),
          finiteNonNegative(switchCooldownSeconds, defaults.switchCooldownSeconds()),
          finitePositive(collectCellMeters, defaults.collectCellMeters()),
          selection);
    }
  }

  public static class WaypointingConfig {
    public Boolean bandTransitionStagingEnabled;
    public Boolean occludingGateStagingEnabled;
    public Boolean centerReturnStagingEnabled;
    public Double centerBandMeters;
    public Double restageDistanceMeters;
    public Double gatePaddingMeters;
    public Double leadThroughScale;
    public Double leadThroughMinMeters;
    public Double leadThroughMaxMeters;
    public Double deepCenterBandMeters;
    public Double centerReturnStageTriggerMeters;
    public Double centerReturnIntersectionTriggerMeters;
    public Double centerReturnExitMinMeters;
    public Double centerReturnExitMaxMeters;
    public Double centerReturnGateMinOffsetMeters;
    public Double fieldEdgeMarginMeters;

    public FieldPlannerWaypointConfig toFieldPlannerWaypointConfig() {
      FieldPlannerWaypointConfig defaults = FieldPlannerWaypointConfig.defaults();
      return new FieldPlannerWaypointConfig(
          boolOrDefault(bandTransitionStagingEnabled, defaults.bandTransitionStagingEnabled()),
          boolOrDefault(occludingGateStagingEnabled, defaults.occludingGateStagingEnabled()),
          boolOrDefault(centerReturnStagingEnabled, defaults.centerReturnStagingEnabled()),
          finiteNonNegative(centerBandMeters, defaults.centerBandMeters()),
          finiteNonNegative(restageDistanceMeters, defaults.restageDistanceMeters()),
          finiteNonNegative(gatePaddingMeters, defaults.gatePaddingMeters()),
          finiteNonNegative(leadThroughScale, defaults.leadThroughScale()),
          finiteNonNegative(leadThroughMinMeters, defaults.leadThroughMinMeters()),
          finiteNonNegative(leadThroughMaxMeters, defaults.leadThroughMaxMeters()),
          finiteNonNegative(deepCenterBandMeters, defaults.deepCenterBandMeters()),
          finiteNonNegative(
              centerReturnStageTriggerMeters, defaults.centerReturnStageTriggerMeters()),
          finiteNonNegative(
              centerReturnIntersectionTriggerMeters,
              defaults.centerReturnIntersectionTriggerMeters()),
          finiteNonNegative(centerReturnExitMinMeters, defaults.centerReturnExitMinMeters()),
          finiteNonNegative(centerReturnExitMaxMeters, defaults.centerReturnExitMaxMeters()),
          finiteNonNegative(
              centerReturnGateMinOffsetMeters, defaults.centerReturnGateMinOffsetMeters()),
          finiteNonNegative(fieldEdgeMarginMeters, defaults.fieldEdgeMarginMeters()));
    }
  }

  public static class AutoPathConfig {
    public Double episodeCooldownSeconds;
    public Double pinnedFailSeconds;
    public Double stuckFailSeconds;
    public Double progressEpsilonMeters;
    public Double pinnedProgressMinMeters;
    public Double stuckDistanceMinMeters;
    public Double successNearDistanceMeters;
    public Integer collectGoalUnits;
    public Double shootLockEnterMeters;
    public Double shootLockExitMeters;
    public Double shootLockMinRotationDegrees;
    public Double shootReadyPositionToleranceMeters;
    public Double shootReadyRotationToleranceDegrees;
    public Double collectHoldGoalNearMeters;
    public Double collectFarResourceMinDistanceMeters;

    public AutoPathRuntimeConfig toAutoPathRuntimeConfig() {
      AutoPathRuntimeConfig defaults = AutoPathRuntimeConfig.defaults();
      return new AutoPathRuntimeConfig(
          finiteNonNegative(episodeCooldownSeconds, defaults.episodeCooldownSeconds()),
          finiteNonNegative(pinnedFailSeconds, defaults.pinnedFailSeconds()),
          finiteNonNegative(stuckFailSeconds, defaults.stuckFailSeconds()),
          finiteNonNegative(progressEpsilonMeters, defaults.progressEpsilonMeters()),
          finiteNonNegative(pinnedProgressMinMeters, defaults.pinnedProgressMinMeters()),
          finiteNonNegative(stuckDistanceMinMeters, defaults.stuckDistanceMinMeters()),
          finiteNonNegative(successNearDistanceMeters, defaults.successNearDistanceMeters()),
          positive(collectGoalUnits, defaults.collectGoalUnits()),
          finiteNonNegative(shootLockEnterMeters, defaults.shootLockEnterMeters()),
          finiteNonNegative(shootLockExitMeters, defaults.shootLockExitMeters()),
          finiteNonNegative(shootLockMinRotationDegrees, defaults.shootLockMinRotationDegrees()),
          finiteNonNegative(
              shootReadyPositionToleranceMeters, defaults.shootReadyPositionToleranceMeters()),
          finiteNonNegative(
              shootReadyRotationToleranceDegrees, defaults.shootReadyRotationToleranceDegrees()),
          finiteNonNegative(collectHoldGoalNearMeters, defaults.collectHoldGoalNearMeters()),
          finiteNonNegative(
              collectFarResourceMinDistanceMeters, defaults.collectFarResourceMinDistanceMeters()));
    }
  }

  public static class PlannerRuntimeConfig {
    public Boolean globalFallbackEnabled;
    public Double globalFallbackCellMeters;
    public Double globalFallbackLookaheadMeters;
    public Integer globalFallbackMaxExpandedNodes;
    public Double globalFallbackMaxRuntimeSeconds;
    public Double forceThroughGoalDistanceMeters;
    public Double forceThroughWallDistanceMeters;

    public FieldPlannerRuntimeConfig toFieldPlannerRuntimeConfig() {
      FieldPlannerRuntimeConfig defaults = FieldPlannerRuntimeConfig.defaults();
      CoarseGlobalPlannerConfig globalDefaults = defaults.globalFallbackConfig();
      CoarseGlobalPlannerConfig globalConfig =
          new CoarseGlobalPlannerConfig(
              finitePositive(globalFallbackCellMeters, globalDefaults.cellMeters()),
              finitePositive(
                  globalFallbackLookaheadMeters, globalDefaults.waypointLookaheadMeters()),
              positive(globalFallbackMaxExpandedNodes, globalDefaults.maxExpandedNodes()),
              finitePositive(globalFallbackMaxRuntimeSeconds, globalDefaults.maxRuntimeSeconds()));
      return new FieldPlannerRuntimeConfig(
          globalFallbackEnabled == null ? defaults.globalFallbackEnabled() : globalFallbackEnabled,
          globalConfig,
          finiteNonNegative(
              forceThroughGoalDistanceMeters, defaults.forceThroughGoalDistanceMeters()),
          finiteNonNegative(
              forceThroughWallDistanceMeters, defaults.forceThroughWallDistanceMeters()));
    }
  }

  public static class StrategyPresetConfig {
    public RankingConfig predictiveRanking = new RankingConfig();
    public ObjectiveSelectionProfileConfig objectiveSelection =
        new ObjectiveSelectionProfileConfig();
    public CollectPlannerConfig collectPlanner = new CollectPlannerConfig();
    public WaypointingConfig waypointing = new WaypointingConfig();
    public PlannerRuntimeConfig plannerRuntime = new PlannerRuntimeConfig();
    public AutoPathConfig autoPath = new AutoPathConfig();

    public RepulsorStrategyPreset toRepulsorStrategyPreset(String name, FieldProfileConfig base) {
      RankingConfig ranking = new RankingConfig();
      mergeRanking(ranking, base == null ? null : base.predictiveRanking);
      mergeRanking(ranking, predictiveRanking);

      ObjectiveSelectionProfileConfig objective = new ObjectiveSelectionProfileConfig();
      mergeObjectiveSelection(objective, base == null ? null : base.objectiveSelection);
      mergeObjectiveSelection(objective, objectiveSelection);

      CollectPlannerConfig collect = new CollectPlannerConfig();
      mergeCollectPlanner(collect, base == null ? null : base.collectPlanner);
      mergeCollectPlanner(collect, collectPlanner);

      WaypointingConfig waypoint = new WaypointingConfig();
      mergeWaypointing(waypoint, base == null ? null : base.waypointing);
      mergeWaypointing(waypoint, waypointing);

      PlannerRuntimeConfig planner = new PlannerRuntimeConfig();
      mergePlannerRuntime(planner, base == null ? null : base.plannerRuntime);
      mergePlannerRuntime(planner, plannerRuntime);

      AutoPathConfig auto = new AutoPathConfig();
      mergeAutoPath(auto, base == null ? null : base.autoPath);
      mergeAutoPath(auto, autoPath);

      return new RepulsorStrategyPreset(
          name,
          collect.toCollectPlannerTuning(),
          ranking.toPredictiveRankingConfig(),
          objective.toObjectiveSelectionConfig(),
          planner.toFieldPlannerRuntimeConfig(),
          new FieldPlannerWaypointPolicyProfile(
              name, waypoint.toFieldPlannerWaypointConfig(), null),
          auto.toAutoPathRuntimeConfig());
    }
  }

  /**
   * Provides projectile shot config functionality for the Repulsor field/profile definition layer
   * used to tune Repulsor for a specific game. Use this type from robot code, field profiles, or
   * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
   * unless a method documents robot-relative motion.
   */
  public static class ProjectileShotConfig {
    /**
     * Configuration value for enabled. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public Boolean enabled;

    /**
     * Configuration value for role. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public String role;

    /**
     * Configuration value for game piece id. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public String gamePieceId;

    public TargetConfig target = new TargetConfig();

    /**
     * Configuration value for target height meters. Distances use meters in WPILib field
     * coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double targetHeightMeters;

    public ShotConstraintsConfig constraints = new ShotConstraintsConfig();

    /**
     * Configuration value for route level. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public String routeLevel;

    /**
     * Configuration value for route mechanism setpoint. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public String routeMechanismSetpoint;

    /**
     * Configuration value for behind target meters. Distances use meters in WPILib field
     * coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double behindTargetMeters;

    /**
     * Configuration value for lateral offsets meters. Distances use meters in WPILib field
     * coordinates and should be treated as tunable when sourced from profiles.
     */
    public double[] lateralOffsetsMeters;

    /**
     * Configuration value for field margin meters. Distances use meters in WPILib field coordinates
     * and should be treated as tunable when sourced from profiles.
     */
    public Double fieldMarginMeters;

    public MovingShotConfig movingShot = new MovingShotConfig();
    public GamePiecePhysicsConfig fallbackGamePiece = new GamePiecePhysicsConfig();
  }

  /** Compatibility config for older profile YAML. Prefer projectileShots. */
  public static class ShuttleShotConfig extends ProjectileShotConfig {}

  /**
   * Provides target config functionality for the Repulsor field/profile definition layer used to
   * tune Repulsor for a specific game. Use this type from robot code, field profiles, or tests when
   * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
   * method documents robot-relative motion.
   */
  public static class TargetConfig {
    /**
     * Configuration value for kind. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public String kind;

    /**
     * Configuration value for blue xmeters. Distances use meters in WPILib field coordinates and
     * should be treated as tunable when sourced from profiles.
     */
    public Double blueXMeters;

    /**
     * Configuration value for blue ymeters. Distances use meters in WPILib field coordinates and
     * should be treated as tunable when sourced from profiles.
     */
    public Double blueYMeters;
  }

  /**
   * Provides game piece physics config functionality for the Repulsor field/profile definition
   * layer used to tune Repulsor for a specific game. Use this type from robot code, field profiles,
   * or tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
   * unless a method documents robot-relative motion.
   */
  public static class GamePiecePhysicsConfig {
    /**
     * Configuration value for mass kg. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public Double massKg;

    /**
     * Configuration value for cross section area m2. Time values use seconds and should be tuned
     * against measured robot loop and mechanism latency.
     */
    public Double crossSectionAreaM2;

    /**
     * Configuration value for drag coefficient. The valid range and tuning source are defined by
     * the owning subsystem or field profile.
     */
    public Double dragCoefficient;
  }

  /**
   * Provides shot constraints config functionality for the Repulsor field/profile definition layer
   * used to tune Repulsor for a specific game. Use this type from robot code, field profiles, or
   * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
   * unless a method documents robot-relative motion.
   */
  public static class ShotConstraintsConfig {
    /**
     * Configuration value for min launch speed meters per second. Distances use meters in WPILib
     * field coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double minLaunchSpeedMetersPerSecond;

    /**
     * Configuration value for max launch speed meters per second. Distances use meters in WPILib
     * field coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double maxLaunchSpeedMetersPerSecond;

    /**
     * Configuration value for min launch angle degrees. Angles use WPILib rotation conventions;
     * names ending in degrees are degrees, otherwise radians are assumed by the API.
     */
    public Double minLaunchAngleDegrees;

    /**
     * Configuration value for max launch angle degrees. Angles use WPILib rotation conventions;
     * names ending in degrees are degrees, otherwise radians are assumed by the API.
     */
    public Double maxLaunchAngleDegrees;

    /**
     * Configuration value for shot style. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public String shotStyle;

    /**
     * Returns the constraints value maintained by this Repulsor component.
     *
     * @param fallback value used by this operation.
     * @return org.curtinfrc.frc2026.util.repulsor.shooting.constraints result for constraints.
     */
    public org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints constraints(
        org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints fallback) {
      org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints base =
          fallback == null
              ? new org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints(0.0, 30.0, 0.0, 90.0)
              : fallback;
      return new org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints(
          finiteNonNegative(minLaunchSpeedMetersPerSecond, base.minLaunchSpeedMetersPerSecond()),
          finiteNonNegative(maxLaunchSpeedMetersPerSecond, base.maxLaunchSpeedMetersPerSecond()),
          finiteNonNegative(minLaunchAngleDegrees, base.minLaunchAngleDeg()),
          finiteNonNegative(maxLaunchAngleDegrees, base.maxLaunchAngleDeg()),
          shotStyle(shotStyle, base.shotStyle()));
    }

    private static org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints.ShotStyle shotStyle(
        String value, org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints.ShotStyle fallback) {
      if (value == null || value.isBlank()) {
        return fallback;
      }
      try {
        return org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints.ShotStyle.valueOf(
            value.trim().toUpperCase());
      } catch (IllegalArgumentException ex) {
        return fallback;
      }
    }
  }

  /**
   * Provides moving shot config functionality for the Repulsor field/profile definition layer used
   * to tune Repulsor for a specific game. Use this type from robot code, field profiles, or tests
   * when integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
   * method documents robot-relative motion.
   */
  public static class MovingShotConfig {
    /**
     * Configuration value for enabled. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public Boolean enabled;

    /**
     * Configuration value for release latency seconds. Time values use seconds and should be tuned
     * against measured robot loop and mechanism latency.
     */
    public Double releaseLatencySeconds;

    /**
     * Configuration value for min flight prediction seconds. Time values use seconds and should be
     * tuned against measured robot loop and mechanism latency.
     */
    public Double minFlightPredictionSeconds;

    /**
     * Configuration value for max flight prediction seconds. Time values use seconds and should be
     * tuned against measured robot loop and mechanism latency.
     */
    public Double maxFlightPredictionSeconds;

    /**
     * Configuration value for default flight prediction seconds. Time values use seconds and should
     * be tuned against measured robot loop and mechanism latency.
     */
    public Double defaultFlightPredictionSeconds;

    /**
     * Configuration value for max compensated speed meters per second. Distances use meters in
     * WPILib field coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double maxCompensatedSpeedMetersPerSecond;

    /**
     * Configuration value for max release speed meters per second. Distances use meters in WPILib
     * field coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double maxReleaseSpeedMetersPerSecond;

    /**
     * Configuration value for yaw tolerance degrees. Angles use WPILib rotation conventions; names
     * ending in degrees are degrees, otherwise radians are assumed by the API.
     */
    public Double yawToleranceDegrees;

    /**
     * Configuration value for max vertical error meters. Distances use meters in WPILib field
     * coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double maxVerticalErrorMeters;

    /**
     * Configuration value for iterations. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public Integer iterations;

    /**
     * Computes the solver config value for the current Repulsor planning state. Call this from
     * periodic planning or tests when a fresh decision is required; inputs should already be
     * expressed in the coordinate frame expected by the parameter names.
     *
     * @return moving shot solver.config result for solver config.
     */
    public MovingShotSolver.Config solverConfig() {
      MovingShotSolver.Config defaults = MovingShotSolver.Config.defaults();
      return new MovingShotSolver.Config(
          finiteNonNegative(releaseLatencySeconds, defaults.releaseLatencySeconds()),
          finiteNonNegative(minFlightPredictionSeconds, defaults.minFlightPredictionSeconds()),
          finiteNonNegative(maxFlightPredictionSeconds, defaults.maxFlightPredictionSeconds()),
          finiteNonNegative(
              defaultFlightPredictionSeconds, defaults.defaultFlightPredictionSeconds()),
          finiteNonNegative(
              maxCompensatedSpeedMetersPerSecond, defaults.maxCompensatedSpeedMetersPerSecond()),
          finiteNonNegative(
              maxReleaseSpeedMetersPerSecond, defaults.maxReleaseSpeedMetersPerSecond()),
          finiteNonNegative(yawToleranceDegrees, defaults.yawToleranceDegrees()),
          finiteNonNegative(maxVerticalErrorMeters, defaults.maxVerticalErrorMeters()),
          positive(iterations, defaults.iterations()));
    }
  }

  /**
   * Returns the finite non negative value maintained by this Repulsor component.
   *
   * @param value value used by this operation.
   * @param fallback value used by this operation.
   * @return value produced by this operation.
   */
  static double finiteNonNegative(Double value, double fallback) {
    return value != null && Double.isFinite(value) && value >= 0.0 ? value : fallback;
  }

  /**
   * Provides rebuilt corridor config functionality for the Repulsor field/profile definition layer
   * used to tune Repulsor for a specific game. Use this type from robot code, field profiles, or
   * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
   * unless a method documents robot-relative motion.
   */
  public static class RebuiltCorridorConfig {
    /**
     * Configuration value for rect width meters. Distances use meters in WPILib field coordinates
     * and should be treated as tunable when sourced from profiles.
     */
    public Double rectWidthMeters;

    /**
     * Configuration value for rect height meters. Distances use meters in WPILib field coordinates
     * and should be treated as tunable when sourced from profiles.
     */
    public Double rectHeightMeters;

    /**
     * Configuration value for rect offset from center meters. Distances use meters in WPILib field
     * coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double rectOffsetFromCenterMeters;

    /**
     * Configuration value for edge offset meters. Distances use meters in WPILib field coordinates
     * and should be treated as tunable when sourced from profiles.
     */
    public Double edgeOffsetMeters;

    /**
     * Configuration value for rect strength. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public Double rectStrength;

    /**
     * Configuration value for rect range xmeters. Distances use meters in WPILib field coordinates
     * and should be treated as tunable when sourced from profiles.
     */
    public Double rectRangeXMeters;

    /**
     * Configuration value for rect range ymeters. Distances use meters in WPILib field coordinates
     * and should be treated as tunable when sourced from profiles.
     */
    public Double rectRangeYMeters;

    /**
     * Configuration value for bias strength. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public Double biasStrength;

    /**
     * Configuration value for bias range meters. Distances use meters in WPILib field coordinates
     * and should be treated as tunable when sourced from profiles.
     */
    public Double biasRangeMeters;

    /**
     * Configuration value for bypass strength scale. The valid range and tuning source are defined
     * by the owning subsystem or field profile.
     */
    public Double bypassStrengthScale;

    /**
     * Configuration value for bypass range meters. Distances use meters in WPILib field coordinates
     * and should be treated as tunable when sourced from profiles.
     */
    public Double bypassRangeMeters;

    /**
     * Configuration value for side pull dx meters. Distances use meters in WPILib field coordinates
     * and should be treated as tunable when sourced from profiles.
     */
    public Double sidePullDxMeters;

    /**
     * Configuration value for side bias strength scale. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public Double sideBiasStrengthScale;

    /**
     * Configuration value for side bias range scale. The valid range and tuning source are defined
     * by the owning subsystem or field profile.
     */
    public Double sideBiasRangeScale;

    /**
     * Configuration value for side bypass strength scale. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public Double sideBypassStrengthScale;

    /**
     * Configuration value for side bypass range scale. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public Double sideBypassRangeScale;

    /**
     * Configuration value for rail xwindow meters. Distances use meters in WPILib field coordinates
     * and should be treated as tunable when sourced from profiles.
     */
    public Double railXWindowMeters;

    /**
     * Configuration value for rail min half width meters. Distances use meters in WPILib field
     * coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double railMinHalfWidthMeters;

    /**
     * Configuration value for rail half width gap scale. Distances use meters in WPILib field
     * coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double railHalfWidthGapScale;

    /**
     * Configuration value for rail strength. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public Double railStrength;

    /**
     * Configuration value for rail max force. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public Double railMaxForce;

    /**
     * Configuration value for center rail min window meters. Distances use meters in WPILib field
     * coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double centerRailMinWindowMeters;

    /**
     * Configuration value for center rail window scale. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public Double centerRailWindowScale;

    /**
     * Configuration value for outer rail xoffset scale. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public Double outerRailXOffsetScale;

    /**
     * Configuration value for outer rail window meters. Distances use meters in WPILib field
     * coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double outerRailWindowMeters;
  }
}
