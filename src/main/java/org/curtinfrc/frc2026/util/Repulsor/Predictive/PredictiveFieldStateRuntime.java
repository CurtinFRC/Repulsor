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
package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Offload.PredictiveFieldStateOffloadEntrypoints_Offloaded;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryDynamicObjectDTO;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryPointDTO;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.CollectProbe;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PredictiveRankingBreakdown;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PredictiveRankingConfig;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceCollectionProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceRecoveryProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective.ObjectiveSelectionConfig;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective.ObjectiveSelectionDecision;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective.ObjectiveSelector;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.ResourceRegionSummary;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;

/**
 * Provides predictive field state runtime functionality for the Repulsor predictive field-state and
 * collection-planning layer. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class PredictiveFieldStateRuntime {
  private final PredictiveFieldStateOps ops = new PredictiveFieldStateOps();

  /**
   * Configuration value for collect age decay. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  static final double COLLECT_AGE_DECAY = 0.75;

  /**
   * Configuration value for resource sigma abs max. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  static final double RESOURCE_SIGMA_ABS_MAX = 0.45;

  /**
   * Configuration value for resource sigma rel max. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  static final double RESOURCE_SIGMA_REL_MAX = 1.25;

  /**
   * Configuration value for resource sigma min. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double RESOURCE_SIGMA_MIN = 0.06;

  /**
   * Configuration value for resource hard max age s. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  static final double RESOURCE_HARD_MAX_AGE_S = 0.95;

  /**
   * Returns the error value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  static boolean error() {
    return PredictiveFieldStateOps.error();
  }

  /**
   * Returns the probe collect value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return collect probe result for probe collect.
   */
  public CollectProbe probeCollect(Translation2d p) {
    return ops.probeCollect(p);
  }

  /**
   * Returns the probe collect value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param countR value used by this operation.
   * @return collect probe result for probe collect.
   */
  public CollectProbe probeCollect(Translation2d p, double countR) {
    return ops.probeCollect(p, countR);
  }

  /**
   * Returns the footprint has collect resource value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public boolean footprintHasCollectResource(Translation2d center, double cellM) {
    return ops.footprintHasCollectResource(center, cellM);
  }

  /**
   * Returns the footprint has fuel value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public boolean footprintHasFuel(Translation2d center, double cellM) {
    return ops.footprintHasFuel(center, cellM);
  }

  /**
   * Runs mark collect depleted in the Repulsor runtime.
   *
   * @param p value used by this operation.
   * @param cellM value used by this operation.
   * @param strength value used by this operation.
   */
  public void markCollectDepleted(Translation2d p, double cellM, double strength) {
    ops.markCollectDepleted(p, cellM, strength);
  }

  /**
   * Updates register resource spec state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param type value used by this operation.
   * @param spec value used by this operation.
   */
  public void registerResourceSpec(String type, ResourceSpec spec) {
    ops.registerResourceSpec(type, spec);
  }

  /**
   * Installs the game/profile-specific predictive collection configuration.
   *
   * @param profile resource collection profile for the active field
   */
  public void configureCollectionProfile(ResourceCollectionProfile profile) {
    ops.configureCollectionProfile(profile);
  }

  /**
   * Replaces the predictive field geometry used by bounds checks and wall costs.
   *
   * @param geometry field dimensions in meters
   */
  public void setFieldGeometry(FieldGeometry geometry) {
    ops.setFieldGeometry(geometry);
  }

  /**
   * Returns the predictive field geometry currently used by collection logic.
   *
   * @return active field geometry
   */
  public FieldGeometry getFieldGeometry() {
    return ops.getFieldGeometry();
  }

  /**
   * Updates register other type weight state or telemetry as part of the Repulsor runtime loop.
   * This may mutate local state, NetworkTables output, planner caches, or command-side runtime
   * state depending on the owning type.
   *
   * @param type value used by this operation.
   * @param weight value used by this operation.
   */
  public void registerOtherTypeWeight(String type, double weight) {
    ops.registerOtherTypeWeight(type, weight);
  }

  /**
   * Updates set collect resource types state or telemetry as part of the Repulsor runtime loop.
   * This may mutate local state, NetworkTables output, planner caches, or command-side runtime
   * state depending on the owning type.
   *
   * @param types value used by this operation.
   */
  public void setCollectResourceTypes(Set<String> types) {
    ops.setCollectResourceTypes(types);
  }

  /**
   * Returns the get collect resource types value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  Set<String> getCollectResourceTypes() {
    return ops.getCollectResourceTypes();
  }

  /**
   * Runs add collect resource type in the Repulsor runtime.
   *
   * @param type value used by this operation.
   */
  public void addCollectResourceType(String type) {
    ops.addCollectResourceType(type);
  }

  /**
   * Runs remove collect resource type in the Repulsor runtime.
   *
   * @param type value used by this operation.
   */
  void removeCollectResourceType(String type) {
    ops.removeCollectResourceType(type);
  }

  /**
   * Returns the is collect resource type value maintained by this Repulsor component.
   *
   * @param type value used by this operation.
   * @return value produced by this operation.
   */
  public boolean isCollectResourceType(String type) {
    return ops.isCollectResourceType(type);
  }

  /**
   * Updates set collect resource position filter state or telemetry as part of the Repulsor runtime
   * loop. This may mutate local state, NetworkTables output, planner caches, or command-side
   * runtime state depending on the owning type.
   *
   * @param filter value used by this operation.
   */
  public void setCollectResourcePositionFilter(Predicate<Translation2d> filter) {
    ops.setCollectResourcePositionFilter(filter);
  }

  /**
   * Updates set dynamic objects state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param objs value used by this operation.
   */
  public void setDynamicObjects(List<DynamicObject> objs) {
    ops.setDynamicObjects(objs);
  }

  /**
   * Updates set world state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param elements value used by this operation.
   * @param ours value used by this operation.
   */
  public void setWorld(List<GameElement> elements, Alliance ours) {
    ops.setWorld(elements, ours);
  }

  /**
   * Updates update ally state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param id value used by this operation.
   * @param pos value used by this operation.
   * @param velHint value used by this operation.
   * @param speedCap value used by this operation.
   */
  public void updateAlly(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    ops.updateAlly(id, pos, velHint, speedCap);
  }

  /**
   * Updates update enemy state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param id value used by this operation.
   * @param pos value used by this operation.
   * @param velHint value used by this operation.
   * @param speedCap value used by this operation.
   */
  public void updateEnemy(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    ops.updateEnemy(id, pos, velHint, speedCap);
  }

  /**
   * Updates clear stale state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param maxAgeS value used by this operation.
   */
  public void clearStale(double maxAgeS) {
    ops.clearStale(maxAgeS);
  }

  /**
   * Returns the rank value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param cat value used by this operation.
   * @param limit value used by this operation.
   * @return value produced by this operation.
   */
  public List<Candidate> rank(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return ops.rank(ourPos, ourSpeedCap, cat, limit);
  }

  public ObjectiveSelectionDecision selectObjective(
      Translation2d ourPos,
      double ourSpeedCap,
      CategorySpec cat,
      RepulsorSetpoint currentObjective,
      ObjectiveSelectionConfig config) {
    ObjectiveSelectionConfig safeConfig =
        config == null ? ObjectiveSelectionConfig.defaults() : config;
    return ObjectiveSelector.select(
        rank(ourPos, ourSpeedCap, cat, safeConfig.candidateLimit()), currentObjective, safeConfig);
  }

  public void configureRanking(PredictiveRankingConfig config) {
    ops.configureRanking(config);
  }

  public List<PredictiveRankingBreakdown> lastRankingBreakdown() {
    return ops.lastRankingBreakdown();
  }

  /**
   * Returns the rank setpoints value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param cat value used by this operation.
   * @param limit value used by this operation.
   * @return list of repulsor setpoint values produced by this operation.
   */
  public List<RepulsorSetpoint> rankSetpoints(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return ops.rankSetpoints(ourPos, ourSpeedCap, cat, limit);
  }

  /**
   * Returns the resource observation count value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public int resourceObservationCount() {
    return ops.resourceObservationCount();
  }

  /**
   * Returns the snap to collect centroid value maintained by this Repulsor component.
   *
   * @param seed value used by this operation.
   * @param r value used by this operation.
   * @param minMass value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d snapToCollectCentroid(Translation2d seed, double r, double minMass) {
    return ops.snapToCollectCentroid(seed, r, minMass);
  }

  /**
   * Computes the nearest collect resource value for the current Repulsor planning state.
   *
   * @param p value used by this operation.
   * @param maxDist value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d nearestCollectResource(Translation2d p, double maxDist) {
    return ops.nearestCollectResource(p, maxDist);
  }

  /**
   * Summarizes collectable resources inside a caller-defined field region for reusable strategy
   * evaluation.
   *
   * @param id stable telemetry identifier for the region
   * @param regionFilter field-relative predicate selecting resources in the region
   * @param robotPos current robot position in field-relative meters
   * @param maxDistanceMeters maximum robot-to-resource distance considered actionable
   * @return normalized resource and risk summary for the region
   */
  public ResourceRegionSummary summarizeResourceRegion(
      String id,
      Predicate<Translation2d> regionFilter,
      Translation2d robotPos,
      double maxDistanceMeters) {
    return ops.summarizeResourceRegion(id, regionFilter, robotPos, maxDistanceMeters);
  }

  /**
   * Summarizes a region using the configured field diagonal as the maximum actionable distance.
   *
   * @param id stable telemetry identifier for the region
   * @param regionFilter field-relative predicate selecting resources in the region
   * @param robotPos current robot position in field-relative meters
   * @return normalized resource and risk summary for the region
   */
  public ResourceRegionSummary summarizeResourceRegion(
      String id, Predicate<Translation2d> regionFilter, Translation2d robotPos) {
    return ops.summarizeResourceRegion(id, regionFilter, robotPos);
  }

  /**
   * Returns the rank collect nearest value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param points value used by this operation.
   * @param cellM value used by this operation.
   * @param goalUnits value used by this operation.
   * @param limit value used by this operation.
   * @return point candidate result for rank collect nearest.
   */
  public PointCandidate rankCollectNearest(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int limit) {
    return ops.rankCollectNearest(ourPos, ourSpeedCap, points, cellM, goalUnits, limit);
  }

  /**
   * Returns the rank collect hierarchical value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param points value used by this operation.
   * @param cellM value used by this operation.
   * @param goalUnits value used by this operation.
   * @param coarseTopK value used by this operation.
   * @param refineGrid value used by this operation.
   * @return point candidate result for rank collect hierarchical.
   */
  public PointCandidate rankCollectHierarchical(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int coarseTopK,
      int refineGrid) {
    return ops.rankCollectHierarchical(
        ourPos, ourSpeedCap, points, cellM, goalUnits, coarseTopK, refineGrid);
  }

  /**
   * Returns the best collect hotspot value maintained by this Repulsor component.
   *
   * @param points value used by this operation.
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d bestCollectHotspot(Translation2d[] points, double cellM) {
    return ops.bestCollectHotspot(points, cellM);
  }

  /**
   * Returns the rank collect points value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param points value used by this operation.
   * @param goalUnits value used by this operation.
   * @param limit value used by this operation.
   * @return point candidate result for rank collect points.
   */
  public PointCandidate rankCollectPoints(
      Translation2d ourPos, double ourSpeedCap, Translation2d[] points, int goalUnits, int limit) {
    return ops.rankCollectPoints(ourPos, ourSpeedCap, points, goalUnits, limit);
  }

  /**
   * Computes the select shuttle recovery point local value for the current Repulsor planning state.
   * Call this from periodic planning or tests when a fresh decision is required; inputs should
   * already be expressed in the coordinate frame expected by the parameter names.
   *
   * @param robotPoseBlue value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param goalUnits value used by this operation.
   * @param flipRedToBlue value used by this operation.
   * @param dynamicObjects value used by this operation.
   * @return shuttle recovery point dto result for select shuttle recovery point local.
   */
  public ShuttleRecoveryPointDTO selectShuttleRecoveryPointLocal(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects) {
    return PredictiveFieldStateLocalAccess.selectShuttleRecoveryPointLocal(
        robotPoseBlue, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects);
  }

  /**
   * Selects a recovery point for a generic transferred resource profile.
   *
   * @param robotPoseBlue robot pose expressed in blue-origin field coordinates
   * @param ourSpeedCap robot speed cap in meters per second
   * @param goalUnits desired recovered resource units
   * @param flipRedToBlue whether dynamic objects should be mirrored into blue coordinates
   * @param dynamicObjects transferred resource observations
   * @param profile recovery profile for the active game
   * @return selected recovery point, or a not-found DTO
   */
  public ShuttleRecoveryPointDTO selectResourceRecoveryPointLocal(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects,
      ResourceRecoveryProfile profile) {
    return PredictiveFieldStateLocalAccess.selectResourceRecoveryPointLocal(
        robotPoseBlue, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects, profile);
  }

  /**
   * Computes the select shuttle recovery point offloaded value for the current Repulsor planning
   * state. Call this from periodic planning or tests when a fresh decision is required; inputs
   * should already be expressed in the coordinate frame expected by the parameter names.
   *
   * @param robotPoseBlue value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param goalUnits value used by this operation.
   * @param flipRedToBlue value used by this operation.
   * @param dynamicObjects value used by this operation.
   * @return shuttle recovery point dto result for select shuttle recovery point offloaded.
   */
  public ShuttleRecoveryPointDTO selectShuttleRecoveryPointOffloaded(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects) {
    return PredictiveFieldStateOffloadEntrypoints_Offloaded.selectShuttleRecoveryPoint_offload(
        robotPoseBlue, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects);
  }

  /**
   * Selects a generic resource recovery point. The current offload boundary only serializes the
   * legacy shuttle DTO shape, so non-default profiles execute locally until a generated generic
   * offload endpoint is added.
   *
   * @param robotPoseBlue robot pose expressed in blue-origin field coordinates
   * @param ourSpeedCap robot speed cap in meters per second
   * @param goalUnits desired recovered resource units
   * @param flipRedToBlue whether dynamic objects should be mirrored into blue coordinates
   * @param dynamicObjects transferred resource observations
   * @param profile recovery profile for the active game
   * @return selected recovery point, or a not-found DTO
   */
  public ShuttleRecoveryPointDTO selectResourceRecoveryPoint(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects,
      ResourceRecoveryProfile profile) {
    return selectResourceRecoveryPointLocal(
        robotPoseBlue, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects, profile);
  }
}
