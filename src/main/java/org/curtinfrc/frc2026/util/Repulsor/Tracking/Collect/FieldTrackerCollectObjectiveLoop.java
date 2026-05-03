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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.CollectProbe;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateRuntime;
import org.curtinfrc.frc2026.util.Repulsor.Target.StickyTarget;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassCandidateResult;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassCandidateStep;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassContext;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassDriveStep;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassSetup;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassSetupResult;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassStickyResult;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassStickyStep;

/**
 * Provides field tracker collect objective loop functionality for the Repulsor collection objective
 * runtime for tracked field resources. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class FieldTrackerCollectObjectiveLoop {
  /**
   * Configuration value for predictor. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final PredictiveFieldStateRuntime predictor;

  /**
   * Configuration value for collect objective points. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public final Supplier<Translation2d[]> collectObjectivePoints;

  /**
   * Configuration value for dynamics supplier. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final Supplier<List<DynamicObject>> dynamicsSupplier;

  /**
   * Configuration value for collect type predicate. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  final Predicate<String> collectTypePredicate;

  public final CollectPlannerTuning tuning;

  /**
   * Creates a field tracker collect objective loop instance with the dependencies and tuning values
   * used by this Repulsor component.
   *
   * @param predictor value used by this operation.
   * @param collectObjectivePoints value used by this operation.
   * @param dynamicsSupplier value used by this operation.
   * @param collectTypePredicate value used by this operation.
   */
  FieldTrackerCollectObjectiveLoop(
      PredictiveFieldStateRuntime predictor,
      Supplier<Translation2d[]> collectObjectivePoints,
      Supplier<List<DynamicObject>> dynamicsSupplier,
      Predicate<String> collectTypePredicate) {
    this(
        predictor,
        collectObjectivePoints,
        dynamicsSupplier,
        collectTypePredicate,
        CollectPlannerTuning.defaults());
  }

  FieldTrackerCollectObjectiveLoop(
      PredictiveFieldStateRuntime predictor,
      Supplier<Translation2d[]> collectObjectivePoints,
      Supplier<List<DynamicObject>> dynamicsSupplier,
      Predicate<String> collectTypePredicate,
      CollectPlannerTuning tuning) {
    this.predictor = predictor;
    this.collectObjectivePoints = collectObjectivePoints;
    this.dynamicsSupplier = dynamicsSupplier;
    this.collectTypePredicate = collectTypePredicate;
    this.tuning = tuning == null ? CollectPlannerTuning.defaults() : tuning;
  }

  /**
   * Returns the snapshot dynamic objects value maintained by this Repulsor component.
   *
   * @return list of dynamic object values produced by this operation.
   */
  public List<DynamicObject> snapshotDynamicObjects() {
    List<DynamicObject> dyn = dynamicsSupplier.get();
    return dyn != null ? dyn : List.of();
  }

  /**
   * Returns the is collect type value maintained by this Repulsor component.
   *
   * @param type value used by this operation.
   * @return value produced by this operation.
   */
  public boolean isCollectType(String type) {
    return collectTypePredicate.test(type);
  }

  public StickyTarget<Translation2d> collectStickySelector = new StickyTarget<>(0.22, 1.25, 1.80);

  /**
   * Updates reset all state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   */
  void resetAll() {
    collectStickySelector = new StickyTarget<>(0.22, 1.25, 1.80);
    clearCollectSticky();
  }

  /**
   * Configuration value for collect sticky reached ts ns. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile long collectStickyReachedTsNs = 0L;

  /**
   * Configuration value for collect sticky approach hat. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile Translation2d collectStickyApproachHat = null;

  /**
   * Configuration value for collect sticky push m. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public volatile double collectStickyPushM = 0.0;

  /**
   * Configuration value for collect sticky no progress since ns. The valid range and tuning source
   * are defined by the owning subsystem or field profile.
   */
  public volatile long collectStickyNoProgressSinceNs = 0L;

  /**
   * Configuration value for collect sticky last dist m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile double collectStickyLastDistM = Double.POSITIVE_INFINITY;

  /**
   * Configuration value for collect sticky drive target. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile Translation2d collectStickyDriveTarget = null;

  /**
   * Configuration value for collect sticky side. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public volatile int collectStickySide = 0;

  /**
   * Configuration value for collect sticky last switch ns. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile long collectStickyLastSwitchNs = 0L;

  /**
   * Configuration value for collect drive last target. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile Translation2d collectDriveLastTarget = null;

  /**
   * Configuration value for collect drive last target ns. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile long collectDriveLastTargetNs = 0L;

  /**
   * Configuration value for collect group cell m. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double COLLECT_GROUP_CELL_M = 0.40;

  /**
   * Configuration value for collect group r1 m. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double COLLECT_GROUP_R1_M = 0.95;

  /**
   * Configuration value for collect group r2 m. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double COLLECT_GROUP_R2_M = 1.70;

  /**
   * Configuration value for collect group min count. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  static final double COLLECT_GROUP_MIN_COUNT = 2.0;

  /**
   * Configuration value for collect relock enable dist m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  static final double COLLECT_RELOCK_ENABLE_DIST_M = 1.6;

  /**
   * Configuration value for collect half keep mid band m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_HALF_KEEP_MID_BAND_M = 1.2;

  /**
   * Configuration value for collect sticky invalid sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public volatile double collectStickyInvalidSec = 0.0;

  /**
   * Configuration value for collect forced drive cand. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile Translation2d collectForcedDriveCand = null;

  /**
   * Configuration value for collect forced drive since ns. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile long collectForcedDriveSinceNs = 0L;

  /**
   * Configuration value for collect stuck radius m. Distances use meters in WPILib field
   * coordinates and should be treated as tunable when sourced from profiles.
   */
  public static final double COLLECT_STUCK_RADIUS_M = 0.10;

  /**
   * Configuration value for collect stuck reset move m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_STUCK_RESET_MOVE_M = 0.22;

  public Translation2d collectStuckAnchorPos = new Translation2d();

  /**
   * Configuration value for collect stuck anchor ns. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public volatile long collectStuckAnchorNs = 0L;

  /**
   * Configuration value for collect sticky half lock. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public volatile int collectStickyHalfLock = 0;

  /**
   * Configuration value for last best. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public PointCandidate lastBest;

  /**
   * Configuration value for collect group w c1. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double COLLECT_GROUP_W_C1 = 1.00;

  /**
   * Configuration value for collect group w c2. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double COLLECT_GROUP_W_C2 = 0.35;

  /**
   * Configuration value for collect group w eta. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double COLLECT_GROUP_W_ETA = 1.25;

  /**
   * Configuration value for collect group w spread. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  static final double COLLECT_GROUP_W_SPREAD = 0.60;

  /**
   * Configuration value for collect group w center. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  static final double COLLECT_GROUP_W_CENTER = 0.18;

  /**
   * Configuration value for collect nearby radius m. Distances use meters in WPILib field
   * coordinates and should be treated as tunable when sourced from profiles.
   */
  public static final double COLLECT_NEARBY_RADIUS_M = 2.2;

  /**
   * Configuration value for collect nearby min count. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final int COLLECT_NEARBY_MIN_COUNT = 1;

  /**
   * Configuration value for collect live fuel near target r m. The valid range and tuning source
   * are defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_LIVE_FUEL_NEAR_TARGET_R_M = 0.65;

  /**
   * Configuration value for collect live obs max age s. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_LIVE_OBS_MAX_AGE_S = 0.30;

  /**
   * Configuration value for collect predictor obs max age s. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_PREDICTOR_OBS_MAX_AGE_S = 0.25;

  /**
   * Configuration value for collect sticky invalid drop sec. Time values use seconds and should be
   * tuned against measured robot loop and mechanism latency.
   */
  public static final double COLLECT_STICKY_INVALID_DROP_SEC = 0.10;

  /**
   * Configuration value for collect reached empty force drop sec. Time values use seconds and
   * should be tuned against measured robot loop and mechanism latency.
   */
  public static final double COLLECT_REACHED_EMPTY_FORCE_DROP_SEC = 0.18;

  /**
   * Configuration value for collect reached empty near target m. The valid range and tuning source
   * are defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_REACHED_EMPTY_NEAR_TARGET_M = 0.95;

  /**
   * Configuration value for collect reached empty sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public double collectReachedEmptySec = 0.0;

  /**
   * Configuration value for collect done no fuel sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public static final double COLLECT_DONE_NO_FUEL_SEC = 0.35;

  /**
   * Configuration value for collect done stuck sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public static final double COLLECT_DONE_STUCK_SEC = 0.35;

  /**
   * Configuration value for collect stuck speed mps. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_STUCK_SPEED_MPS = 0.15;

  /**
   * Configuration value for collect auto switch still sec. Time values use seconds and should be
   * tuned against measured robot loop and mechanism latency.
   */
  public static final double COLLECT_AUTO_SWITCH_STILL_SEC = 1.0;

  /**
   * Configuration value for collect empty drive done sec. Time values use seconds and should be
   * tuned against measured robot loop and mechanism latency.
   */
  public static final double COLLECT_EMPTY_DRIVE_DONE_SEC = 0.35;

  /**
   * Configuration value for collect empty drive near robot m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_EMPTY_DRIVE_NEAR_ROBOT_M = 1.05;

  /**
   * Configuration value for collect empty drive probe r m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_EMPTY_DRIVE_PROBE_R_M = 0.55;

  /**
   * Configuration value for collect empty drive min units. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_EMPTY_DRIVE_MIN_UNITS = 0.06;

  /**
   * Configuration value for collect empty drive sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public double collectEmptyDriveSec = 0.0;

  /**
   * Configuration value for collect cell m. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double COLLECT_CELL_M = 0.14;

  /**
   * Configuration value for collect coarse topk. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final int COLLECT_COARSE_TOPK = 4;

  /**
   * Configuration value for collect refine grid. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final int COLLECT_REFINE_GRID = 3;

  /**
   * Configuration value for collect drive probe r m. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_DRIVE_PROBE_R_M = 0.55;

  /**
   * Configuration value for collect drive min units. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_DRIVE_MIN_UNITS = 0.045;

  /**
   * Configuration value for collect drive search step m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_DRIVE_SEARCH_STEP_M = 0.14;

  /**
   * Configuration value for collect drive search grid. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final int COLLECT_DRIVE_SEARCH_GRID = 3;

  public Translation2d collectStickyStillFiltPos = new Translation2d();
  public Translation2d collectStickyStillFiltLastPos = new Translation2d();

  /**
   * Configuration value for collect sticky robot half last. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public int collectStickyRobotHalfLast = 0;

  /**
   * Configuration value for collect sticky robot flicker sec. Time values use seconds and should be
   * tuned against measured robot loop and mechanism latency.
   */
  public double collectStickyRobotFlickerSec = 0.0;

  /**
   * Configuration value for collect sticky same m. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_STICKY_SAME_M = 0.45;

  /**
   * Configuration value for collect switch close m. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_SWITCH_CLOSE_M = 0.85;

  /**
   * Configuration value for collect sticky reached m. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_STICKY_REACHED_M = 0.22;

  /**
   * Configuration value for collect sticky target recalc eps m. The valid range and tuning source
   * are defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_STICKY_TARGET_RECALC_EPS_M = 0.14;

  /**
   * Configuration value for collect sticky no progress s. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_STICKY_NO_PROGRESS_S = 0.40;

  /**
   * Configuration value for collect sticky no progress drop m. The valid range and tuning source
   * are defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_STICKY_NO_PROGRESS_DROP_M = 0.06;

  /**
   * Configuration value for collect sticky no progress min dist m. The valid range and tuning
   * source are defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_STICKY_NO_PROGRESS_MIN_DIST_M = 0.50;

  /**
   * Configuration value for collect sticky flap cooldown s. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_STICKY_FLAP_COOLDOWN_S = 0.95;

  /**
   * Configuration value for collect switch min move m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_SWITCH_MIN_MOVE_M = 0.18;

  /**
   * Configuration value for collect switch cooldown s. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_SWITCH_COOLDOWN_S = 0.70;

  /**
   * Configuration value for collect no fuel sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public double collectNoFuelSec = 0.0;

  /**
   * Configuration value for collect stuck sec. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  public double collectStuckSec = 0.0;

  /**
   * Configuration value for collect auto switch still sec. Time values use seconds and should be
   * tuned against measured robot loop and mechanism latency.
   */
  public double collectAutoSwitchStillSec = 0.0;

  public Translation2d lastRobotPosForStuck = new Translation2d();

  /**
   * Configuration value for collect sticky eta s. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  volatile double collectStickyEtaS = 0.0;

  /**
   * Configuration value for last objective tick ns. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public volatile long lastObjectiveTickNs = 0L;

  /**
   * Configuration value for collect resource snap max dist m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  static final double COLLECT_RESOURCE_SNAP_MAX_DIST_M = 0.55;

  /**
   * Configuration value for collect resource snap tiny m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  static final double COLLECT_RESOURCE_SNAP_TINY_M = 0.14;

  /**
   * Configuration value for collect valid near fuel m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_VALID_NEAR_FUEL_M = 0.25;

  /**
   * Configuration value for collect resource snap min units. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_RESOURCE_SNAP_MIN_UNITS = 0.07;

  /**
   * Configuration value for collect snap to point m. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_SNAP_TO_POINT_M = 0.22;

  /**
   * Configuration value for collect snap hyst m. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_SNAP_HYST_M = 0.10;

  /**
   * Configuration value for collect snap active. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public boolean collectSnapActive = false;

  public Translation2d collectStickyStillLastPos = new Translation2d();

  /**
   * Configuration value for collect sticky still sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public double collectStickyStillSec = 0.0;

  /**
   * Configuration value for collect sticky last switch robot pos. The valid range and tuning source
   * are defined by the owning subsystem or field profile.
   */
  public Translation2d collectStickyLastSwitchRobotPos = null;

  /**
   * Configuration value for collect sticky point. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public volatile Translation2d collectStickyPoint = null;

  /**
   * Configuration value for collect sticky score. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public volatile double collectStickyScore = -1e18;

  /**
   * Configuration value for collect sticky ts ns. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public volatile long collectStickyTsNs = 0L;

  /**
   * Configuration value for collect empty space max dist to fuel m. The valid range and tuning
   * source are defined by the owning subsystem or field profile.
   */
  static final double COLLECT_EMPTY_SPACE_MAX_DIST_TO_FUEL_M = 0.18;

  /**
   * Configuration value for collect hotspot snap radius m. Distances use meters in WPILib field
   * coordinates and should be treated as tunable when sourced from profiles.
   */
  public static final double COLLECT_HOTSPOT_SNAP_RADIUS_M = 0.85;

  /**
   * Configuration value for collect max drive offset from fuel m. The valid range and tuning source
   * are defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_MAX_DRIVE_OFFSET_FROM_FUEL_M = 0.12;

  /**
   * Configuration value for collect snap to nearest fuel m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_SNAP_TO_NEAREST_FUEL_M = 0.22;

  /**
   * Configuration value for collect force on fuel search m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_FORCE_ON_FUEL_SEARCH_M = 0.40;

  /**
   * Configuration value for collect force on fuel probe r m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_FORCE_ON_FUEL_PROBE_R_M = 0.45;

  /**
   * Configuration value for collect force on fuel min units. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_FORCE_ON_FUEL_MIN_UNITS = 0.055;

  /**
   * Updates clear collect sticky state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   */
  public void clearCollectSticky() {
    collectStickyPoint = null;
    collectStickyScore = -1e18;
    collectStickyTsNs = 0L;
    collectStickyReachedTsNs = 0L;
    collectStickyApproachHat = null;
    collectStickyPushM = 0.0;
    collectStickyDriveTarget = null;
    collectStickyNoProgressSinceNs = 0L;
    collectStickyLastDistM = Double.POSITIVE_INFINITY;
    collectStickyLastSwitchNs = 0L;
    collectStickyLastSwitchRobotPos = null;
    collectStickyEtaS = 0.0;
    collectStickySide = 0;
    collectSnapActive = false;
    collectStickyHalfLock = 0;
    collectNoFuelSec = 0.0;
    collectStuckSec = 0.0;
    collectAutoSwitchStillSec = 0.0;
    lastRobotPosForStuck = new Translation2d();
    collectEmptyDriveSec = 0.0;
    collectStuckAnchorPos = new Translation2d();
    collectStuckAnchorNs = 0L;
    collectReachedEmptySec = 0.0;
    collectStickySelector.clear();
    collectStickyStillLastPos = new Translation2d();
    collectStickyStillSec = 0.0;
    collectStickyStillFiltPos = new Translation2d();
    collectStickyStillFiltLastPos = new Translation2d();
    collectStickyRobotHalfLast = 0;
    collectStickyRobotFlickerSec = 0.0;
    collectStickyInvalidSec = 0.0;
    collectDriveLastTarget = null;
    collectDriveLastTargetNs = 0L;
  }

  /**
   * Returns the count live collect resources within value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param center value used by this operation.
   * @param r value used by this operation.
   * @return value produced by this operation.
   */
  public int countLiveCollectResourcesWithin(
      List<DynamicObject> dyn, Translation2d center, double r) {
    if (dyn == null || dyn.isEmpty() || center == null) return 0;
    double r2 = r * r;
    int n = 0;
    for (int i = 0; i < dyn.size(); i++) {
      DynamicObject o = dyn.get(i);
      if (!isFreshCollectObservation(o)) continue;
      double dx = o.pos.getX() - center.getX();
      double dy = o.pos.getY() - center.getY();
      if ((dx * dx + dy * dy) <= r2) n++;
    }
    return n;
  }

  /**
   * Returns the is fresh collect observation value maintained by this Repulsor component.
   *
   * @param o value used by this operation.
   * @return value produced by this operation.
   */
  public boolean isFreshCollectObservation(DynamicObject o) {
    if (o == null || o.pos == null || o.type == null) return false;
    if (!isCollectType(o.type)) return false;
    return o.ageS <= COLLECT_LIVE_OBS_MAX_AGE_S;
  }

  /**
   * Returns the filter dynamics for collect predictor value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @return list of dynamic object values produced by this operation.
   */
  public List<DynamicObject> filterDynamicsForCollectPredictor(List<DynamicObject> dyn) {
    if (dyn == null || dyn.isEmpty()) return List.of();
    ArrayList<DynamicObject> out = new ArrayList<>(dyn.size());
    for (int i = 0; i < dyn.size(); i++) {
      DynamicObject o = dyn.get(i);
      if (o == null || o.pos == null || o.type == null) continue;
      if (isCollectType(o.type) && o.ageS > COLLECT_PREDICTOR_OBS_MAX_AGE_S) continue;
      out.add(o);
    }
    return out;
  }

  /**
   * Returns the relock collect point to live fuel value maintained by this Repulsor component.
   *
   * @param desiredCollectPoint value used by this operation.
   * @param clampToFieldRobotSafe value used by this operation.
   * @param inForbidden value used by this operation.
   * @param violatesWall value used by this operation.
   * @param nudgeOutOfForbidden value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d relockCollectPointToLiveFuel(
      Translation2d desiredCollectPoint,
      java.util.function.Function<Translation2d, Translation2d> clampToFieldRobotSafe,
      java.util.function.Predicate<Translation2d> inForbidden,
      java.util.function.Predicate<Translation2d> violatesWall,
      java.util.function.Function<Translation2d, Translation2d> nudgeOutOfForbidden) {

    if (desiredCollectPoint == null) return null;

    Translation2d p = desiredCollectPoint;

    Translation2d nearTiny = predictor.nearestCollectResource(p, COLLECT_RESOURCE_SNAP_TINY_M);
    if (nearTiny != null) {
      p = nearTiny;
    } else {
      Translation2d near = predictor.nearestCollectResource(p, COLLECT_RESOURCE_SNAP_MAX_DIST_M);
      if (near != null) {
        p = near;
      }
    }

    p = clampToFieldRobotSafe.apply(p);
    if (inForbidden.test(p)) p = nudgeOutOfForbidden.apply(p);
    p = clampToFieldRobotSafe.apply(p);

    if (inForbidden.test(p) || violatesWall.test(p)) return desiredCollectPoint;
    return p;
  }

  /**
   * Computes the compute frozen drive target value for the current Repulsor planning state. Call
   * this from periodic planning or tests when a fresh decision is required; inputs should already
   * be expressed in the coordinate frame expected by the parameter names.
   *
   * @param resource value used by this operation.
   * @param approachHat value used by this operation.
   * @param pushM value used by this operation.
   * @param clampToFieldRobotSafe value used by this operation.
   * @param inForbidden value used by this operation.
   * @param violatesWall value used by this operation.
   * @param nudgeOutOfForbidden value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d computeFrozenDriveTarget(
      Translation2d resource,
      Translation2d approachHat,
      double pushM,
      java.util.function.Function<Translation2d, Translation2d> clampToFieldRobotSafe,
      java.util.function.Predicate<Translation2d> inForbidden,
      java.util.function.Predicate<Translation2d> violatesWall,
      java.util.function.Function<Translation2d, Translation2d> nudgeOutOfForbidden) {

    Translation2d t = clampToFieldRobotSafe.apply(resource.plus(approachHat.times(pushM)));
    if (inForbidden.test(t)) t = nudgeOutOfForbidden.apply(t);
    t = clampToFieldRobotSafe.apply(t);

    if (inForbidden.test(t) || violatesWall.test(t)) {
      Translation2d base = clampToFieldRobotSafe.apply(resource);
      if (inForbidden.test(base)) base = nudgeOutOfForbidden.apply(base);
      base = clampToFieldRobotSafe.apply(base);
      if (!inForbidden.test(base) && !violatesWall.test(base)) return base;
    }
    return t;
  }

  /**
   * Returns the fallback collect pose value maintained by this Repulsor component.
   *
   * @param robotPoseBlue value used by this operation.
   * @return value produced by this operation.
   */
  public Pose2d fallbackCollectPose(Pose2d robotPoseBlue) {
    // return new Pose2d(Constants.FIELD_LENGTH / 2, Constants.FIELD_WIDTH / 2, new Rotation2d());
    Translation2d p =
        collectStickyDriveTarget != null ? collectStickyDriveTarget : collectStickyPoint;
    if (p == null && lastBest != null) p = lastBest.point;
    if (p == null && robotPoseBlue != null) p = robotPoseBlue.getTranslation();
    if (p == null) p = new Translation2d();
    Rotation2d rot = robotPoseBlue != null ? robotPoseBlue.getRotation() : new Rotation2d();
    return new Pose2d(p, rot);
  }

  /**
   * Returns the next objective goal blue value maintained by this Repulsor component.
   *
   * @param robotPoseBlue value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param goalUnits value used by this operation.
   * @param cat value used by this operation.
   * @return value produced by this operation.
   */
  public Pose2d nextObjectiveGoalBlue(
      Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits, CategorySpec cat) {
    if (robotPoseBlue == null) return Pose2d.kZero;
    double cap = Math.max(0.2, ourSpeedCap);

    for (int pass = 0; pass < 2; pass++) {
      FieldTrackerCollectPassSetupResult setup =
          FieldTrackerCollectPassSetup.prepare(this, robotPoseBlue, cap);
      if (setup.immediatePose() != null) return setup.immediatePose();

      FieldTrackerCollectPassContext ctx = setup.context();
      FieldTrackerCollectPassCandidateResult candidate =
          FieldTrackerCollectPassCandidateStep.choose(this, ctx, goalUnits);
      if (candidate.immediatePose() != null) return candidate.immediatePose();

      FieldTrackerCollectPassStickyResult sticky =
          FieldTrackerCollectPassStickyStep.selectAndPrime(this, ctx, candidate, pass);
      if (sticky.immediatePose() != null) return sticky.immediatePose();

      return FieldTrackerCollectPassDriveStep.driveAndFinish(this, ctx, candidate, sticky, pass);
    }

    clearCollectSticky();
    return fallbackCollectPose(robotPoseBlue);
  }

  /**
   * Returns the force drive target onto fuel value maintained by this Repulsor component.
   *
   * @param robotPos value used by this operation.
   * @param cap value used by this operation.
   * @param collectPoint value used by this operation.
   * @param driveSeed value used by this operation.
   * @param clampToFieldRobotSafe value used by this operation.
   * @param inForbidden value used by this operation.
   * @param violatesWall value used by this operation.
   * @param nudgeOutOfForbidden value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d forceDriveTargetOntoFuel(
      Translation2d robotPos,
      double cap,
      Translation2d collectPoint,
      Translation2d driveSeed,
      java.util.function.Function<Translation2d, Translation2d> clampToFieldRobotSafe,
      java.util.function.Predicate<Translation2d> inForbidden,
      java.util.function.Predicate<Translation2d> violatesWall,
      java.util.function.Function<Translation2d, Translation2d> nudgeOutOfForbidden) {

    Translation2d drive = driveSeed != null ? driveSeed : collectPoint;
    if (drive == null) return null;

    drive = clampToFieldRobotSafe.apply(drive);
    if (inForbidden.test(drive)) drive = nudgeOutOfForbidden.apply(drive);
    drive = clampToFieldRobotSafe.apply(drive);

    if (inForbidden.test(drive) || violatesWall.test(drive)) return null;

    Translation2d near = predictor.nearestCollectResource(drive, COLLECT_SNAP_TO_NEAREST_FUEL_M);
    if (near != null) {
      Translation2d snapped = clampToFieldRobotSafe.apply(near);
      if (inForbidden.test(snapped)) snapped = nudgeOutOfForbidden.apply(snapped);
      snapped = clampToFieldRobotSafe.apply(snapped);
      if (!inForbidden.test(snapped) && !violatesWall.test(snapped)) return snapped;
    }

    Translation2d centroid = predictor.snapToCollectCentroid(drive, 0.75, 0.15);
    if (centroid != null) {
      centroid = clampToFieldRobotSafe.apply(centroid);
      if (inForbidden.test(centroid)) centroid = nudgeOutOfForbidden.apply(centroid);
      centroid = clampToFieldRobotSafe.apply(centroid);
      if (!inForbidden.test(centroid) && !violatesWall.test(centroid)) {
        Translation2d near2 =
            predictor.nearestCollectResource(centroid, COLLECT_SNAP_TO_NEAREST_FUEL_M);
        if (near2 != null) {
          Translation2d snapped2 = clampToFieldRobotSafe.apply(near2);
          if (inForbidden.test(snapped2)) snapped2 = nudgeOutOfForbidden.apply(snapped2);
          snapped2 = clampToFieldRobotSafe.apply(snapped2);
          if (!inForbidden.test(snapped2) && !violatesWall.test(snapped2)) return snapped2;
        }
        CollectProbe probe = predictor.probeCollect(centroid, COLLECT_FORCE_ON_FUEL_PROBE_R_M);
        if (probe != null && probe.count > 0 && probe.units >= COLLECT_FORCE_ON_FUEL_MIN_UNITS) {
          return centroid;
        }
      }
    }

    if (collectPoint != null) {
      Translation2d nearCollect =
          predictor.nearestCollectResource(
              collectPoint,
              Math.max(COLLECT_SNAP_TO_NEAREST_FUEL_M, COLLECT_FORCE_ON_FUEL_SEARCH_M));
      if (nearCollect != null) {
        Translation2d snappedCollect = clampToFieldRobotSafe.apply(nearCollect);
        if (inForbidden.test(snappedCollect))
          snappedCollect = nudgeOutOfForbidden.apply(snappedCollect);
        snappedCollect = clampToFieldRobotSafe.apply(snappedCollect);
        if (!inForbidden.test(snappedCollect) && !violatesWall.test(snappedCollect)) {
          return snappedCollect;
        }
      }
    }

    Translation2d fallback = clampToFieldRobotSafe.apply(collectPoint);
    if (inForbidden.test(fallback)) fallback = nudgeOutOfForbidden.apply(fallback);
    fallback = clampToFieldRobotSafe.apply(fallback);
    if (!inForbidden.test(fallback) && !violatesWall.test(fallback)) return fallback;

    return drive;
  }

  /**
   * Updates clear state state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   */
  public void clearState() {
    clearCollectSticky();
  }
}
