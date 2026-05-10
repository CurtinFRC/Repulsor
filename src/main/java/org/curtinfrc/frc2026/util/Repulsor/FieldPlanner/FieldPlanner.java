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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Diagnostics.RepulsorDecisionEntry;
import org.curtinfrc.frc2026.util.Repulsor.Diagnostics.RepulsorDecisionTrace;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.NtRepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.RepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathing;
import org.curtinfrc.frc2026.util.Repulsor.Fallback.PlannerFallback;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerForceModel;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerGeometry;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerGoalManager;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointObjectiveRole;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointPolicyProfile;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointStatus;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointStrategy;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldLayoutProvider;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Force;
import org.curtinfrc.frc2026.util.Repulsor.HeadingGate;
import org.curtinfrc.frc2026.util.Repulsor.Offload.FieldPlannerCalculateResultDTO;
import org.curtinfrc.frc2026.util.Repulsor.Offload.FieldPlannerOffloadEntrypoints_Offloaded;
import org.curtinfrc.frc2026.util.Repulsor.Offload.FieldPlannerPathingOffloadEntrypoints_Offloaded;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadExecutionContext;
import org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.ReactiveBypass;
import org.curtinfrc.frc2026.util.Repulsor.RepulsorDiagnostics;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultDriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultTurnTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.TurnTuning;
import org.littletonrobotics.junction.Logger;

/**
 * Provides field planner functionality for the Repulsor repulsor-field planner that combines goals,
 * obstacles, and force samples. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class FieldPlanner {
  /**
   * Configuration value for goal strength. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double GOAL_STRENGTH = 2.2;

  private static final ThreadLocal<Alliance> OFFLOAD_FALLBACK_ALLIANCE = new ThreadLocal<>();
  private static final boolean OFFLOAD_PATHING_ENABLED =
      Boolean.parseBoolean(
          System.getProperty("repulsor.offload.fieldplanner.pathing.enabled", "true"));
  private static final boolean OFFLOAD_CALCULATE_ENABLED =
      Boolean.parseBoolean(
          System.getProperty("repulsor.offload.fieldplanner.calculate.enabled", "true"));

  private static final class ClearMemo {
    Boolean toGoalDyn;
    Boolean toGoalNoDyn;

    boolean toGoalDyn(
        Translation2d a, Translation2d b, List<? extends Obstacle> dyn, double rx, double ry) {
      if (toGoalDyn != null) return toGoalDyn.booleanValue();
      toGoalDyn = isClearPath("Repulsor/IsClear", a, b, dyn, rx, ry, true);
      return toGoalDyn.booleanValue();
    }

    boolean toGoalNoDyn(Translation2d a, Translation2d b, double rx, double ry) {
      if (toGoalNoDyn != null) return toGoalNoDyn.booleanValue();
      toGoalNoDyn =
          isClearPath("Repulsor/ForceThrough/NoDyn", a, b, Collections.emptyList(), rx, ry, false);
      return toGoalNoDyn.booleanValue();
    }
  }

  /**
   * Contract for obstacle provider implementations used by the Repulsor repulsor-field planner that
   * combines goals, obstacles, and force samples. Use this type from robot code, field profiles, or
   * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
   * unless a method documents robot-relative motion.
   */
  public interface ObstacleProvider {
    /**
     * Returns the field obstacles value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    List<Obstacle> fieldObstacles();

    /**
     * Returns the walls value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    List<Obstacle> walls();
  }

  /**
   * Provides default obstacle provider functionality for the Repulsor repulsor-field planner that
   * combines goals, obstacles, and force samples. Use this type from robot code, field profiles, or
   * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
   * unless a method documents robot-relative motion.
   */
  public static final class DefaultObstacleProvider implements ObstacleProvider {
    /**
     * Returns the field obstacles value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public List<Obstacle> fieldObstacles() {
      return List.of();
    }

    /**
     * Returns the walls value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public List<Obstacle> walls() {
      return List.of();
    }
  }

  private Optional<RepulsorSetpoint> lastChosenSetpoint = Optional.empty();

  private final TurnTuning turnTuning;
  private final DriveTuning driveTuning;
  public final ReactiveBypass bypass = new ReactiveBypass();
  private final HeadingGate headingGate = new HeadingGate();

  private final ObstacleProvider obstacleProvider;
  private final List<Obstacle> fieldObstacles;
  private final List<Obstacle> walls;
  private final List<GatedAttractorObstacle> gatedAttractors = new ArrayList<>();
  private final double fieldLengthMeters;
  private final double fieldWidthMeters;

  private final FieldPlannerForceModel forceModel;
  private final FieldPlannerGoalManager goalManager;
  private volatile FieldPlannerRuntimeConfig runtimeConfig;
  private volatile CoarseGlobalPlanner globalPlanner;

  private Optional<Distance> currentErr = Optional.empty();
  private Optional<PlannerFallback> fallback = Optional.empty();
  private volatile RepulsorPlanningResult lastPlanningResult = RepulsorPlanningResult.empty();
  private volatile FieldPlannerCalculateResultDTO lastOffloadedCalculateResult;
  private Alliance fallbackAllianceOverride = null;

  /**
   * Configuration value for suppress is clear path. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public boolean suppressIsClearPath = false;

  private int stuckStepCount = 0;
  private static final int MAX_STUCK_STEPS = 40;

  /**
   * Returns the get obstacle provider value maintained by this Repulsor component.
   *
   * @return obstacle provider result for get obstacle provider.
   */
  public ObstacleProvider getObstacleProvider() {
    return obstacleProvider;
  }

  /** Returns the field planner value maintained by this Repulsor component. */
  public FieldPlanner() {
    this(new DefaultTurnTuning(), new DefaultDriveTuning(), Constants.FIELD);
  }

  /**
   * Returns the field planner value maintained by this Repulsor component.
   *
   * @param obstacleProvider obstacle set used for safety checks, costs, or replanning.
   * @param driveTuning value used by this operation.
   */
  public FieldPlanner(ObstacleProvider obstacleProvider, DriveTuning driveTuning) {
    this(new DefaultTurnTuning(), driveTuning, obstacleProvider);
  }

  /**
   * Returns the field planner value maintained by this Repulsor component.
   *
   * @param turnTuning value used by this operation.
   * @param driveTuning value used by this operation.
   */
  public FieldPlanner(TurnTuning turnTuning, DriveTuning driveTuning) {
    this(turnTuning, driveTuning, new DefaultObstacleProvider());
  }

  /**
   * Returns the field planner value maintained by this Repulsor component.
   *
   * @param turnTuning value used by this operation.
   * @param driveTuning value used by this operation.
   * @param obstacleProvider obstacle set used for safety checks, costs, or replanning.
   */
  public FieldPlanner(
      TurnTuning turnTuning, DriveTuning driveTuning, ObstacleProvider obstacleProvider) {
    this(
        turnTuning,
        driveTuning,
        obstacleProvider,
        waypointConfigFor(obstacleProvider),
        FieldPlannerWaypointStrategy.defaults(),
        runtimeConfigFor(obstacleProvider));
  }

  private static FieldPlannerWaypointConfig waypointConfigFor(ObstacleProvider obstacleProvider) {
    if (obstacleProvider instanceof FieldLayoutProvider field) {
      FieldPlannerWaypointConfig config = field.waypointConfig();
      return config == null ? FieldPlannerWaypointConfig.defaults() : config;
    }
    return FieldPlannerWaypointConfig.defaults();
  }

  private static FieldPlannerRuntimeConfig runtimeConfigFor(ObstacleProvider obstacleProvider) {
    if (obstacleProvider instanceof FieldLayoutProvider field) {
      FieldPlannerRuntimeConfig config = field.fieldPlannerRuntimeConfig();
      return config == null ? FieldPlannerRuntimeConfig.defaults() : config;
    }
    return FieldPlannerRuntimeConfig.defaults();
  }

  public FieldPlanner(
      TurnTuning turnTuning,
      DriveTuning driveTuning,
      ObstacleProvider obstacleProvider,
      FieldPlannerWaypointConfig waypointConfig) {
    this(
        turnTuning,
        driveTuning,
        obstacleProvider,
        waypointConfig,
        FieldPlannerWaypointStrategy.defaults(),
        runtimeConfigFor(obstacleProvider));
  }

  public FieldPlanner(
      TurnTuning turnTuning,
      DriveTuning driveTuning,
      ObstacleProvider obstacleProvider,
      FieldPlannerWaypointConfig waypointConfig,
      FieldPlannerWaypointStrategy waypointStrategy) {
    this(
        turnTuning,
        driveTuning,
        obstacleProvider,
        waypointConfig,
        waypointStrategy,
        runtimeConfigFor(obstacleProvider));
  }

  public FieldPlanner(
      TurnTuning turnTuning,
      DriveTuning driveTuning,
      ObstacleProvider obstacleProvider,
      FieldPlannerWaypointConfig waypointConfig,
      FieldPlannerWaypointStrategy waypointStrategy,
      FieldPlannerRuntimeConfig runtimeConfig) {
    this.turnTuning = turnTuning;
    this.driveTuning = driveTuning;
    this.obstacleProvider =
        obstacleProvider == null ? new DefaultObstacleProvider() : obstacleProvider;
    this.runtimeConfig =
        runtimeConfig == null ? FieldPlannerRuntimeConfig.defaults() : runtimeConfig;
    this.globalPlanner = new CoarseGlobalPlanner(this.runtimeConfig.globalFallbackConfig());
    if (this.obstacleProvider instanceof FieldLayoutProvider field) {
      FieldGeometry geometry = field.geometry();
      this.fieldLengthMeters = geometry.lengthMeters();
      this.fieldWidthMeters = geometry.widthMeters();
    } else {
      this.fieldLengthMeters = Constants.FIELD_LENGTH;
      this.fieldWidthMeters = Constants.FIELD_WIDTH;
    }
    this.fieldObstacles = new ArrayList<>(this.obstacleProvider.fieldObstacles());
    this.walls = new ArrayList<>(this.obstacleProvider.walls());

    for (Obstacle obs : this.fieldObstacles) {
      if (obs instanceof GatedAttractorObstacle gated) {
        if (gated.waypoint) {
          gatedAttractors.add(gated);
        }
      }
    }

    this.forceModel =
        new FieldPlannerForceModel(fieldObstacles, walls, fieldLengthMeters, fieldWidthMeters);
    this.goalManager =
        new FieldPlannerGoalManager(
            gatedAttractors, fieldLengthMeters, fieldWidthMeters, waypointConfig, waypointStrategy);

    String prefix = System.getenv("REACTIVE_BYPASS_ID");
    String logName;
    if (prefix != null && !prefix.isEmpty()) {
      logName = prefix + "ReactiveBypassLog.csv";
    } else {
      logName = "ReactiveBypassLog.csv";
    }
    // bypass.enableLogging(logName);
  }

  /**
   * Returns the segment intersects polygon outer value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param poly distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public static boolean segmentIntersectsPolygonOuter(
      Translation2d a, Translation2d b, Translation2d[] poly) {
    return FieldPlannerGeometry.segmentIntersectsPolygonOuter(a, b, poly);
  }

  /**
   * Returns the robot rect value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param yaw value used by this operation.
   * @param rx distance or field-coordinate value in meters.
   * @param ry distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public static Translation2d[] robotRect(
      Translation2d center, Rotation2d yaw, double rx, double ry) {
    return TurnTuning.robotRect(center, yaw, rx, ry);
  }

  private boolean rectIntersectsDynamic(Translation2d[] rect, List<? extends Obstacle> dynamics) {
    for (Obstacle d : dynamics) if (d.intersectsRectangle(rect)) return true;
    return false;
  }

  private boolean rectIntersectsAny(Translation2d[] rect, List<? extends Obstacle> dynamics) {
    for (Obstacle w : walls) if (w.intersectsRectangle(rect)) return true;
    for (Obstacle f : fieldObstacles) if (f.intersectsRectangle(rect)) return true;
    for (Obstacle d : dynamics) if (d.intersectsRectangle(rect)) return true;
    return false;
  }

  /**
   * Returns the get obstacles value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public List<Obstacle> getObstacles() {
    return fieldObstacles;
  }

  /**
   * Returns the get goal value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Translation2d getGoal() {
    return goalManager.getGoalTranslation();
  }

  /**
   * Returns the get goal pose value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Pose2d getGoalPose() {
    return goalManager.getGoalPose();
  }

  public CoarseGlobalPlannerStats getGlobalFallbackStats() {
    return globalPlanner.lastStats();
  }

  public FieldPlannerWaypointConfig getWaypointConfig() {
    return goalManager.getWaypointConfig();
  }

  public FieldPlannerRuntimeConfig getRuntimeConfig() {
    return runtimeConfig;
  }

  public void setRuntimeConfig(FieldPlannerRuntimeConfig runtimeConfig) {
    FieldPlannerRuntimeConfig safeConfig =
        runtimeConfig == null ? FieldPlannerRuntimeConfig.defaults() : runtimeConfig;
    this.runtimeConfig = safeConfig;
    this.globalPlanner = new CoarseGlobalPlanner(safeConfig.globalFallbackConfig());
  }

  public FieldPlannerWaypointStatus getWaypointStatus() {
    return goalManager.getWaypointStatus();
  }

  public void setWaypointStrategy(FieldPlannerWaypointStrategy waypointStrategy) {
    goalManager.setWaypointStrategy(waypointStrategy);
  }

  public void setWaypointPolicyProfile(FieldPlannerWaypointPolicyProfile profile) {
    goalManager.setWaypointPolicyProfile(profile);
  }

  /**
   * Returns the get requested goal pose value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Pose2d getRequestedGoalPose() {
    return goalManager.getRequestedGoalPose();
  }

  /**
   * Returns the with fallback value maintained by this Repulsor component.
   *
   * @param _fallback value used by this operation.
   * @return field planner result for with fallback.
   */
  public FieldPlanner withFallback(PlannerFallback _fallback) {
    fallback = Optional.of(_fallback);
    return this;
  }

  /**
   * Updates update arrows state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param dynamicObstacles obstacle set used for safety checks, costs, or replanning.
   */
  public void updateArrows(List<? extends Obstacle> dynamicObstacles) {
    forceModel.updateArrows(goalManager.getGoalTranslation(), dynamicObstacles);
  }

  /**
   * Returns the get arrows value maintained by this Repulsor component.
   *
   * @return array list of pose2d result for get arrows.
   */
  public ArrayList<Pose2d> getArrows() {
    return forceModel.getArrows();
  }

  /**
   * Returns the get goal force value maintained by this Repulsor component.
   *
   * @param curLocation value used by this operation.
   * @param goal value used by this operation.
   * @return value produced by this operation.
   */
  Force getGoalForce(Translation2d curLocation, Translation2d goal) {
    return forceModel.getGoalForce(curLocation, goal);
  }

  /**
   * Returns the get wall force value maintained by this Repulsor component.
   *
   * @param curLocation value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  Force getWallForce(Translation2d curLocation, Translation2d target) {
    return forceModel.getWallForce(curLocation, target);
  }

  /**
   * Returns the get obstacle force value maintained by this Repulsor component.
   *
   * @param curLocation value used by this operation.
   * @param target value used by this operation.
   * @param extra value used by this operation.
   * @return value produced by this operation.
   */
  Force getObstacleForce(
      Translation2d curLocation, Translation2d target, List<? extends Obstacle> extra) {
    return forceModel.getObstacleForce(curLocation, target, extra);
  }

  /**
   * Returns the get obstacle force value maintained by this Repulsor component.
   *
   * @param curLocation value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  Force getObstacleForce(Translation2d curLocation, Translation2d target) {
    return forceModel.getObstacleForce(curLocation, target);
  }

  /**
   * Returns the get force value maintained by this Repulsor component.
   *
   * @param curLocation value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  Force getForce(Translation2d curLocation, Translation2d target) {
    return forceModel.getForce(curLocation, target);
  }

  /**
   * Updates set requested goal state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param requested value used by this operation.
   */
  public void setRequestedGoal(Pose2d requested) {
    goalManager.setRequestedGoal(requested);
    lastChosenSetpoint = Optional.empty();
  }

  /**
   * Runs sync goal manager state in the Repulsor runtime.
   *
   * @param requested value used by this operation.
   * @param active value used by this operation.
   */
  public void syncGoalManagerState(Pose2d requested, Pose2d active) {
    Pose2d requestedGoal = requested == null ? Pose2d.kZero : requested;
    setRequestedGoal(requestedGoal);
    setActiveGoal(active == null ? requestedGoal : active);
  }

  /**
   * Updates set active goal state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param active value used by this operation.
   */
  void setActiveGoal(Pose2d active) {
    goalManager.setActiveGoal(active);
  }

  /**
   * Returns the get err value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Optional<Distance> getErr() {
    return currentErr;
  }

  /**
   * Updates clear committed state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   */
  public void clearCommitted() {}

  /**
   * Computes the calculate and clear value for the current Repulsor planning state. Call this from
   * periodic planning or tests when a fresh decision is required; inputs should already be
   * expressed in the coordinate frame expected by the parameter names.
   *
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @param dynamicObstacles obstacle set used for safety checks, costs, or replanning.
   * @param robot_x distance or field-coordinate value in meters.
   * @param robot_y distance or field-coordinate value in meters.
   * @param cat value used by this operation.
   * @param shooterReleaseHeightMeters distance or field-coordinate value in meters.
   * @return repulsor sample result for calculate and clear.
   */
  public RepulsorSample calculateAndClear(
      Pose2d pose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      CategorySpec cat,
      double shooterReleaseHeightMeters) {
    return calculate(
        pose, dynamicObstacles, robot_x, robot_y, cat, false, shooterReleaseHeightMeters);
  }

  /** Computes a planner sample from an immutable request object. */
  public RepulsorSample calculate(PlannerCalculationRequest request) {
    if (request == null) {
      return new RepulsorSample(Pose2d.kZero.getTranslation(), 0, 0, Radians.of(0.0));
    }
    return calculate(
        request.pose(),
        request.dynamicObstacles(),
        request.robotHalfLengthMeters(),
        request.robotHalfWidthMeters(),
        request.category(),
        request.suppressFallback(),
        request.shooterReleaseHeightMeters());
  }

  public RepulsorPlanningResult calculateDetailed(RepulsorPlanningRequest request) {
    RepulsorPlanningRequest safeRequest =
        request == null ? RepulsorPlanningRequest.from(null) : request;
    Alliance previousOverride = fallbackAllianceOverride;
    fallbackAllianceOverride = safeRequest.fallbackAllianceOverride().orElse(null);
    try {
      RepulsorSample sample =
          calculate(
              safeRequest.pose(),
              safeRequest.dynamicObstacles(),
              safeRequest.robotHalfLengthMeters(),
              safeRequest.robotHalfWidthMeters(),
              safeRequest.category(),
              safeRequest.suppressFallback(),
              safeRequest.shooterReleaseHeightMeters());
      lastPlanningResult =
          new RepulsorPlanningResult(
              safeRequest,
              sample,
              lastPlanningResult.diagnostics(),
              lastPlanningResult.decisionTrace());
      return lastPlanningResult;
    } finally {
      fallbackAllianceOverride = previousOverride;
    }
  }

  public RepulsorPlanningResult lastPlanningResult() {
    return lastPlanningResult;
  }

  /**
   * Computes the calculate value for the current Repulsor planning state. Call this from periodic
   * planning or tests when a fresh decision is required; inputs should already be expressed in the
   * coordinate frame expected by the parameter names.
   *
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @param dynamicObstacles obstacle set used for safety checks, costs, or replanning.
   * @param robot_x distance or field-coordinate value in meters.
   * @param robot_y distance or field-coordinate value in meters.
   * @param cat value used by this operation.
   * @param suppressFallback value used by this operation.
   * @param shooterReleaseHeightMeters distance or field-coordinate value in meters.
   * @return repulsor sample result for calculate.
   */
  public RepulsorSample calculate(
      Pose2d pose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      CategorySpec cat,
      boolean suppressFallback,
      double shooterReleaseHeightMeters) {
    RepulsorPlanningRequest planningRequest =
        new RepulsorPlanningRequest(
            pose,
            dynamicObstacles,
            robot_x,
            robot_y,
            cat,
            suppressFallback,
            shooterReleaseHeightMeters,
            preferredAllianceForFallback(),
            "default");

    if (OFFLOAD_CALCULATE_ENABLED && !isOffloadWorkerThread()) {
      try {
        RepulsorSample sample =
            calculateOffloaded(
                pose,
                dynamicObstacles,
                robot_x,
                robot_y,
                cat,
                suppressFallback,
                shooterReleaseHeightMeters);
        return finishOffloadedPlanningResult(planningRequest, sample, lastOffloadedCalculateResult);
      } catch (RuntimeException ex) {
        RepulsorDiagnostics.warnThrottled(
            "FieldPlanner/offloadCalculateFallback",
            "Offloaded FieldPlanner.calculate failed, using local planner: " + ex.getMessage(),
            2.0);
        // If remote calculate fails, continue with local calculate behavior.
      }
    }

    Logger.recordOutput("CalculateCalled", true);

    Translation2d curTrans = pose.getTranslation();
    double distToGoal = curTrans.getDistance(goalManager.getGoalTranslation());

    var dsBase = safeDriverStation();
    if (!isOffloadWorkerThread() && dsBase instanceof NtRepulsorDriverStation ds) {
      ds.forcedGoalPose("main").ifPresent(this::setRequestedGoal);
    }

    boolean slowDown =
        goalManager.updateStagedGoal(
            curTrans, dynamicObstacles, FieldPlannerWaypointObjectiveRole.fromCategory(cat));
    distToGoal = curTrans.getDistance(goalManager.getGoalTranslation());
    Pose2d calculationGoal = goalManager.getGoalPose();
    Translation2d calculationGoalTranslation = calculationGoal.getTranslation();

    ClearMemo memo = new ClearMemo();

    boolean forceThrough = bypass.isPinnedMode();
    List<? extends Obstacle> effectiveDynamics =
        forceThrough ? Collections.emptyList() : dynamicObstacles;
    boolean pathBlocked = false;
    boolean robotIntersecting = false;
    boolean globalFallbackActive = false;
    Optional<Pose2d> globalFallbackWaypoint = Optional.empty();
    boolean reactiveBypassActive = false;
    boolean stuckAbort = false;

    if (!forceThrough && !suppressFallback) {
      boolean blockedWithDynamics =
          !isClearPath(
              "Repulsor/ForceThrough/WithDyn",
              curTrans,
              goalManager.getGoalTranslation(),
              dynamicObstacles,
              robot_x,
              robot_y,
              false);

      boolean blockedWithoutDynamics =
          !memo.toGoalNoDyn(curTrans, goalManager.getGoalTranslation(), robot_x, robot_y);

      double dxWall = Math.min(curTrans.getX(), fieldLengthMeters - curTrans.getX());
      double dyWall = Math.min(curTrans.getY(), fieldWidthMeters - curTrans.getY());
      double dWall = Math.min(dxWall, dyWall);
      boolean nearWall = dWall < runtimeConfig.forceThroughWallDistanceMeters();
      boolean nearGoal = distToGoal <= runtimeConfig.forceThroughGoalDistanceMeters();

      if (blockedWithDynamics && !blockedWithoutDynamics && nearGoal && nearWall) {
        forceThrough = true;
        effectiveDynamics = Collections.emptyList();
      }
    }

    if (!suppressFallback) {
      if (!forceThrough && robotIntersects(curTrans, robot_x, robot_y, dynamicObstacles)) {
        robotIntersecting = true;
        currentErr = Optional.of(Meters.of(curTrans.getDistance(goalManager.getGoalTranslation())));
        return finishPlanningResult(
            planningRequest,
            new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians())),
            pathBlocked,
            forceThrough,
            globalFallbackActive,
            globalFallbackWaypoint,
            reactiveBypassActive,
            robotIntersecting,
            stuckAbort,
            false);
      }

      if (!suppressIsClearPath) {
        pathBlocked =
            !memo.toGoalDyn(
                curTrans, goalManager.getGoalTranslation(), effectiveDynamics, robot_x, robot_y);
      }

      if (pathBlocked && !suppressFallback) {
        Alliance preferred = preferredAllianceForFallback();

        var cands =
            FieldTrackerCore.getInstance().getPredictedSetpoints(preferred, curTrans, 3.5, cat, 8);

        SetpointContext spCtx =
            new SetpointContext(
                Optional.of(pose),
                Math.max(0.0, robot_x) * 2.0,
                Math.max(0.0, robot_y) * 2.0,
                shooterReleaseHeightMeters,
                effectiveDynamics);

        for (RepulsorSetpoint sp : cands) {
          Pose2d altGoal = sp.get(spCtx);

          if (altGoal.getTranslation().getDistance(goalManager.getGoalTranslation()) < 1e-3)
            continue;

          boolean clear =
              isClearPath(
                  "Repulsor/IsClear/Reroute",
                  curTrans,
                  altGoal.getTranslation(),
                  effectiveDynamics,
                  robot_x,
                  robot_y,
                  true);

          if (clear) {
            setActiveGoal(altGoal);
            calculationGoal = altGoal;
            calculationGoalTranslation = altGoal.getTranslation();
            lastChosenSetpoint = Optional.of(sp);
            pathBlocked = false;
            break;
          }
        }

        if (pathBlocked && runtimeConfig.globalFallbackEnabled()) {
          ArrayList<Obstacle> globalObstacles =
              new ArrayList<>(fieldObstacles.size() + walls.size() + effectiveDynamics.size());
          globalObstacles.addAll(fieldObstacles);
          globalObstacles.addAll(walls);
          globalObstacles.addAll(effectiveDynamics);
          Optional<Pose2d> waypoint =
              globalPlanner.nextWaypoint(
                  curTrans,
                  goalManager.getGoalPose(),
                  globalObstacles,
                  robot_x,
                  robot_y,
                  fieldLengthMeters,
                  fieldWidthMeters);
          if (waypoint.isPresent()) {
            calculationGoal = waypoint.get();
            calculationGoalTranslation = waypoint.get().getTranslation();
            lastChosenSetpoint = Optional.empty();
            pathBlocked = false;
            globalFallbackActive = true;
            globalFallbackWaypoint = waypoint;
          }
        }

        recordGlobalFallbackTelemetry(globalFallbackActive, globalFallbackWaypoint);

        if (pathBlocked) {
          return finishPlanningResult(
              planningRequest,
              new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians())),
              true,
              forceThrough,
              globalFallbackActive,
              globalFallbackWaypoint,
              reactiveBypassActive,
              robotIntersecting,
              stuckAbort,
              false);
        }
      }
    }

    final List<? extends Obstacle> effectiveDynamicsFinal = effectiveDynamics;
    final Pose2d calculationGoalFinal = calculationGoal;
    final Translation2d calculationGoalTranslationFinal = calculationGoalTranslation;

    forceModel.updateArrows(calculationGoalTranslationFinal, effectiveDynamicsFinal);

    var err = curTrans.minus(calculationGoalTranslationFinal);
    currentErr = Optional.of(Meters.of(err.getNorm()));

    if (err.getNorm() < 0.04) {
      return finishPlanningResult(
          planningRequest,
          new RepulsorSample(
              curTrans, 0, 0, Radians.of(calculationGoalFinal.getRotation().getRadians())),
          pathBlocked,
          forceThrough,
          globalFallbackActive,
          globalFallbackWaypoint,
          reactiveBypassActive,
          robotIntersecting,
          stuckAbort,
          false);
    }

    if (fallback.isPresent() && fallback.get().within(err)) {
      var speeds = fallback.get().calculate(curTrans, calculationGoalTranslationFinal);
      return finishPlanningResult(
          planningRequest,
          new RepulsorSample(
              calculationGoalTranslationFinal, speeds, Radians.of(pose.getRotation().getRadians())),
          pathBlocked,
          forceThrough,
          globalFallbackActive,
          globalFallbackWaypoint,
          reactiveBypassActive,
          robotIntersecting,
          stuckAbort,
          false);
    }

    var obstacleForceToGoal =
        getObstacleForce(curTrans, calculationGoalTranslationFinal, effectiveDynamicsFinal)
            .plus(getWallForce(curTrans, calculationGoalTranslationFinal));
    boolean clearPathToGoalForForce =
        !suppressIsClearPath
            && isClearPath(
                "Repulsor/Force/ClearCalculationGoal",
                curTrans,
                calculationGoalTranslationFinal,
                effectiveDynamicsFinal,
                robot_x,
                robot_y,
                false);
    obstacleForceToGoal =
        removeBackwardObstacleForceWhenClear(
            obstacleForceToGoal,
            curTrans,
            calculationGoalTranslationFinal,
            clearPathToGoalForForce);
    var netForceToGoal =
        getGoalForce(curTrans, calculationGoalTranslationFinal).plus(obstacleForceToGoal);
    Rotation2d headingToGoal = netForceToGoal.getAngle();

    var maybeBypass =
        bypass.update(
            pose,
            calculationGoalFinal,
            headingToGoal,
            driveTuning.dtSeconds(),
            robot_x,
            robot_y,
            effectiveDynamicsFinal,
            rect -> rectIntersectsDynamic(rect, effectiveDynamicsFinal),
            tag ->
                isClearPath(
                    "Repulsor/Bypass/Rejoin",
                    curTrans,
                    calculationGoalTranslationFinal,
                    effectiveDynamicsFinal,
                    robot_x,
                    robot_y,
                    true));

    Pose2d effectiveGoal = maybeBypass.orElse(calculationGoalFinal);
    reactiveBypassActive = maybeBypass.isPresent();

    var obstacleForce =
        getObstacleForce(curTrans, effectiveGoal.getTranslation(), effectiveDynamicsFinal)
            .plus(getWallForce(curTrans, effectiveGoal.getTranslation()));
    boolean clearPathToEffectiveGoal =
        !suppressIsClearPath
            && (!maybeBypass.isPresent()
                ? clearPathToGoalForForce
                : isClearPath(
                    "Repulsor/ClearEffectiveGoal",
                    curTrans,
                    effectiveGoal.getTranslation(),
                    effectiveDynamicsFinal,
                    robot_x,
                    robot_y,
                    false));
    obstacleForce =
        removeBackwardObstacleForceWhenClear(
            obstacleForce, curTrans, effectiveGoal.getTranslation(), clearPathToEffectiveGoal);
    var netForce = getGoalForce(curTrans, effectiveGoal.getTranslation()).plus(obstacleForce);
    var dist = curTrans.getDistance(effectiveGoal.getTranslation());

    double stepSize_m =
        driveTuning.stepSizeMeters(
            dist, obstacleForce.getNorm(), (cat == CategorySpec.kScore), slowDown);
    var step = new Translation2d(stepSize_m, netForce.getAngle());

    if (step.getNorm() < 1e-3) {
      stuckStepCount++;
    } else {
      stuckStepCount = 0;
    }

    if (stuckStepCount >= MAX_STUCK_STEPS) {
      stuckAbort = true;
      RepulsorDiagnostics.warnThrottled(
          "FieldPlanner/stuck",
          "Planner stuck, aborting after " + stuckStepCount + " tiny steps.",
          1.0);
      return finishPlanningResult(
          planningRequest,
          new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians())),
          pathBlocked,
          forceThrough,
          globalFallbackActive,
          globalFallbackWaypoint,
          reactiveBypassActive,
          robotIntersecting,
          stuckAbort,
          false);
    }

    Rotation2d desiredHeadingRaw =
        (cat == CategorySpec.kCollect) ? effectiveGoal.getRotation() : netForce.getAngle();
    Rotation2d desiredHeading =
        headingGate.filter(pose.getRotation(), desiredHeadingRaw, driveTuning.dtSeconds());

    var turn =
        turnTuning.plan(
            pose,
            effectiveGoal,
            desiredHeading,
            step,
            (cat == CategorySpec.kScore),
            robot_x,
            robot_y,
            rect -> rectIntersectsAny(rect, effectiveDynamicsFinal));

    step = step.times(turn.speedScale);

    if (!isOffloadWorkerThread() && !RobotBase.isReal()) {
      Pose2d arrowPose = new Pose2d(curTrans, netForce.getAngle());
    }

    return finishPlanningResult(
        planningRequest,
        new RepulsorSample(
            effectiveGoal.getTranslation(),
            step.getX() / driveTuning.dtSeconds(),
            step.getY() / driveTuning.dtSeconds(),
            Radians.of(turn.yaw.getRadians())),
        pathBlocked,
        forceThrough,
        globalFallbackActive,
        globalFallbackWaypoint,
        reactiveBypassActive,
        robotIntersecting,
        stuckAbort,
        false);
  }

  private RepulsorSample finishOffloadedPlanningResult(
      RepulsorPlanningRequest request,
      RepulsorSample sample,
      FieldPlannerCalculateResultDTO remote) {
    if (remote == null) {
      return finishPlanningResult(
          request,
          sample,
          false,
          bypass.isPinnedMode(),
          false,
          Optional.empty(),
          false,
          false,
          false,
          true);
    }

    Pose2d requestedGoal = goalManager.getRequestedGoalPose();
    Pose2d activeGoal = goalManager.getGoalPose();
    FieldPlannerWaypointStatus waypointStatus =
        new FieldPlannerWaypointStatus(
            requestedGoal,
            activeGoal,
            null,
            null,
            remote.getSelectedCandidateReason(),
            remote.isWaypointActiveStage(),
            null,
            null,
            null,
            false,
            false,
            remote.isWaypointUsingBypass(),
            false,
            remote.getWaypointStagedModeTicks());
    CoarseGlobalPlannerStats globalStats =
        new CoarseGlobalPlannerStats(
            remote.isGlobalFallbackFound(),
            remote.isGlobalFallbackTimedOut(),
            remote.isGlobalFallbackExhaustedNodeBudget(),
            remote.getGlobalFallbackExpandedNodes(),
            remote.getGlobalFallbackGeneratedNodes(),
            remote.getGlobalFallbackRawPathNodes(),
            remote.getGlobalFallbackPathNodes(),
            remote.getGlobalFallbackElapsedNanos(),
            parseGlobalFallbackFailureReason(remote.getGlobalFallbackFailureReason()));
    Optional<Pose2d> globalWaypoint =
        remote.isHasGlobalFallbackWaypoint()
            ? Optional.of(
                new Pose2d(
                    remote.getGlobalFallbackWaypointX(),
                    remote.getGlobalFallbackWaypointY(),
                    Rotation2d.fromRadians(remote.getGlobalFallbackWaypointThetaRadians())))
            : Optional.empty();
    RepulsorDiagnosticsSnapshot diagnostics =
        new RepulsorDiagnosticsSnapshot(
            requestedGoal,
            activeGoal,
            waypointStatus,
            globalStats,
            remote.isGlobalFallbackActive(),
            globalWaypoint,
            remote.isReactiveBypassActive(),
            remote.isReactiveBypassPinned(),
            remote.isForceThroughActive(),
            remote.isPathBlocked(),
            remote.isRobotIntersecting(),
            remote.isStuckAbort(),
            true,
            currentErr.map(distance -> distance.in(Meters)).orElse(Double.NaN));
    RepulsorDecisionTrace trace = buildDecisionTrace(diagnostics);
    lastPlanningResult = new RepulsorPlanningResult(request, sample, diagnostics, trace);
    Logger.recordOutput("Repulsor/Diagnostics/PathBlocked", remote.isPathBlocked());
    Logger.recordOutput(
        "Repulsor/Diagnostics/ReactiveBypassActive", remote.isReactiveBypassActive());
    Logger.recordOutput("Repulsor/Diagnostics/ForceThrough", remote.isForceThroughActive());
    Logger.recordOutput("Repulsor/Diagnostics/RobotIntersecting", remote.isRobotIntersecting());
    Logger.recordOutput("Repulsor/Diagnostics/StuckAbort", remote.isStuckAbort());
    Logger.recordOutput("Repulsor/Diagnostics/Offloaded", true);
    recordDecisionTraceTelemetry(trace);
    return sample;
  }

  private static CoarseGlobalPlannerFailureReason parseGlobalFallbackFailureReason(String value) {
    if (value == null || value.isBlank()) return CoarseGlobalPlannerFailureReason.NONE;
    try {
      return CoarseGlobalPlannerFailureReason.valueOf(value);
    } catch (IllegalArgumentException ignored) {
      return CoarseGlobalPlannerFailureReason.NONE;
    }
  }

  private void recordGlobalFallbackTelemetry(boolean active, Optional<Pose2d> waypoint) {
    CoarseGlobalPlannerStats stats = globalPlanner.lastStats();
    Logger.recordOutput("Repulsor/GlobalFallback/Active", active);
    Logger.recordOutput("Repulsor/GlobalFallback/Found", stats.found());
    Logger.recordOutput("Repulsor/GlobalFallback/TimedOut", stats.timedOut());
    Logger.recordOutput("Repulsor/GlobalFallback/ExhaustedNodeBudget", stats.exhaustedNodeBudget());
    Logger.recordOutput("Repulsor/GlobalFallback/ExpandedNodes", stats.expandedNodes());
    Logger.recordOutput("Repulsor/GlobalFallback/GeneratedNodes", stats.generatedNodes());
    Logger.recordOutput("Repulsor/GlobalFallback/RawPathNodes", stats.rawPathNodes());
    Logger.recordOutput("Repulsor/GlobalFallback/PathNodes", stats.pathNodes());
    Logger.recordOutput("Repulsor/GlobalFallback/FailureReason", stats.failureReason().name());
    Logger.recordOutput("Repulsor/GlobalFallback/ElapsedMs", stats.elapsedNanos() / 1.0e6);
    Logger.recordOutput("Repulsor/GlobalFallback/Waypoint", waypoint.orElse(Pose2d.kZero));
  }

  private RepulsorSample finishPlanningResult(
      RepulsorPlanningRequest request,
      RepulsorSample sample,
      boolean pathBlocked,
      boolean forceThrough,
      boolean globalFallbackActive,
      Optional<Pose2d> globalFallbackWaypoint,
      boolean reactiveBypassActive,
      boolean robotIntersecting,
      boolean stuckAbort,
      boolean offloaded) {
    RepulsorDiagnosticsSnapshot diagnostics =
        new RepulsorDiagnosticsSnapshot(
            goalManager.getRequestedGoalPose(),
            goalManager.getGoalPose(),
            goalManager.getWaypointStatus(),
            globalPlanner.lastStats(),
            globalFallbackActive,
            globalFallbackWaypoint,
            reactiveBypassActive,
            bypass.isPinnedMode(),
            forceThrough,
            pathBlocked,
            robotIntersecting,
            stuckAbort,
            offloaded,
            currentErr.map(distance -> distance.in(Meters)).orElse(Double.NaN));
    RepulsorDecisionTrace trace = buildDecisionTrace(diagnostics);
    lastPlanningResult = new RepulsorPlanningResult(request, sample, diagnostics, trace);
    Logger.recordOutput("Repulsor/Diagnostics/PathBlocked", pathBlocked);
    Logger.recordOutput("Repulsor/Diagnostics/ReactiveBypassActive", reactiveBypassActive);
    Logger.recordOutput("Repulsor/Diagnostics/ForceThrough", forceThrough);
    Logger.recordOutput("Repulsor/Diagnostics/RobotIntersecting", robotIntersecting);
    Logger.recordOutput("Repulsor/Diagnostics/StuckAbort", stuckAbort);
    Logger.recordOutput("Repulsor/Diagnostics/Offloaded", offloaded);
    recordDecisionTraceTelemetry(trace);
    return sample;
  }

  private RepulsorDecisionTrace buildDecisionTrace(RepulsorDiagnosticsSnapshot diagnostics) {
    if (diagnostics == null) return RepulsorDecisionTrace.empty();
    ArrayList<RepulsorDecisionEntry> entries = new ArrayList<>();

    FieldPlannerWaypointStatus waypoint = diagnostics.waypointStatus();
    String waypointDecision = "direct";
    String waypointReason = "using_active_goal";
    Map<String, String> waypointMetadata = Map.of();
    if (waypoint != null) {
      if (waypoint.usingBypass()) {
        waypointDecision = "bypass";
        waypointReason = waypoint.transitionReason();
      } else if (waypoint.activeStage()) {
        waypointDecision = "stage";
        waypointReason = waypoint.transitionReason();
      } else if (waypoint.stagedComplete()) {
        waypointDecision = "complete";
        waypointReason = waypoint.transitionReason();
      } else {
        waypointReason = waypoint.transitionReason();
      }
      waypointMetadata =
          Map.of(
              "objectiveRole",
              waypoint.lastObjectiveRole().name(),
              "strategyMode",
              waypoint.lastStrategyDecision().mode().name(),
              "transitionReason",
              waypoint.transitionReason(),
              "exitPhase",
              Boolean.toString(waypoint.exitPhase()),
              "centerReturn",
              Boolean.toString(waypoint.centerReturn()),
              "stagedModeTicks",
              Integer.toString(waypoint.stagedModeTicks()));
    }
    entries.add(
        new RepulsorDecisionEntry(
            "WaypointPolicy",
            waypointDecision,
            waypointReason,
            formatPose(diagnostics.activeGoal()),
            formatPose(diagnostics.requestedGoal()),
            0.0,
            0.0,
            waypointMetadata));

    CoarseGlobalPlannerStats stats = diagnostics.globalFallbackStats();
    String fallbackDecision = diagnostics.globalFallbackActive() ? "active" : "not_used";
    String fallbackReason = "clear_or_disabled";
    if (diagnostics.globalFallbackActive()) {
      if (diagnostics.globalFallbackWaypoint().isPresent()) {
        fallbackDecision = "temporary_waypoint";
        fallbackReason = stats.found() ? "search_found_waypoint" : stats.failureReason().name();
      } else if (stats.timedOut()) {
        fallbackDecision = "timeout";
        fallbackReason = stats.failureReason().name();
      } else if (stats.exhaustedNodeBudget()) {
        fallbackDecision = "node_budget";
        fallbackReason = stats.failureReason().name();
      } else {
        fallbackDecision = "active_no_waypoint";
        fallbackReason = stats.failureReason().name();
      }
    }
    entries.add(
        new RepulsorDecisionEntry(
            "GlobalFallback",
            fallbackDecision,
            fallbackReason,
            diagnostics.globalFallbackWaypoint().map(this::formatPose).orElse(""),
            formatPose(diagnostics.requestedGoal()),
            0.0,
            0.0,
            Map.of(
                "found",
                Boolean.toString(stats.found()),
                "timedOut",
                Boolean.toString(stats.timedOut()),
                "exhaustedNodeBudget",
                Boolean.toString(stats.exhaustedNodeBudget()),
                "expandedNodes",
                Integer.toString(stats.expandedNodes()),
                "failureReason",
                stats.failureReason().name(),
                "pathNodes",
                Integer.toString(stats.pathNodes()))));

    String bypassDecision = diagnostics.reactiveBypassActive() ? "active" : "inactive";
    String bypassReason = diagnostics.reactiveBypassActive() ? "local_path_blocked" : "not_needed";
    if (diagnostics.reactiveBypassPinned()) {
      bypassDecision = "pinned";
      bypassReason = "bypass_pinned_mode";
    }
    entries.add(
        new RepulsorDecisionEntry(
            "ReactiveBypass",
            bypassDecision,
            bypassReason,
            formatPose(diagnostics.activeGoal()),
            formatPose(diagnostics.requestedGoal()),
            0.0,
            0.0,
            Map.of(
                "pathBlocked",
                Boolean.toString(diagnostics.pathBlocked()),
                "robotIntersecting",
                Boolean.toString(diagnostics.robotIntersecting()))));

    entries.add(
        RepulsorDecisionEntry.of(
                "ForceThrough",
                diagnostics.forceThroughActive() ? "active" : "inactive",
                diagnostics.forceThroughActive() ? "near_goal_or_wall" : "not_needed")
            .withSelection(
                formatPose(diagnostics.activeGoal()), formatPose(diagnostics.requestedGoal())));

    String plannerDecision = diagnostics.offloaded() ? "offloaded" : "local";
    String plannerReason =
        diagnostics.offloaded() ? "offload_calculation_result" : "local_calculation";
    if (diagnostics.stuckAbort()) {
      plannerDecision = "stuck_abort";
      plannerReason = "tiny_step_limit_exceeded";
    } else if (diagnostics.pathBlocked()) {
      plannerDecision = "path_blocked";
      plannerReason = "no_clear_path_or_fallback";
    }
    entries.add(
        new RepulsorDecisionEntry(
            "FieldPlanner",
            plannerDecision,
            plannerReason,
            formatPose(diagnostics.activeGoal()),
            formatPose(diagnostics.requestedGoal()),
            diagnostics.errorMeters(),
            0.0,
            Map.of("offloaded", Boolean.toString(diagnostics.offloaded()))));

    return new RepulsorDecisionTrace(entries, "", "");
  }

  private void recordDecisionTraceTelemetry(RepulsorDecisionTrace trace) {
    RepulsorDecisionTrace safeTrace = trace == null ? RepulsorDecisionTrace.empty() : trace;
    Logger.recordOutput("Repulsor/DecisionTrace/Count", safeTrace.entries().size());
    Logger.recordOutput("Repulsor/DecisionTrace/Summary", safeTrace.summary());
  }

  private String formatPose(Pose2d pose) {
    if (pose == null) return "";
    return String.format(
        "%.3f,%.3f,%.3f", pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  /**
   * Returns the is point in polygon value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param polygon value used by this operation.
   * @return value produced by this operation.
   */
  public static boolean isPointInPolygon(Translation2d point, Translation2d[] polygon) {
    return FieldPlannerGeometry.isPointInPolygon(point, polygon);
  }

  /**
   * Returns the dot value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  public static double dot(Translation2d a, Translation2d b) {
    return FieldPlannerGeometry.dot(a, b);
  }

  /**
   * Returns the distance from point to segment value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  public static double distanceFromPointToSegment(
      Translation2d p, Translation2d a, Translation2d b) {
    return FieldPlannerGeometry.distanceFromPointToSegment(p, a, b);
  }

  private RepulsorSample calculateOffloaded(
      Pose2d pose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      CategorySpec cat,
      boolean suppressFallback,
      double shooterReleaseHeightMeters) {
    FieldPlannerCalculateResultDTO remote =
        FieldPlannerOffloadEntrypoints_Offloaded.calculate_offload(
            pose,
            goalManager.getRequestedGoalPose(),
            goalManager.getGoalPose(),
            dynamicObstacles,
            robot_x,
            robot_y,
            cat == null ? CategorySpec.kScore.name() : cat.name(),
            preferredAllianceForFallback().name(),
            suppressFallback,
            shooterReleaseHeightMeters);
    if (remote == null) {
      throw new IllegalStateException("Null field planner offload result");
    }
    lastOffloadedCalculateResult = remote;

    setActiveGoal(
        new Pose2d(
            remote.getActiveGoalX(),
            remote.getActiveGoalY(),
            Rotation2d.fromRadians(remote.getActiveGoalThetaRadians())));
    if (remote.isHasErrMeters()) {
      currentErr = Optional.of(Meters.of(remote.getErrMeters()));
    } else {
      currentErr = Optional.empty();
    }

    return new RepulsorSample(
        new Translation2d(remote.getGoalX(), remote.getGoalY()),
        remote.getVxMetersPerSecond(),
        remote.getVyMetersPerSecond(),
        Radians.of(remote.getOmegaRadians()));
  }

  private Alliance preferredAllianceForFallback() {
    if (fallbackAllianceOverride != null) return fallbackAllianceOverride;
    if (isOffloadWorkerThread()) {
      Alliance supplied = OFFLOAD_FALLBACK_ALLIANCE.get();
      if (supplied != null) {
        return supplied;
      }
      String forced = System.getProperty("repulsor.offload.fieldplanner.fallbackAlliance", "blue");
      return "red".equalsIgnoreCase(forced) ? Alliance.kRed : Alliance.kBlue;
    }
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
        ? Alliance.kBlue
        : Alliance.kRed;
  }

  private static boolean isOffloadWorkerThread() {
    return OffloadExecutionContext.isWorkerOrLegacyThread();
  }

  /**
   * Updates set offload fallback alliance state or telemetry as part of the Repulsor runtime loop.
   * This may mutate local state, NetworkTables output, planner caches, or command-side runtime
   * state depending on the owning type.
   *
   * @param alliance value used by this operation.
   */
  public static void setOffloadFallbackAlliance(Alliance alliance) {
    if (alliance == null) {
      OFFLOAD_FALLBACK_ALLIANCE.remove();
    } else {
      OFFLOAD_FALLBACK_ALLIANCE.set(alliance);
    }
  }

  /**
   * Updates clear offload fallback alliance state or telemetry as part of the Repulsor runtime
   * loop. This may mutate local state, NetworkTables output, planner caches, or command-side
   * runtime state depending on the owning type.
   */
  public static void clearOffloadFallbackAlliance() {
    OFFLOAD_FALLBACK_ALLIANCE.remove();
  }

  private static RepulsorDriverStation safeDriverStation() {
    try {
      return RepulsorDriverStation.getInstance();
    } catch (Throwable ex) {
      RepulsorDiagnostics.warnThrottled(
          "FieldPlanner/driverStationUnavailable",
          "Driver station unavailable while resolving planner state: " + ex.getMessage(),
          5.0);
      return null;
    }
  }

  private static Force removeBackwardObstacleForceWhenClear(
      Force obstacleForce, Translation2d current, Translation2d target, boolean clearPath) {
    if (!clearPath || obstacleForce.getNorm() < 1e-9) return obstacleForce;

    Translation2d toTarget = target.minus(current);
    double distance = toTarget.getNorm();
    if (distance < 1e-9) return obstacleForce;

    double ux = toTarget.getX() / distance;
    double uy = toTarget.getY() / distance;
    double along = obstacleForce.getX() * ux + obstacleForce.getY() * uy;
    if (along >= 0.0) return obstacleForce;

    return new Force(obstacleForce.getX() - along * ux, obstacleForce.getY() - along * uy);
  }

  private static boolean isClearPath(
      String topicRoot,
      Translation2d start,
      Translation2d goal,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      boolean publishSamples) {
    if (!OFFLOAD_PATHING_ENABLED || isOffloadWorkerThread()) {
      return ExtraPathing.isClearPath(
          topicRoot, start, goal, obstacles, robotLengthMeters, robotWidthMeters, publishSamples);
    }
    return FieldPlannerPathingOffloadEntrypoints_Offloaded.isClearPath_offload(
        topicRoot, start, goal, obstacles, robotLengthMeters, robotWidthMeters, publishSamples);
  }

  private static boolean robotIntersects(
      Translation2d center,
      double robotLengthMeters,
      double robotWidthMeters,
      List<? extends Obstacle> obstacles) {
    if (!OFFLOAD_PATHING_ENABLED || isOffloadWorkerThread()) {
      return ExtraPathing.robotIntersects(center, robotLengthMeters, robotWidthMeters, obstacles);
    }
    return FieldPlannerPathingOffloadEntrypoints_Offloaded.robotIntersects_offload(
        center, robotLengthMeters, robotWidthMeters, obstacles);
  }

  /**
   * Updates poll chosen setpoint state or telemetry as part of the Repulsor runtime loop.
   *
   * @return optional repulsor setpoint produced by this operation.
   */
  public Optional<RepulsorSetpoint> pollChosenSetpoint() {
    var out = lastChosenSetpoint;
    lastChosenSetpoint = Optional.empty();
    return out;
  }
}
