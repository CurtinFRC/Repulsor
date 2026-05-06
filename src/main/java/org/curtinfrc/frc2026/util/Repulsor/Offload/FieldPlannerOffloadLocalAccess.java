package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.CoarseGlobalPlannerStats;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorDiagnosticsSnapshot;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;

/**
 * Provides field planner offload local access functionality for the Repulsor offload serialization
 * and native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class FieldPlannerOffloadLocalAccess {
  private static final Object LOCK = new Object();
  private static FieldPlanner planner;

  private FieldPlannerOffloadLocalAccess() {}

  /**
   * Computes the calculate local value for the current Repulsor planning state. Call this from
   * periodic planning or tests when a fresh decision is required; inputs should already be
   * expressed in the coordinate frame expected by the parameter names.
   *
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @param requestedGoalPose value used by this operation.
   * @param activeGoalPose value used by this operation.
   * @param dynamicObstacles obstacle set used for safety checks, costs, or replanning.
   * @param robot_x distance or field-coordinate value in meters.
   * @param robot_y distance or field-coordinate value in meters.
   * @param categoryName value used by this operation.
   * @param preferredAllianceName value used by this operation.
   * @param suppressFallback value used by this operation.
   * @param shooterReleaseHeightMeters distance or field-coordinate value in meters.
   * @return field planner calculate result dto result for calculate local.
   */
  public static FieldPlannerCalculateResultDTO calculateLocal(
      Pose2d pose,
      Pose2d requestedGoalPose,
      Pose2d activeGoalPose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      String categoryName,
      String preferredAllianceName,
      boolean suppressFallback,
      double shooterReleaseHeightMeters) {
    synchronized (LOCK) {
      FieldPlanner localPlanner = planner();
      localPlanner.syncGoalManagerState(
          requestedGoalPose == null ? Pose2d.kZero : requestedGoalPose, activeGoalPose);

      CategorySpec cat = parseCategory(categoryName);
      Alliance preferredAlliance = parseAlliance(preferredAllianceName);
      RepulsorSample sample;
      FieldPlanner.setOffloadFallbackAlliance(preferredAlliance);
      try {
        sample =
            localPlanner.calculate(
                pose == null ? Pose2d.kZero : pose,
                dynamicObstacles == null ? List.of() : dynamicObstacles,
                robot_x,
                robot_y,
                cat,
                suppressFallback,
                shooterReleaseHeightMeters);
      } finally {
        FieldPlanner.clearOffloadFallbackAlliance();
      }

      FieldPlannerCalculateResultDTO out = new FieldPlannerCalculateResultDTO();
      Translation2d goal = sample.goal();
      out.setGoalX(goal.getX());
      out.setGoalY(goal.getY());
      out.setVxMetersPerSecond(sample.vxMetersPerSecond());
      out.setVyMetersPerSecond(sample.vyMetersPerSecond());
      out.setOmegaRadians(sample.omegaRadians());
      out.setHasErrMeters(localPlanner.getErr().isPresent());
      out.setErrMeters(localPlanner.getErr().map(d -> d.in(Meters)).orElse(0.0));

      Pose2d activeGoal = localPlanner.getGoalPose();
      out.setActiveGoalX(activeGoal.getX());
      out.setActiveGoalY(activeGoal.getY());
      out.setActiveGoalThetaRadians(activeGoal.getRotation().getRadians());
      populateDiagnostics(out, localPlanner.lastPlanningResult().diagnostics());
      return out;
    }
  }

  private static void populateDiagnostics(
      FieldPlannerCalculateResultDTO out, RepulsorDiagnosticsSnapshot diagnostics) {
    if (out == null || diagnostics == null) return;
    out.setPathBlocked(diagnostics.pathBlocked());
    out.setGlobalFallbackActive(diagnostics.globalFallbackActive());
    out.setReactiveBypassActive(diagnostics.reactiveBypassActive());
    out.setReactiveBypassPinned(diagnostics.reactiveBypassPinned());
    out.setForceThroughActive(diagnostics.forceThroughActive());
    out.setRobotIntersecting(diagnostics.robotIntersecting());
    out.setStuckAbort(diagnostics.stuckAbort());
    if (diagnostics.waypointStatus() != null) {
      out.setWaypointActiveStage(diagnostics.waypointStatus().activeStage());
      out.setWaypointUsingBypass(diagnostics.waypointStatus().usingBypass());
      out.setWaypointStagedModeTicks(diagnostics.waypointStatus().stagedModeTicks());
    }
    diagnostics
        .globalFallbackWaypoint()
        .ifPresent(
            waypoint -> {
              out.setHasGlobalFallbackWaypoint(true);
              out.setGlobalFallbackWaypointX(waypoint.getX());
              out.setGlobalFallbackWaypointY(waypoint.getY());
              out.setGlobalFallbackWaypointThetaRadians(waypoint.getRotation().getRadians());
            });
    CoarseGlobalPlannerStats stats = diagnostics.globalFallbackStats();
    if (stats != null) {
      out.setGlobalFallbackFound(stats.found());
      out.setGlobalFallbackTimedOut(stats.timedOut());
      out.setGlobalFallbackExhaustedNodeBudget(stats.exhaustedNodeBudget());
      out.setGlobalFallbackExpandedNodes(stats.expandedNodes());
      out.setGlobalFallbackGeneratedNodes(stats.generatedNodes());
      out.setGlobalFallbackPathNodes(stats.pathNodes());
      out.setGlobalFallbackElapsedNanos(stats.elapsedNanos());
    }
  }

  private static FieldPlanner planner() {
    if (planner == null) {
      planner = new FieldPlanner();
    }
    return planner;
  }

  static void resetPlannerForTesting() {
    synchronized (LOCK) {
      planner = null;
    }
  }

  private static CategorySpec parseCategory(String categoryName) {
    if (categoryName == null || categoryName.isBlank()) {
      return CategorySpec.kScore;
    }
    try {
      return CategorySpec.valueOf(categoryName);
    } catch (IllegalArgumentException ignored) {
      return CategorySpec.kScore;
    }
  }

  private static Alliance parseAlliance(String allianceName) {
    if (allianceName == null || allianceName.isBlank()) {
      return Alliance.kBlue;
    }
    try {
      return Alliance.valueOf(allianceName);
    } catch (IllegalArgumentException ignored) {
      return Alliance.kBlue;
    }
  }
}
