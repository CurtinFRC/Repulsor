package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointStatus;

/** Unified diagnostic snapshot for a Repulsor planning cycle. */
public record RepulsorDiagnosticsSnapshot(
    Pose2d requestedGoal,
    Pose2d activeGoal,
    FieldPlannerWaypointStatus waypointStatus,
    CoarseGlobalPlannerStats globalFallbackStats,
    boolean globalFallbackActive,
    Optional<Pose2d> globalFallbackWaypoint,
    boolean reactiveBypassActive,
    boolean reactiveBypassPinned,
    boolean forceThroughActive,
    boolean pathBlocked,
    boolean robotIntersecting,
    boolean stuckAbort,
    boolean offloaded,
    double errorMeters) {
  public RepulsorDiagnosticsSnapshot {
    if (requestedGoal == null) requestedGoal = Pose2d.kZero;
    if (activeGoal == null) activeGoal = requestedGoal;
    if (globalFallbackStats == null) globalFallbackStats = CoarseGlobalPlannerStats.empty();
    globalFallbackWaypoint =
        globalFallbackWaypoint == null ? Optional.empty() : globalFallbackWaypoint;
    if (!Double.isFinite(errorMeters)) errorMeters = Double.NaN;
  }

  public static RepulsorDiagnosticsSnapshot empty() {
    return new RepulsorDiagnosticsSnapshot(
        Pose2d.kZero,
        Pose2d.kZero,
        null,
        CoarseGlobalPlannerStats.empty(),
        false,
        Optional.empty(),
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        Double.NaN);
  }
}
