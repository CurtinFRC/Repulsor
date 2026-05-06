package org.curtinfrc.frc2026.util.Repulsor.Scenario;

import edu.wpi.first.math.geometry.Pose2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorPlanningResult;

/** Aggregate quality metrics from running a planner scenario. */
public record RepulsorPlannerScenarioResult(
    RepulsorPlannerScenario scenario,
    Pose2d finalPose,
    RepulsorPlanningResult lastPlanningResult,
    int steps,
    boolean reachedGoal,
    double initialDistanceMeters,
    double finalDistanceMeters,
    double minDistanceMeters,
    int pathBlockedCycles,
    int globalFallbackCycles,
    int waypointStageCycles,
    int waypointBypassCycles,
    int reactiveBypassCycles,
    int robotIntersectingCycles,
    int stuckAbortCycles,
    double maxCommandSpeedMetersPerSecond) {
  public double progressMeters() {
    return initialDistanceMeters - finalDistanceMeters;
  }

  public boolean madeProgress() {
    return progressMeters() > 1e-6;
  }
}
