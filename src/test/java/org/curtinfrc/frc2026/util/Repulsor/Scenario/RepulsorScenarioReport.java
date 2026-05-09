package org.curtinfrc.frc2026.util.Repulsor.Scenario;

import java.util.Locale;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorPlanningResult;

/** Human-readable report for scenario results. Intended for debugging, not timing assertions. */
public final class RepulsorScenarioReport {
  private RepulsorScenarioReport() {}

  public static String summarize(RepulsorPlannerScenarioResult result) {
    if (result == null) return "scenario: <null>";
    RepulsorPlanningResult planningResult = result.lastPlanningResult();
    String traceSummary =
        planningResult == null || planningResult.decisionTrace() == null
            ? "empty"
            : planningResult.decisionTrace().summary();
    return String.join(
        System.lineSeparator(),
        "scenario: " + result.scenario().name(),
        "reachedGoal: " + result.reachedGoal(),
        "finalPose: " + pose(result),
        "finalErrorMeters: " + format(result.finalDistanceMeters()),
        "progressMeters: " + format(result.progressMeters()),
        "fallbackCycles: " + result.globalFallbackCycles(),
        "waypointStageCycles: " + result.waypointStageCycles(),
        "waypointBypassCycles: " + result.waypointBypassCycles(),
        "reactiveBypassCycles: " + result.reactiveBypassCycles(),
        "pathBlockedCycles: " + result.pathBlockedCycles(),
        "robotIntersectingCycles: " + result.robotIntersectingCycles(),
        "stuckAbortCycles: " + result.stuckAbortCycles(),
        "maxCommandSpeedMetersPerSecond: " + format(result.maxCommandSpeedMetersPerSecond()),
        "decisionTrace: " + traceSummary);
  }

  private static String pose(RepulsorPlannerScenarioResult result) {
    return String.format(
        Locale.ROOT,
        "(%.3f, %.3f, %.1fdeg)",
        result.finalPose().getX(),
        result.finalPose().getY(),
        result.finalPose().getRotation().getDegrees());
  }

  private static String format(double value) {
    return String.format(Locale.ROOT, "%.3f", value);
  }
}
