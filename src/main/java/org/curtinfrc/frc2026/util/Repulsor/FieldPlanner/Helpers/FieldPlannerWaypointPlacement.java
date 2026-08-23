package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

/**
 * Tunable placement distances for goal-manager waypoint staging. Defaults mirror the historical
 * hard-coded goal-manager literals so default configs produce identical decisions.
 */
public record FieldPlannerWaypointPlacement(
    double entryReachEnterMeters,
    double exitReachEnterMeters,
    double reachExitMeters,
    double entryPassWithinMeters,
    double entryPassedProjectionMeters,
    double exitPassedProjectionMeters,
    double gateClearExitMeters,
    double passedGateHysteresisMeters,
    double goalSideProjectionMeters,
    double centerReturnExitAdvanceScale) {
  public static FieldPlannerWaypointPlacement defaults() {
    return new FieldPlannerWaypointPlacement(
        0.50, 0.45, 0.55, 0.85, 0.06, 0.10, 0.40, 0.35, 0.05, 0.45);
  }

  public FieldPlannerWaypointPlacement {
    entryReachEnterMeters = nonNegative(entryReachEnterMeters, 0.50);
    exitReachEnterMeters = nonNegative(exitReachEnterMeters, 0.45);
    reachExitMeters = nonNegative(reachExitMeters, 0.55);
    entryPassWithinMeters = nonNegative(entryPassWithinMeters, 0.85);
    entryPassedProjectionMeters = nonNegative(entryPassedProjectionMeters, 0.06);
    exitPassedProjectionMeters = nonNegative(exitPassedProjectionMeters, 0.10);
    gateClearExitMeters = nonNegative(gateClearExitMeters, 0.40);
    passedGateHysteresisMeters = nonNegative(passedGateHysteresisMeters, 0.35);
    goalSideProjectionMeters = nonNegative(goalSideProjectionMeters, 0.05);
    centerReturnExitAdvanceScale = nonNegative(centerReturnExitAdvanceScale, 0.45);
  }

  private static double nonNegative(double value, double fallback) {
    return Double.isFinite(value) && value >= 0.0 ? value : fallback;
  }
}
