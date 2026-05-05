package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

/** Tunable weights for comparing strategy-generated waypoint candidates. */
public record FieldPlannerWaypointScoringConfig(
    double distanceCost,
    double goalAlignmentGain,
    double obstacleClearanceGain,
    double preferenceGain) {
  public static FieldPlannerWaypointScoringConfig defaults() {
    return new FieldPlannerWaypointScoringConfig(1.0, 0.35, 0.20, 1.0);
  }

  public FieldPlannerWaypointScoringConfig {
    distanceCost = finiteNonNegative(distanceCost, 1.0);
    goalAlignmentGain = finiteNonNegative(goalAlignmentGain, 0.35);
    obstacleClearanceGain = finiteNonNegative(obstacleClearanceGain, 0.20);
    preferenceGain = finiteNonNegative(preferenceGain, 1.0);
  }

  private static double finiteNonNegative(double value, double fallback) {
    return Double.isFinite(value) && value >= 0.0 ? value : fallback;
  }
}
