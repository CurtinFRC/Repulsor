package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

/** Additive weighted cost components for a coarse global route. */
public record CoarseRouteCostBreakdown(
    double total,
    double distanceCost,
    double obstacleClearanceCost,
    double wallClearanceCost,
    double turnCost) {
  public CoarseRouteCostBreakdown {
    total = finiteNonNegative(total);
    distanceCost = finiteNonNegative(distanceCost);
    obstacleClearanceCost = finiteNonNegative(obstacleClearanceCost);
    wallClearanceCost = finiteNonNegative(wallClearanceCost);
    turnCost = finiteNonNegative(turnCost);
  }

  public static CoarseRouteCostBreakdown empty() {
    return new CoarseRouteCostBreakdown(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  public CoarseRouteCostBreakdown plus(CoarseRouteCostBreakdown other) {
    if (other == null) return this;
    return new CoarseRouteCostBreakdown(
        total + other.total,
        distanceCost + other.distanceCost,
        obstacleClearanceCost + other.obstacleClearanceCost,
        wallClearanceCost + other.wallClearanceCost,
        turnCost + other.turnCost);
  }

  private static double finiteNonNegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
