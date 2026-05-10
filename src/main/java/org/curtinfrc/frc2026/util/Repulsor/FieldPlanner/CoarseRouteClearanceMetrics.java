package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

/** Minimum and average clearance diagnostics for the raw coarse global route. */
public record CoarseRouteClearanceMetrics(
    double minRouteClearanceMeters, double averageRouteClearanceMeters) {
  public CoarseRouteClearanceMetrics {
    minRouteClearanceMeters = finiteNonNegative(minRouteClearanceMeters);
    averageRouteClearanceMeters = finiteNonNegative(averageRouteClearanceMeters);
  }

  public static CoarseRouteClearanceMetrics empty() {
    return new CoarseRouteClearanceMetrics(0.0, 0.0);
  }

  private static double finiteNonNegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
