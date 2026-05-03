package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

/** Tunable constants for the collect-objective loop. */
public record CollectPlannerTuning(
    double groupCellMeters,
    double nearbyRadiusMeters,
    double liveObservationMaxAgeSeconds,
    double predictorObservationMaxAgeSeconds,
    double stickyNoProgressSeconds,
    double switchCooldownSeconds,
    double collectCellMeters) {

  public static CollectPlannerTuning defaults() {
    return new CollectPlannerTuning(0.40, 2.2, 0.30, 0.25, 0.40, 0.70, 0.14);
  }

  public CollectPlannerTuning {
    groupCellMeters = nonNegative(groupCellMeters);
    nearbyRadiusMeters = nonNegative(nearbyRadiusMeters);
    liveObservationMaxAgeSeconds = nonNegative(liveObservationMaxAgeSeconds);
    predictorObservationMaxAgeSeconds = nonNegative(predictorObservationMaxAgeSeconds);
    stickyNoProgressSeconds = nonNegative(stickyNoProgressSeconds);
    switchCooldownSeconds = nonNegative(switchCooldownSeconds);
    collectCellMeters = nonNegative(collectCellMeters);
  }

  private static double nonNegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
