package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

/** Tunable constants for the collect-objective loop. */
public record CollectPlannerTuning(
    double groupCellMeters,
    double nearbyRadiusMeters,
    double liveObservationMaxAgeSeconds,
    double predictorObservationMaxAgeSeconds,
    double stickyNoProgressSeconds,
    double switchCooldownSeconds,
    double collectCellMeters,
    CollectObjectiveSelectionConfig selection,
    ForbiddenBandTuning forbiddenBands) {

  /**
   * Creates planner tuning with the forbidden-band carrier left at its profile defaults.
   *
   * @param groupCellMeters clustering cell size in meters
   * @param nearbyRadiusMeters nearby-resource aggregation radius in meters
   * @param liveObservationMaxAgeSeconds maximum live observation age in seconds
   * @param predictorObservationMaxAgeSeconds maximum predictor observation age in seconds
   * @param stickyNoProgressSeconds sticky no-progress timeout in seconds
   * @param switchCooldownSeconds minimum seconds between objective switches
   * @param collectCellMeters collect grid cell size in meters
   * @param selection objective selection configuration
   */
  public CollectPlannerTuning(
      double groupCellMeters,
      double nearbyRadiusMeters,
      double liveObservationMaxAgeSeconds,
      double predictorObservationMaxAgeSeconds,
      double stickyNoProgressSeconds,
      double switchCooldownSeconds,
      double collectCellMeters,
      CollectObjectiveSelectionConfig selection) {
    this(
        groupCellMeters,
        nearbyRadiusMeters,
        liveObservationMaxAgeSeconds,
        predictorObservationMaxAgeSeconds,
        stickyNoProgressSeconds,
        switchCooldownSeconds,
        collectCellMeters,
        selection,
        null);
  }

  public static CollectPlannerTuning defaults() {
    return new CollectPlannerTuning(
        0.40, 2.2, 0.30, 0.25, 0.40, 0.70, 0.14, CollectObjectiveSelectionConfig.defaults());
  }

  public CollectPlannerTuning {
    groupCellMeters = nonNegative(groupCellMeters);
    nearbyRadiusMeters = nonNegative(nearbyRadiusMeters);
    liveObservationMaxAgeSeconds = nonNegative(liveObservationMaxAgeSeconds);
    predictorObservationMaxAgeSeconds = nonNegative(predictorObservationMaxAgeSeconds);
    stickyNoProgressSeconds = nonNegative(stickyNoProgressSeconds);
    switchCooldownSeconds = nonNegative(switchCooldownSeconds);
    collectCellMeters = nonNegative(collectCellMeters);
    selection = selection == null ? CollectObjectiveSelectionConfig.defaults() : selection;
    forbiddenBands = forbiddenBands == null ? ForbiddenBandTuning.defaults() : forbiddenBands;
  }

  private static double nonNegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
