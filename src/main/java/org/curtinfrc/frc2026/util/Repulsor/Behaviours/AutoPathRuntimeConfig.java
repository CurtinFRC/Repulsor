package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

/** Tuning for AutoPath objective selection, progress detection, and score lock behavior. */
public record AutoPathRuntimeConfig(
    double episodeCooldownSeconds,
    double pinnedFailSeconds,
    double stuckFailSeconds,
    double progressEpsilonMeters,
    double pinnedProgressMinMeters,
    double stuckDistanceMinMeters,
    double successNearDistanceMeters,
    int collectGoalUnits,
    double shootLockEnterMeters,
    double shootLockExitMeters,
    double shootLockMinRotationDegrees,
    double shootReadyPositionToleranceMeters,
    double shootReadyRotationToleranceDegrees,
    double collectHoldGoalNearMeters,
    double collectFarResourceMinDistanceMeters) {
  public static AutoPathRuntimeConfig defaults() {
    return new AutoPathRuntimeConfig(
        1.0, 2.0, 3.0, 0.03, 0.15, 0.5, 0.40, 3, 3.0, 3.6, 8.0, 0.28, 10.0, 0.25, 1.10);
  }

  public AutoPathRuntimeConfig {
    episodeCooldownSeconds = finiteNonNegative(episodeCooldownSeconds, 1.0);
    pinnedFailSeconds = finiteNonNegative(pinnedFailSeconds, 2.0);
    stuckFailSeconds = finiteNonNegative(stuckFailSeconds, 3.0);
    progressEpsilonMeters = finiteNonNegative(progressEpsilonMeters, 0.03);
    pinnedProgressMinMeters = finiteNonNegative(pinnedProgressMinMeters, 0.15);
    stuckDistanceMinMeters = finiteNonNegative(stuckDistanceMinMeters, 0.5);
    successNearDistanceMeters = finiteNonNegative(successNearDistanceMeters, 0.40);
    collectGoalUnits = Math.max(1, collectGoalUnits);
    shootLockEnterMeters = finiteNonNegative(shootLockEnterMeters, 3.0);
    shootLockExitMeters =
        Math.max(shootLockEnterMeters, finiteNonNegative(shootLockExitMeters, 3.6));
    shootLockMinRotationDegrees = finiteNonNegative(shootLockMinRotationDegrees, 8.0);
    shootReadyPositionToleranceMeters = finiteNonNegative(shootReadyPositionToleranceMeters, 0.28);
    shootReadyRotationToleranceDegrees =
        finiteNonNegative(shootReadyRotationToleranceDegrees, 10.0);
    collectHoldGoalNearMeters = finiteNonNegative(collectHoldGoalNearMeters, 0.25);
    collectFarResourceMinDistanceMeters =
        finiteNonNegative(collectFarResourceMinDistanceMeters, 1.10);
  }

  public long episodeCooldownNanos() {
    return secondsToNanos(episodeCooldownSeconds);
  }

  public long pinnedFailNanos() {
    return secondsToNanos(pinnedFailSeconds);
  }

  public long stuckFailNanos() {
    return secondsToNanos(stuckFailSeconds);
  }

  private static long secondsToNanos(double seconds) {
    return (long) (Math.max(0.0, seconds) * 1_000_000_000L);
  }

  private static double finiteNonNegative(double value, double fallback) {
    return Double.isFinite(value) && value >= 0.0 ? value : fallback;
  }
}
