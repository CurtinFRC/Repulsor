package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Force;

/** Deterministic local force-field stability diagnostics from the most recent planner sample. */
public record FieldPlannerLocalForceStabilitySnapshot(
    Translation2d previousSample,
    Translation2d previousTarget,
    Force previousRawForce,
    Force previousOutputForce,
    double previousErrorMeters,
    double progressMeters,
    int lowProgressSamples,
    int directionFlipSamples,
    boolean blended,
    boolean oscillationSuspected) {
  public FieldPlannerLocalForceStabilitySnapshot {
    previousErrorMeters = finiteOrNaN(previousErrorMeters);
    progressMeters = finiteOrZero(progressMeters);
    lowProgressSamples = Math.max(0, lowProgressSamples);
    directionFlipSamples = Math.max(0, directionFlipSamples);
    if (previousRawForce == null) previousRawForce = Force.kZero;
    if (previousOutputForce == null) previousOutputForce = Force.kZero;
  }

  public static FieldPlannerLocalForceStabilitySnapshot empty() {
    return new FieldPlannerLocalForceStabilitySnapshot(
        null, null, Force.kZero, Force.kZero, Double.NaN, 0.0, 0, 0, false, false);
  }

  private static double finiteOrNaN(double value) {
    return Double.isFinite(value) ? value : Double.NaN;
  }

  private static double finiteOrZero(double value) {
    return Double.isFinite(value) ? value : 0.0;
  }
}
