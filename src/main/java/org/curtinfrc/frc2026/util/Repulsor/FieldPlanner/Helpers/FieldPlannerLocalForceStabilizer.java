package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Force;

/** Maintains deterministic local force-field state to damp repeated oscillatory samples. */
public final class FieldPlannerLocalForceStabilizer {
  private static final double SAME_TARGET_METERS = 0.35;
  private static final double LOW_PROGRESS_METERS = 0.015;
  private static final double DIRECTION_FLIP_DOT = -0.35;
  private static final double CURRENT_FORCE_BLEND = 0.65;
  private static final double PREVIOUS_FORCE_BLEND = 1.0 - CURRENT_FORCE_BLEND;
  private static final double EPS = 1e-9;

  private FieldPlannerLocalForceStabilitySnapshot snapshot =
      FieldPlannerLocalForceStabilitySnapshot.empty();

  public FieldPlannerLocalForceStabilitySnapshot snapshot() {
    return snapshot;
  }

  public void reset() {
    snapshot = FieldPlannerLocalForceStabilitySnapshot.empty();
  }

  public Result stabilize(
      Translation2d sample, Translation2d target, Force rawForce, boolean stabilizationEnabled) {
    Force safeRaw = rawForce == null ? Force.kZero : rawForce;
    if (sample == null || target == null) {
      reset();
      return new Result(safeRaw, snapshot);
    }

    double errorMeters = sample.getDistance(target);
    boolean sameTarget =
        snapshot.previousTarget() != null
            && snapshot.previousTarget().getDistance(target) <= SAME_TARGET_METERS;
    double progressMeters =
        sameTarget && Double.isFinite(snapshot.previousErrorMeters())
            ? snapshot.previousErrorMeters() - errorMeters
            : 0.0;

    boolean lowProgress = sameTarget && progressMeters < LOW_PROGRESS_METERS;
    boolean directionFlip =
        sameTarget && directionDot(snapshot.previousOutputForce(), safeRaw) < DIRECTION_FLIP_DOT;

    int lowProgressSamples = lowProgress ? snapshot.lowProgressSamples() + 1 : 0;
    int directionFlipSamples = directionFlip ? snapshot.directionFlipSamples() + 1 : 0;
    boolean shouldBlend =
        stabilizationEnabled
            && safeRaw.getNorm() > EPS
            && snapshot.previousOutputForce().getNorm() > EPS
            && (directionFlipSamples > 0 || lowProgressSamples >= 2);
    Force output = shouldBlend ? blend(safeRaw, snapshot.previousOutputForce()) : safeRaw;
    boolean oscillationSuspected = directionFlipSamples >= 2 || lowProgressSamples >= 3;

    snapshot =
        new FieldPlannerLocalForceStabilitySnapshot(
            sample,
            target,
            safeRaw,
            output,
            errorMeters,
            progressMeters,
            lowProgressSamples,
            directionFlipSamples,
            shouldBlend,
            oscillationSuspected);
    return new Result(output, snapshot);
  }

  private static Force blend(Force current, Force previous) {
    Force blended = current.times(CURRENT_FORCE_BLEND).plus(previous.times(PREVIOUS_FORCE_BLEND));
    if (blended.getNorm() <= EPS) return current;
    return blended;
  }

  private static double directionDot(Force a, Force b) {
    if (a == null || b == null || a.getNorm() <= EPS || b.getNorm() <= EPS) return 1.0;
    return (a.getX() * b.getX() + a.getY() * b.getY()) / (a.getNorm() * b.getNorm());
  }

  public record Result(Force force, FieldPlannerLocalForceStabilitySnapshot snapshot) {
    public Result {
      if (force == null) force = Force.kZero;
      if (snapshot == null) snapshot = FieldPlannerLocalForceStabilitySnapshot.empty();
    }
  }
}
