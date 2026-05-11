package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

/**
 * Soft dynamic-obstacle prediction envelope used by route costs and local forces.
 *
 * <p>Predicted envelopes intentionally do not hard-block robot rectangles. Current-cycle obstacle
 * geometry remains responsible for collision rejection, while prediction only biases routing away
 * from likely near-future conflicts.
 */
public final class PredictedDynamicObstacleEnvelope extends Obstacle {
  private static final double EPS = 1e-9;

  public final Translation2d center;
  public final double radiusX;
  public final double radiusY;
  public final double horizonWeight;
  public final int horizonStep;

  public PredictedDynamicObstacleEnvelope(
      Translation2d center,
      double radiusX,
      double radiusY,
      double strength,
      double horizonWeight,
      int horizonStep) {
    super(strength, true);
    this.center = center == null ? new Translation2d() : center;
    this.radiusX = Math.max(EPS, radiusX);
    this.radiusY = Math.max(EPS, radiusY);
    this.horizonWeight = clamp01(horizonWeight);
    this.horizonStep = Math.max(1, horizonStep);
  }

  @Override
  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    if (position == null || horizonWeight <= 0.0) return Force.kZero;
    Translation2d delta = position.minus(center);
    double distance = delta.getNorm();
    double scaledRadius = effectiveRadiusToward(delta);
    double radial = distance - scaledRadius;
    if (distance <= EPS) return Force.kZero;
    double magnitude = distToForceMag(radial) * horizonWeight;
    if (!Double.isFinite(magnitude) || Math.abs(magnitude) < 1e-12) return Force.kZero;
    return new Force(magnitude, delta.getAngle());
  }

  @Override
  public Force sampleForceAtPosition(Translation2d position, Translation2d target) {
    return getForceAtPosition(position, target);
  }

  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    return false;
  }

  public double clearanceMeters(
      Translation2d point, double robotHalfLength, double robotHalfWidth) {
    if (point == null) return Double.POSITIVE_INFINITY;
    Translation2d delta = point.minus(center);
    double distance = delta.getNorm();
    double inflatedRadius =
        effectiveRadiusToward(delta) + Math.max(robotHalfLength, robotHalfWidth);
    return distance - inflatedRadius;
  }

  private double effectiveRadiusToward(Translation2d delta) {
    double norm = delta.getNorm();
    if (norm <= EPS) return Math.max(radiusX, radiusY);
    Rotation2d angle = delta.getAngle();
    double cos = angle.getCos();
    double sin = angle.getSin();
    double denom = Math.sqrt((radiusY * cos) * (radiusY * cos) + (radiusX * sin) * (radiusX * sin));
    if (denom <= EPS) return Math.max(radiusX, radiusY);
    return (radiusX * radiusY) / denom;
  }

  private static double clamp01(double value) {
    if (!Double.isFinite(value)) return 0.0;
    return Math.max(0.0, Math.min(1.0, value));
  }
}
