package org.curtinfrc.frc2026.util.Repulsor.Setpoints.Specific;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;
import org.junit.jupiter.api.Test;

class HubShotCacheKeyTest {
  @Test
  void equalContentObstacleListsProduceEqualHashes() {
    Obstacle a = new RadialObstacle(new Translation2d(5.0, 4.0), 0.30, 1.5);
    Obstacle b = new RadialObstacle(new Translation2d(5.0, 4.0), 0.30, 1.5);

    assertEquals(
        _Rebuilt2026.obstaclesStableHash(List.of(a)), _Rebuilt2026.obstaclesStableHash(List.of(b)));
    assertEquals(
        _Rebuilt2026.obstaclesStableHash(List.of(a)),
        _Rebuilt2026.obstaclesStableHash(
            List.of(new RadialObstacle(new Translation2d(5.0, 4.0), 0.30, 1.5))));
    assertEquals(0, _Rebuilt2026.obstaclesStableHash(List.of()));
    assertEquals(0, _Rebuilt2026.obstaclesStableHash(null));
  }

  @Test
  void hashIsOrderIndependent() {
    Obstacle a = new RadialObstacle(new Translation2d(5.0, 4.0), 0.30, 1.5);
    Obstacle b = new RadialObstacle(new Translation2d(9.5, 3.0), 0.25, 1.2);

    assertEquals(
        _Rebuilt2026.obstaclesStableHash(List.of(a, b)),
        _Rebuilt2026.obstaclesStableHash(List.of(b, a)));
  }

  @Test
  void movedObstacleChangesHash() {
    double baseline =
        _Rebuilt2026.obstaclesStableHash(
            List.of(new RadialObstacle(new Translation2d(5.0, 4.0), 0.30, 1.5)));

    assertNotEquals(
        baseline,
        _Rebuilt2026.obstaclesStableHash(
            List.of(new RadialObstacle(new Translation2d(5.05, 4.0), 0.30, 1.5))));
    assertNotEquals(
        baseline,
        _Rebuilt2026.obstaclesStableHash(
            List.of(new RadialObstacle(new Translation2d(5.0, 4.05), 0.30, 1.5))));
    assertNotEquals(
        baseline,
        _Rebuilt2026.obstaclesStableHash(
            List.of(new RadialObstacle(new Translation2d(5.0, 4.0), 0.35, 1.5))));
    assertNotEquals(
        baseline,
        _Rebuilt2026.obstaclesStableHash(
            List.of(new RadialObstacle(new Translation2d(5.0, 4.0), 0.30, 1.7))));
  }

  @Test
  void duplicateObstaclesDoNotCancelOutOfTheFold() {
    Obstacle a = new RadialObstacle(new Translation2d(5.0, 4.0), 0.30, 1.5);

    assertNotEquals(
        _Rebuilt2026.obstaclesStableHash(List.of(a)),
        _Rebuilt2026.obstaclesStableHash(List.of(a, a)));
  }

  private static final class RadialObstacle extends Obstacle {
    private final Translation2d center;
    private final double radius;

    RadialObstacle(Translation2d center, double radius, double strength) {
      super(strength, true);
      this.center = center;
      this.radius = radius;
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      double distance = center.getDistance(position);
      if (distance > 3.0) return Force.kZero;
      double radial = distance - radius;
      double mag = strength / (0.00001 + radial * radial);
      Translation2d delta = position.minus(center);
      if (delta.getNorm() < 1e-9 || Math.abs(mag) < 1e-12) return Force.kZero;
      return new Force(mag, delta.getAngle());
    }
  }
}
