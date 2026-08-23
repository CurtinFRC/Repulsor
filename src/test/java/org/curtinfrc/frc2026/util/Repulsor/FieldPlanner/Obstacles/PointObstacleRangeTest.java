package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class PointObstacleRangeTest {
  private static final Translation2d TARGET = new Translation2d(20.0, 20.0);

  @Test
  void defaultActivationRangeIsFourMeters() {
    assertEquals(4.0, PointObstacle.DEFAULT_ACTIVATION_RANGE_METERS, 0.0);

    PointObstacle obstacle = new PointObstacle(new Translation2d(0.0, 0.0), 1.0, true);
    assertEquals(4.0, obstacle.activationRangeMeters, 0.0);
    assertEquals(
        0.0,
        obstacle.getForceAtPosition(new Translation2d(4.0000001, 0.0), TARGET).getNorm(),
        1e-12);
    assertTrue(obstacle.getForceAtPosition(new Translation2d(3.9, 0.0), TARGET).getNorm() > 0.0);
    assertEquals(
        0.0, obstacle.getForceAtPosition(new Translation2d(-4.5, 0.0), TARGET).getNorm(), 1e-12);
  }

  @Test
  void customActivationRangeHonored() {
    PointObstacle obstacle = new PointObstacle(new Translation2d(0.0, 0.0), 1.0, true, 2.0);
    assertEquals(2.0, obstacle.activationRangeMeters, 0.0);

    assertEquals(
        0.0, obstacle.getForceAtPosition(new Translation2d(2.1, 0.0), TARGET).getNorm(), 1e-12);
    assertEquals(
        0.0, obstacle.getForceAtPosition(new Translation2d(0.0, -2.5), TARGET).getNorm(), 1e-12);
    assertTrue(obstacle.getForceAtPosition(new Translation2d(1.9, 0.0), TARGET).getNorm() > 0.0);
    assertTrue(obstacle.getForceAtPosition(new Translation2d(0.5, 0.0), TARGET).getNorm() > 0.0);
  }
}
