package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PointObstacle;
import org.junit.jupiter.api.Test;

class DragShotFootprintConventionTest {
  private static final Translation2d SHOOTER_POS = new Translation2d(2.0, 1.5);
  private static final Translation2d TARGET = new Translation2d(10.0, 1.5);
  private static final double HALF_LENGTH = 0.40;
  private static final double HALF_WIDTH = 0.30;

  private static PointObstacle probeAt(double x, double y) {
    PointObstacle obstacle = new PointObstacle(new Translation2d(x, y), 1.0, true);
    obstacle.radius = 0.01;
    return obstacle;
  }

  @Test
  void standoffRectangleSpansFullRobotLength() {
    List<Obstacle> probe =
        List.of(probeAt(SHOOTER_POS.getX() - HALF_LENGTH + 0.05, SHOOTER_POS.getY()));

    boolean valid =
        DragShotPlanner.isShooterPoseValid(
            SHOOTER_POS, TARGET, HALF_LENGTH, HALF_WIDTH, probe, false);

    assertFalse(valid, "obstacle inside full-length footprint must reject the pose");
  }

  @Test
  void standoffRectangleSpansFullRobotWidth() {
    List<Obstacle> probe =
        List.of(probeAt(SHOOTER_POS.getX(), SHOOTER_POS.getY() + HALF_WIDTH - 0.05));

    boolean valid =
        DragShotPlanner.isShooterPoseValid(
            SHOOTER_POS, TARGET, HALF_LENGTH, HALF_WIDTH, probe, false);

    assertFalse(valid, "obstacle inside full-width footprint must reject the pose");
  }

  @Test
  void standoffRectangleIgnoresObstaclesBeyondFullFootprint() {
    List<Obstacle> probe =
        List.of(probeAt(SHOOTER_POS.getX() - HALF_LENGTH - 0.05, SHOOTER_POS.getY()));

    boolean valid =
        DragShotPlanner.isShooterPoseValid(
            SHOOTER_POS, TARGET, HALF_LENGTH, HALF_WIDTH, probe, false);

    assertTrue(valid, "obstacle just beyond the full-length footprint must not reject the pose");
  }
}
