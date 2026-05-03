package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

class RectangleObstacleStateSamplingTest {
  @Test
  void sampleForceDoesNotChangeCommitState() throws Exception {
    RectangleObstacle obstacle =
        new RectangleObstacle(
            new Translation2d(2.0, 2.0), 1.0, 1.0, Rotation2d.kZero, 2.0, 2.0, 2.0, true);

    Translation2d pos = new Translation2d(1.3, 2.15);
    Translation2d target = new Translation2d(3.4, 2.15);

    obstacle.sampleForceAtPosition(pos, target);
    obstacle.sampleForceAtPosition(pos, target);

    assertEquals(
        0, commitDir(obstacle), "Telemetry sampling should not latch rectangle flow state");

    obstacle.getForceAtPosition(pos, target);
    assertTrue(commitDir(obstacle) != 0, "Control-loop force evaluation should still be stateful");
  }

  @Test
  void customTuningCanChangeFlowAssistSampleWithoutApiBreakage() {
    Translation2d pos = new Translation2d(1.3, 2.15);
    Translation2d target = new Translation2d(3.4, 2.15);

    RectangleObstacle normal =
        new RectangleObstacle(
            new Translation2d(2.0, 2.0), 1.0, 1.0, Rotation2d.kZero, 2.0, 2.0, 2.0, true);
    RectangleObstacle tuned =
        new RectangleObstacle(
            new Translation2d(2.0, 2.0),
            1.0,
            1.0,
            Rotation2d.kZero,
            2.0,
            2.0,
            2.0,
            true,
            new RectangleObstacleTuning(0.0, 0.18, 0.52, 0.50, 0.65, 0.0, 0.24, 0.30));

    double normalNorm = normal.sampleForceAtPosition(pos, target).getNorm();
    double tunedNorm = tuned.sampleForceAtPosition(pos, target).getNorm();

    assertTrue(Math.abs(normalNorm - tunedNorm) > 1e-9);
  }

  private static int commitDir(RectangleObstacle obstacle) throws Exception {
    Field f = RectangleObstacle.class.getDeclaredField("commitDir");
    f.setAccessible(true);
    return f.getInt(obstacle);
  }
}
