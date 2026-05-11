package org.curtinfrc.frc2026.util.Repulsor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PredictedDynamicObstacleEnvelope;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision;
import org.junit.jupiter.api.Test;

class VisionPlannerTest {
  @Test
  void tickAddsDeterministicPredictedEnvelopeFromPreviousDetection() {
    var type = new RepulsorVision.ObstacleType(0.4, 0.4, RepulsorVision.Kind.kRobotRed);
    var vision =
        new ScriptedVision(
            List.of(
                new RepulsorVision.Obstacle(2.0, 1.0, type),
                new RepulsorVision.Obstacle(2.4, 1.3, type)));
    VisionPlanner planner = new VisionPlanner().withVision(vision);

    planner.tick();
    List<org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle> first = planner.getObstacles();
    planner.tick();
    List<org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle> second = planner.getObstacles();

    assertEquals(1, first.size(), "first detection has no prior sample to extrapolate from");
    assertEquals(2, second.size(), "second detection should include current plus prediction");
    PredictedDynamicObstacleEnvelope prediction =
        assertInstanceOf(PredictedDynamicObstacleEnvelope.class, second.get(1));
    assertEquals(2.8, prediction.center.getX(), 1e-9);
    assertEquals(1.6, prediction.center.getY(), 1e-9);
    assertTrue(prediction.radiusX > 0.2, "uncertainty should inflate predicted radius");
    assertTrue(prediction.horizonWeight > 0.0);
  }

  @Test
  void unchangedDetectionDoesNotDuplicateStaticObstacleAsPrediction() {
    var type = new RepulsorVision.ObstacleType(0.4, 0.4, RepulsorVision.Kind.kRobotBlue);
    var vision =
        new ScriptedVision(
            List.of(
                new RepulsorVision.Obstacle(2.0, 1.0, type),
                new RepulsorVision.Obstacle(2.0, 1.0, type)));
    VisionPlanner planner = new VisionPlanner().withVision(vision);

    planner.tick();
    planner.tick();

    assertEquals(1, planner.getObstacles().size());
  }

  private static final class ScriptedVision implements RepulsorVision {
    private final List<RepulsorVision.Obstacle> sequence;
    private int index = -1;

    ScriptedVision(List<RepulsorVision.Obstacle> sequence) {
      this.sequence = List.copyOf(sequence);
    }

    @Override
    public void tick() {
      index = Math.min(index + 1, sequence.size() - 1);
    }

    @Override
    public RepulsorVision.Obstacle[] getObstacles() {
      if (index < 0 || sequence.isEmpty()) return new RepulsorVision.Obstacle[0];
      return new RepulsorVision.Obstacle[] {sequence.get(index)};
    }
  }
}
