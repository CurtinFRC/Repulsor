package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadExecutionContext;
import org.junit.jupiter.api.Test;

class FieldPlannerGlobalFallbackIntegrationTest {
  @Test
  void calculateUsesTemporaryGlobalWaypointWithoutMutatingActiveGoal() {
    FieldPlanner planner = new FieldPlanner();
    Pose2d finalGoal = new Pose2d(5.0, 2.0, Rotation2d.kZero);
    planner.setRequestedGoal(finalGoal);

    OffloadExecutionContext.runWorker(
        () ->
            planner.calculate(
                new Pose2d(1.0, 2.0, Rotation2d.kZero),
                List.of(),
                0.18,
                0.18,
                CategorySpec.kScore,
                false,
                0.0));
    Pose2d activeGoalBeforeFallback = planner.getGoalPose();

    RectangleObstacle block =
        RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.9, 2.0, 1.0, 1.0, 1.0);

    RepulsorSample sample =
        OffloadExecutionContext.runWorker(
            () ->
                planner.calculate(
                    new Pose2d(1.0, 2.0, Rotation2d.kZero),
                    List.of(block),
                    0.18,
                    0.18,
                    CategorySpec.kScore,
                    false,
                    0.0));

    assertTrue(planner.getGlobalFallbackStats().found(), "global fallback should find a route");
    assertTrue(
        Math.abs(sample.goal().getY() - 2.0) > 0.15,
        "returned sample should aim at an off-center temporary waypoint");
    assertNotEquals(
        finalGoal.getTranslation(), sample.goal(), "sample goal should be the temporary waypoint");
    assertEquals(
        activeGoalBeforeFallback.getTranslation().getX(),
        planner.getGoalPose().getTranslation().getX(),
        1e-9);
    assertEquals(
        activeGoalBeforeFallback.getTranslation().getY(),
        planner.getGoalPose().getTranslation().getY(),
        1e-9);
  }
}
