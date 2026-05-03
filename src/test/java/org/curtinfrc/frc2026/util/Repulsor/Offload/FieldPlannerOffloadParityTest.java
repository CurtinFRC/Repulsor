package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.junit.jupiter.api.Test;

class FieldPlannerOffloadParityTest {
  @Test
  void offloadEntrypointMatchesLocalWorkerCalculation() {
    Pose2d pose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(15.0));
    Pose2d goal = new Pose2d(4.0, 2.5, Rotation2d.fromDegrees(30.0));

    FieldPlanner localPlanner = new FieldPlanner();
    localPlanner.syncGoalManagerState(goal, goal);

    RepulsorSample local =
        OffloadExecutionContext.runWorker(
            () -> {
              FieldPlanner.setOffloadFallbackAlliance(Alliance.kBlue);
              try {
                return localPlanner.calculate(
                    pose, List.of(), 0.18, 0.18, CategorySpec.kScore, false, 0.0);
              } finally {
                FieldPlanner.clearOffloadFallbackAlliance();
              }
            });

    FieldPlannerCalculateResultDTO offloaded =
        FieldPlannerOffloadEntrypoints.calculate(
            pose,
            goal,
            goal,
            List.of(),
            0.18,
            0.18,
            CategorySpec.kScore.name(),
            Alliance.kBlue.name(),
            false,
            0.0);

    assertEquals(local.goal().getX(), offloaded.getGoalX(), 1e-9);
    assertEquals(local.goal().getY(), offloaded.getGoalY(), 1e-9);
    assertEquals(local.vxMetersPerSecond(), offloaded.getVxMetersPerSecond(), 1e-9);
    assertEquals(local.vyMetersPerSecond(), offloaded.getVyMetersPerSecond(), 1e-9);
    assertEquals(local.omegaRadians(), offloaded.getOmegaRadians(), 1e-9);
    assertEquals(localPlanner.getErr().isPresent(), offloaded.isHasErrMeters());
    assertEquals(
        localPlanner.getErr().orElseThrow().baseUnitMagnitude(), offloaded.getErrMeters(), 1e-9);
    assertEquals(localPlanner.getGoalPose().getX(), offloaded.getActiveGoalX(), 1e-9);
    assertEquals(localPlanner.getGoalPose().getY(), offloaded.getActiveGoalY(), 1e-9);
  }
}
