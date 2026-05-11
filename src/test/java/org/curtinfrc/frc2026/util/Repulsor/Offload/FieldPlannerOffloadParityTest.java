package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorDiagnosticsSnapshot;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.junit.jupiter.api.Test;

class FieldPlannerOffloadParityTest {
  @Test
  void offloadEntrypointMatchesLocalWorkerCalculation() {
    Pose2d pose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(15.0));
    Pose2d goal = new Pose2d(4.0, 2.5, Rotation2d.fromDegrees(30.0));
    assertPlannerParity(pose, goal, List.of(), CategorySpec.kScore);
  }

  @Test
  void offloadEntrypointMatchesLocalWithDynamicObstacle() {
    Pose2d pose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(15.0));
    Pose2d goal = new Pose2d(4.0, 2.5, Rotation2d.fromDegrees(30.0));
    RectangleObstacle obstacle =
        RectangleObstacle.simple(new Translation2d(2.5, 1.7), 0.4, 0.4, 1.0, 1.0, 1.0);

    assertPlannerParity(pose, goal, List.of(obstacle), CategorySpec.kScore);
  }

  @Test
  void offloadEntrypointMatchesLocalWhenGlobalFallbackProvidesTemporaryWaypoint() {
    Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.kZero);
    Pose2d goal = new Pose2d(5.0, 2.0, Rotation2d.kZero);
    RectangleObstacle blocker =
        RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.9, 2.0, 1.0, 1.0, 1.0);

    assertPlannerParity(pose, goal, List.of(blocker), CategorySpec.kScore);
  }

  private static void assertPlannerParity(
      Pose2d pose, Pose2d goal, List<? extends Obstacle> obstacles, CategorySpec category) {
    FieldPlannerOffloadLocalAccess.resetPlannerForTesting();
    FieldPlanner localPlanner = new FieldPlanner();
    localPlanner.syncGoalManagerState(goal, goal);

    RepulsorSample local =
        OffloadExecutionContext.runWorker(
            () -> {
              FieldPlanner.setOffloadFallbackAlliance(Alliance.kBlue);
              try {
                return localPlanner.calculate(pose, obstacles, 0.18, 0.18, category, false, 0.0);
              } finally {
                FieldPlanner.clearOffloadFallbackAlliance();
              }
            });

    FieldPlannerCalculateResultDTO offloaded =
        FieldPlannerOffloadEntrypoints.calculate(
            pose,
            goal,
            goal,
            obstacles,
            0.18,
            0.18,
            category.name(),
            Alliance.kBlue.name(),
            false,
            0.0);

    assertEquals(local.goal().getX(), offloaded.getGoalX(), 1e-9);
    assertEquals(local.goal().getY(), offloaded.getGoalY(), 1e-9);
    assertEquals(local.vxMetersPerSecond(), offloaded.getVxMetersPerSecond(), 1e-9);
    assertEquals(local.vyMetersPerSecond(), offloaded.getVyMetersPerSecond(), 1e-9);
    assertEquals(local.omegaRadians(), offloaded.getOmegaRadians(), 1e-9);
    assertEquals(localPlanner.getErr().isPresent(), offloaded.isHasErrMeters());
    if (localPlanner.getErr().isPresent()) {
      assertEquals(
          localPlanner.getErr().orElseThrow().baseUnitMagnitude(), offloaded.getErrMeters(), 1e-9);
    } else {
      assertEquals(0.0, offloaded.getErrMeters(), 1e-9);
    }
    assertEquals(localPlanner.getGoalPose().getX(), offloaded.getActiveGoalX(), 1e-9);
    assertEquals(localPlanner.getGoalPose().getY(), offloaded.getActiveGoalY(), 1e-9);
    assertEquals(RepulsorOffloadContract.CONTRACT_VERSION, offloaded.getContractVersion());
    assertEquals(
        RepulsorOffloadContract.FIELD_PLANNER_CALCULATE_VERSION, offloaded.getTaskVersion());
    assertTrue(RepulsorOffloadContract.isCompatible(offloaded.getContractVersion()));
    assertDiagnosticsParity(localPlanner.lastPlanningResult().diagnostics(), offloaded);
  }

  private static void assertDiagnosticsParity(
      RepulsorDiagnosticsSnapshot local, FieldPlannerCalculateResultDTO offloaded) {
    assertEquals(local.pathBlocked(), offloaded.isPathBlocked());
    assertEquals(local.globalFallbackActive(), offloaded.isGlobalFallbackActive());
    assertEquals(local.reactiveBypassActive(), offloaded.isReactiveBypassActive());
    assertEquals(local.reactiveBypassPinned(), offloaded.isReactiveBypassPinned());
    assertEquals(local.forceThroughActive(), offloaded.isForceThroughActive());
    assertEquals(local.robotIntersecting(), offloaded.isRobotIntersecting());
    assertEquals(local.stuckAbort(), offloaded.isStuckAbort());
    assertEquals(
        local.globalFallbackWaypoint().isPresent(), offloaded.isHasGlobalFallbackWaypoint());
    assertEquals(local.globalFallbackStats().found(), offloaded.isGlobalFallbackFound());
    assertEquals(local.globalFallbackStats().timedOut(), offloaded.isGlobalFallbackTimedOut());
    assertEquals(
        local.globalFallbackStats().exhaustedNodeBudget(),
        offloaded.isGlobalFallbackExhaustedNodeBudget());
    assertEquals(
        local.globalFallbackStats().expandedNodes(), offloaded.getGlobalFallbackExpandedNodes());
    assertEquals(
        local.globalFallbackStats().generatedNodes(), offloaded.getGlobalFallbackGeneratedNodes());
    assertEquals(
        local.globalFallbackStats().rawPathNodes(), offloaded.getGlobalFallbackRawPathNodes());
    assertEquals(local.globalFallbackStats().pathNodes(), offloaded.getGlobalFallbackPathNodes());
    assertEquals(
        local.globalFallbackStats().routeCostBreakdown().total(),
        offloaded.getGlobalFallbackRouteTotalCost(),
        1e-9);
    assertEquals(
        local.globalFallbackStats().routeCostBreakdown().distanceCost(),
        offloaded.getGlobalFallbackRouteDistanceCost(),
        1e-9);
    assertEquals(
        local.globalFallbackStats().routeCostBreakdown().obstacleClearanceCost(),
        offloaded.getGlobalFallbackRouteObstacleClearanceCost(),
        1e-9);
    assertEquals(
        local.globalFallbackStats().routeCostBreakdown().wallClearanceCost(),
        offloaded.getGlobalFallbackRouteWallClearanceCost(),
        1e-9);
    assertEquals(
        local.globalFallbackStats().routeCostBreakdown().turnCost(),
        offloaded.getGlobalFallbackRouteTurnCost(),
        1e-9);
    assertEquals(
        local.globalFallbackStats().routeClearanceMetrics().minRouteClearanceMeters(),
        offloaded.getGlobalFallbackMinRouteClearanceMeters(),
        1e-9);
    assertEquals(
        local.globalFallbackStats().routeClearanceMetrics().averageRouteClearanceMeters(),
        offloaded.getGlobalFallbackAverageRouteClearanceMeters(),
        1e-9);
    assertEquals(
        local.globalFallbackStats().selectedWaypointIndex(),
        offloaded.getGlobalFallbackSelectedWaypointIndex());
    assertEquals(
        local.globalFallbackStats().selectedWaypointReason().name(),
        offloaded.getGlobalFallbackSelectedWaypointReason());
    assertEquals(
        local.globalFallbackStats().failureReason().name(),
        offloaded.getGlobalFallbackFailureReason());
    assertTrue(offloaded.isHasSelectedCandidate());
    assertEquals(offloaded.getGoalX(), offloaded.getSelectedCandidateX(), 1e-9);
    assertEquals(offloaded.getGoalY(), offloaded.getSelectedCandidateY(), 1e-9);
    assertFalse(offloaded.getSelectedCandidateReason().isBlank());
    assertTrue(offloaded.getTraceSummary().startsWith("fieldPlanner"));
    if (local.waypointStatus() != null) {
      assertEquals(local.waypointStatus().activeStage(), offloaded.isWaypointActiveStage());
      assertEquals(local.waypointStatus().usingBypass(), offloaded.isWaypointUsingBypass());
      assertEquals(
          local.waypointStatus().stagedModeTicks(), offloaded.getWaypointStagedModeTicks());
    }
  }
}
