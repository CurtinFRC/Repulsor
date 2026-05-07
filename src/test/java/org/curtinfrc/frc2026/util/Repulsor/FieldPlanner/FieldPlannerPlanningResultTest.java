package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointStrategy;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadExecutionContext;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultDriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultTurnTuning;
import org.junit.jupiter.api.Test;

class FieldPlannerPlanningResultTest {
  @Test
  void detailedCalculationReturnsStoredSampleAndDiagnosticsSnapshot() {
    FieldPlanner planner = new FieldPlanner();
    Pose2d goal = new Pose2d(3.0, 1.0, Rotation2d.kZero);
    planner.setRequestedGoal(goal);

    RepulsorPlanningRequest request =
        new RepulsorPlanningRequest(
            new Pose2d(1.0, 1.0, Rotation2d.kZero),
            List.of(),
            0.18,
            0.18,
            CategorySpec.kScore,
            false,
            0.0,
            Alliance.kBlue,
            "score-default");

    RepulsorPlanningResult result =
        OffloadExecutionContext.runWorker(() -> planner.calculateDetailed(request));

    assertSame(result, planner.lastPlanningResult());
    assertEquals(request, result.request());
    assertNotNull(result.sample());
    assertEquals(goal, result.diagnostics().requestedGoal());
    assertNotNull(result.diagnostics().waypointStatus());
    assertFalse(result.diagnostics().pathBlocked());
    assertFalse(result.diagnostics().robotIntersecting());
    assertFalse(result.diagnostics().stuckAbort());
    assertFalse(result.diagnostics().offloaded());
    assertTrue(Double.isFinite(result.diagnostics().errorMeters()));
    assertFalse(result.decisionTrace().entries().isEmpty());
    assertTrue(result.decisionTrace().hasLayer("WaypointPolicy"));
    assertTrue(result.decisionTrace().hasLayer("GlobalFallback"));
    assertTrue(result.decisionTrace().hasLayer("ReactiveBypass"));
    assertTrue(result.decisionTrace().hasLayer("ForceThrough"));
    assertTrue(result.decisionTrace().hasLayer("FieldPlanner"));
  }

  @Test
  void globalFallbackStateIsReportedWithoutMutatingActiveGoal() {
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

    RepulsorPlanningResult result =
        OffloadExecutionContext.runWorker(
            () ->
                planner.calculateDetailed(
                    new RepulsorPlanningRequest(
                        new Pose2d(1.0, 2.0, Rotation2d.kZero),
                        List.of(block),
                        0.18,
                        0.18,
                        CategorySpec.kScore,
                        false,
                        0.0,
                        Alliance.kBlue,
                        "blocked-score")));

    assertTrue(result.diagnostics().globalFallbackActive());
    assertTrue(result.diagnostics().globalFallbackWaypoint().isPresent());
    assertTrue(result.diagnostics().globalFallbackStats().found());
    assertFalse(result.diagnostics().pathBlocked());
    assertEquals(activeGoalBeforeFallback, planner.getGoalPose());
    assertTrue(
        result.decisionTrace().entries().stream()
            .anyMatch(
                entry ->
                    entry.layer().equals("GlobalFallback")
                        && entry.decision().equals("temporary_waypoint")
                        && entry.reason().equals("search_found_waypoint")));
  }

  @Test
  void runtimeConfigCanDisableGlobalFallbackWithoutInvokingReactiveBypass() {
    FieldPlannerRuntimeConfig runtimeConfig =
        FieldPlannerRuntimeConfig.defaults().withGlobalFallbackEnabled(false);
    FieldPlanner planner =
        new FieldPlanner(
            new DefaultTurnTuning(),
            new DefaultDriveTuning(),
            new FieldPlanner.DefaultObstacleProvider(),
            FieldPlannerWaypointConfig.defaults(),
            FieldPlannerWaypointStrategy.defaults(),
            runtimeConfig);
    planner.setRequestedGoal(new Pose2d(5.0, 2.0, Rotation2d.kZero));
    RectangleObstacle block =
        RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.9, 2.0, 1.0, 1.0, 1.0);

    RepulsorPlanningResult result =
        OffloadExecutionContext.runWorker(
            () ->
                planner.calculateDetailed(
                    new RepulsorPlanningRequest(
                        new Pose2d(1.0, 2.0, Rotation2d.kZero),
                        List.of(block),
                        0.18,
                        0.18,
                        CategorySpec.kScore,
                        false,
                        0.0,
                        Alliance.kBlue,
                        "blocked-no-global-fallback")));

    assertFalse(result.diagnostics().globalFallbackActive());
    assertTrue(result.diagnostics().globalFallbackWaypoint().isEmpty());
    assertTrue(result.diagnostics().pathBlocked());
    assertFalse(result.diagnostics().reactiveBypassActive());
    assertEquals(runtimeConfig, planner.getRuntimeConfig());
    assertTrue(
        result.decisionTrace().entries().stream()
            .anyMatch(
                entry ->
                    entry.layer().equals("FieldPlanner")
                        && entry.decision().equals("path_blocked")));
  }

  @Test
  void legacyRequestsDoNotForceFallbackAllianceOverrideAndCopyDynamicObstacles() {
    ArrayList<Obstacle> dynamicObstacles = new ArrayList<>();
    RectangleObstacle obstacle =
        RectangleObstacle.simple(new Translation2d(2.0, 2.0), 0.2, 0.2, 1.0, 1.0, 1.0);
    dynamicObstacles.add(obstacle);

    RepulsorPlanningRequest request =
        RepulsorPlanningRequest.from(
            new PlannerCalculationRequest(
                new Pose2d(1.0, 1.0, Rotation2d.kZero),
                dynamicObstacles,
                0.18,
                0.18,
                CategorySpec.kCollect,
                false,
                0.3));

    dynamicObstacles.clear();

    assertTrue(request.fallbackAllianceOverride().isEmpty());
    assertEquals(CategorySpec.kCollect, request.category());
    assertEquals(1, request.dynamicObstacles().size());
    assertSame(obstacle, request.dynamicObstacles().get(0));
  }
}
