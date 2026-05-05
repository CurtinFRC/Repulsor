package org.curtinfrc.frc2026.util.Repulsor.Scenario;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlannerRuntimeConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointStrategy;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadExecutionContext;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultDriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultTurnTuning;
import org.junit.jupiter.api.Test;

class RepulsorPlannerScenarioRunnerTest {
  @Test
  void clearScenarioProgressesTowardGoalWithoutFallbackLayers() {
    RepulsorPlannerScenario scenario =
        RepulsorPlannerScenario.simple(
            "clear-score",
            new FieldPlanner(),
            new Pose2d(1.0, 1.0, Rotation2d.kZero),
            new Pose2d(3.0, 1.0, Rotation2d.kZero));

    RepulsorPlannerScenarioResult result =
        OffloadExecutionContext.runWorker(() -> RepulsorPlannerScenarioRunner.run(scenario));

    assertTrue(result.madeProgress());
    assertTrue(result.finalDistanceMeters() < result.initialDistanceMeters() * 0.80);
    assertEquals(0, result.pathBlockedCycles());
    assertEquals(0, result.globalFallbackCycles());
    assertEquals(0, result.robotIntersectingCycles());
    assertTrue(result.maxCommandSpeedMetersPerSecond() > 0.0);
  }

  @Test
  void blockedScenarioRecordsGlobalFallbackInsteadOfMutatingGoal() {
    FieldPlanner planner = new FieldPlanner();
    Pose2d goal = new Pose2d(5.0, 2.0, Rotation2d.kZero);
    RectangleObstacle block =
        RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.9, 2.0, 1.0, 1.0, 1.0);
    RepulsorPlannerScenario scenario =
        new RepulsorPlannerScenario(
            "blocked-global-fallback",
            planner,
            new Pose2d(1.0, 2.0, Rotation2d.kZero),
            goal,
            List.of(block),
            0.18,
            0.18,
            CategorySpec.kScore,
            false,
            0.0,
            Alliance.kBlue,
            1,
            0.02,
            0.12);

    RepulsorPlannerScenarioResult result =
        OffloadExecutionContext.runWorker(() -> RepulsorPlannerScenarioRunner.run(scenario));

    assertTrue(result.globalFallbackCycles() > 0);
    assertEquals(0, result.pathBlockedCycles());
    assertTrue(result.lastPlanningResult().diagnostics().globalFallbackStats().found());
    assertEquals(goal, planner.getRequestedGoalPose());
    assertTrue(result.lastPlanningResult().diagnostics().globalFallbackWaypoint().isPresent());
    assertFalse(result.lastPlanningResult().diagnostics().reactiveBypassActive());
  }

  @Test
  void disabledGlobalFallbackScenarioReportsBlockedWithoutReactiveBypassOverlap() {
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
    Pose2d goal = new Pose2d(5.0, 2.0, Rotation2d.kZero);
    RectangleObstacle block =
        RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.9, 2.0, 1.0, 1.0, 1.0);
    RepulsorPlannerScenario scenario =
        new RepulsorPlannerScenario(
            "blocked-no-global-fallback",
            planner,
            new Pose2d(1.0, 2.0, Rotation2d.kZero),
            goal,
            List.of(block),
            0.18,
            0.18,
            CategorySpec.kScore,
            false,
            0.0,
            Alliance.kBlue,
            3,
            0.02,
            0.12);

    RepulsorPlannerScenarioResult result =
        OffloadExecutionContext.runWorker(() -> RepulsorPlannerScenarioRunner.run(scenario));

    assertTrue(result.pathBlockedCycles() > 0);
    assertEquals(0, result.globalFallbackCycles());
    assertEquals(0, result.reactiveBypassCycles());
    assertFalse(result.madeProgress());
    assertEquals(0.0, result.maxCommandSpeedMetersPerSecond(), 1e-9);
  }
}
