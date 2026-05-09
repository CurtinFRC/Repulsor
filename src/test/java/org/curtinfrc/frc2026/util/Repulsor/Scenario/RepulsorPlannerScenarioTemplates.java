package org.curtinfrc.frc2026.util.Repulsor.Scenario;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlannerRuntimeConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointDecision;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointPlan;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointStrategy;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultDriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultTurnTuning;

/** Reusable planner scenario templates for cross-game Repulsor validation. */
public final class RepulsorPlannerScenarioTemplates {
  private RepulsorPlannerScenarioTemplates() {}

  public static RepulsorPlannerScenario clearRoute(String name) {
    return clearRoute(
        name,
        new FieldPlanner(),
        new Pose2d(1.0, 1.0, Rotation2d.kZero),
        new Pose2d(3.0, 1.0, Rotation2d.kZero));
  }

  public static RepulsorPlannerScenario clearRoute(
      String name, FieldPlanner planner, Pose2d startPose, Pose2d goalPose) {
    return RepulsorPlannerScenario.simple(name, planner, startPose, goalPose);
  }

  public static RepulsorPlannerScenario centerCrossing(
      String name, double fieldLengthMeters, double fieldWidthMeters) {
    double centerY = Math.max(0.5, fieldWidthMeters * 0.5);
    return clearRoute(
        name,
        new FieldPlanner(),
        new Pose2d(Math.max(0.5, fieldLengthMeters * 0.20), centerY, Rotation2d.kZero),
        new Pose2d(Math.max(1.0, fieldLengthMeters * 0.80), centerY, Rotation2d.kZero));
  }

  public static RepulsorPlannerScenario dynamicObstacle(String name) {
    RectangleObstacle obstacle =
        RectangleObstacle.simple(new Translation2d(2.3, 1.2), 0.35, 0.35, 1.0, 1.0, 1.0);
    return withObstacles(
        name,
        new FieldPlanner(),
        new Pose2d(1.0, 1.0, Rotation2d.kZero),
        new Pose2d(4.0, 1.5, Rotation2d.kZero),
        List.of(obstacle),
        CategorySpec.kScore,
        false,
        24);
  }

  public static RepulsorPlannerScenario blockedRoute(String name) {
    RectangleObstacle blocker =
        RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.9, 2.0, 1.0, 1.0, 1.0);
    return withObstacles(
        name,
        new FieldPlanner(),
        new Pose2d(1.0, 2.0, Rotation2d.kZero),
        new Pose2d(5.0, 2.0, Rotation2d.kZero),
        List.of(blocker),
        CategorySpec.kScore,
        false,
        6);
  }

  public static RepulsorPlannerScenario blockedWithoutGlobalFallback(String name) {
    FieldPlanner planner =
        new FieldPlanner(
            new DefaultTurnTuning(),
            new DefaultDriveTuning(),
            new FieldPlanner.DefaultObstacleProvider(),
            FieldPlannerWaypointConfig.defaults(),
            FieldPlannerWaypointStrategy.defaults(),
            FieldPlannerRuntimeConfig.defaults().withGlobalFallbackEnabled(false));
    RectangleObstacle blocker =
        RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.9, 2.0, 1.0, 1.0, 1.0);
    return withObstacles(
        name,
        planner,
        new Pose2d(1.0, 2.0, Rotation2d.kZero),
        new Pose2d(5.0, 2.0, Rotation2d.kZero),
        List.of(blocker),
        CategorySpec.kScore,
        false,
        3);
  }

  public static RepulsorPlannerScenario forcedCollect(String name) {
    return withObstacles(
        name,
        new FieldPlanner(),
        new Pose2d(3.0, 3.0, Rotation2d.kZero),
        new Pose2d(1.2, 1.0, Rotation2d.kZero),
        List.of(),
        CategorySpec.kCollect,
        false,
        24);
  }

  public static RepulsorPlannerScenario forcedScore(String name) {
    return withObstacles(
        name,
        new FieldPlanner(),
        new Pose2d(1.2, 1.0, Rotation2d.kZero),
        new Pose2d(4.2, 3.2, Rotation2d.kZero),
        List.of(),
        CategorySpec.kScore,
        false,
        24);
  }

  public static RepulsorPlannerScenario defensiveObstacle(String name) {
    RectangleObstacle defender =
        RectangleObstacle.simple(new Translation2d(2.5, 1.0), 0.45, 0.75, 1.2, 1.0, 1.0);
    return withObstacles(
        name,
        new FieldPlanner(),
        new Pose2d(1.0, 1.0, Rotation2d.kZero),
        new Pose2d(4.0, 1.0, Rotation2d.kZero),
        List.of(defender),
        CategorySpec.kScore,
        false,
        24);
  }

  public static RepulsorPlannerScenario edgeOfField(String name, double fieldWidthMeters) {
    double y = Math.max(0.25, Math.min(fieldWidthMeters - 0.25, 0.35));
    return clearRoute(
        name,
        new FieldPlanner(),
        new Pose2d(1.0, y, Rotation2d.kZero),
        new Pose2d(3.5, y, Rotation2d.kZero));
  }

  public static RepulsorPlannerScenario customWaypointStage(String name) {
    FieldPlannerWaypointStrategy stageMidpoint =
        context ->
            FieldPlannerWaypointDecision.stage(
                FieldPlannerWaypointPlan.single(new Translation2d(2.0, 1.0)));
    FieldPlanner planner =
        new FieldPlanner(
            new DefaultTurnTuning(),
            new DefaultDriveTuning(),
            new FieldPlanner.DefaultObstacleProvider(),
            FieldPlannerWaypointConfig.defaults(),
            stageMidpoint,
            FieldPlannerRuntimeConfig.defaults());
    return clearRoute(
        name,
        planner,
        new Pose2d(1.0, 1.0, Rotation2d.kZero),
        new Pose2d(4.0, 1.0, Rotation2d.kZero));
  }

  private static RepulsorPlannerScenario withObstacles(
      String name,
      FieldPlanner planner,
      Pose2d start,
      Pose2d goal,
      List<? extends Obstacle> obstacles,
      CategorySpec category,
      boolean suppressFallback,
      int maxSteps) {
    return new RepulsorPlannerScenario(
        name,
        planner,
        start,
        goal,
        obstacles,
        0.18,
        0.18,
        category,
        suppressFallback,
        0.0,
        Alliance.kBlue,
        maxSteps,
        0.02,
        0.12);
  }
}
