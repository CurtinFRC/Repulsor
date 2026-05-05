package org.curtinfrc.frc2026.util.Repulsor.Scenario;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;

/** Immutable setup for a repeatable FieldPlanner scenario test. */
public record RepulsorPlannerScenario(
    String name,
    FieldPlanner planner,
    Pose2d startPose,
    Pose2d requestedGoal,
    List<? extends Obstacle> dynamicObstacles,
    double robotHalfLengthMeters,
    double robotHalfWidthMeters,
    CategorySpec category,
    boolean suppressFallback,
    double shooterReleaseHeightMeters,
    Alliance fallbackAlliance,
    int maxSteps,
    double dtSeconds,
    double goalToleranceMeters) {
  public RepulsorPlannerScenario {
    if (name == null || name.isBlank()) name = "scenario";
    if (planner == null) planner = new FieldPlanner();
    if (startPose == null) startPose = Pose2d.kZero;
    if (requestedGoal == null) requestedGoal = startPose;
    dynamicObstacles = dynamicObstacles == null ? List.of() : List.copyOf(dynamicObstacles);
    robotHalfLengthMeters = Math.max(0.0, robotHalfLengthMeters);
    robotHalfWidthMeters = Math.max(0.0, robotHalfWidthMeters);
    category = category == null ? CategorySpec.kScore : category;
    shooterReleaseHeightMeters = Math.max(0.0, shooterReleaseHeightMeters);
    if (fallbackAlliance == null) fallbackAlliance = Alliance.kBlue;
    maxSteps = Math.max(1, maxSteps);
    dtSeconds = Double.isFinite(dtSeconds) && dtSeconds > 0.0 ? dtSeconds : 0.02;
    goalToleranceMeters = Math.max(0.0, goalToleranceMeters);
  }

  public static RepulsorPlannerScenario simple(
      String name, FieldPlanner planner, Pose2d startPose, Pose2d requestedGoal) {
    return new RepulsorPlannerScenario(
        name,
        planner,
        startPose,
        requestedGoal,
        List.of(),
        0.18,
        0.18,
        CategorySpec.kScore,
        false,
        0.0,
        Alliance.kBlue,
        100,
        0.02,
        0.12);
  }
}
