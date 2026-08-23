package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;

/** Central immutable request for one Repulsor planning cycle. */
public record RepulsorPlanningRequest(
    Pose2d pose,
    List<? extends Obstacle> dynamicObstacles,
    double robotLengthMeters,
    double robotWidthMeters,
    CategorySpec category,
    boolean suppressFallback,
    double shooterReleaseHeightMeters,
    Optional<Alliance> fallbackAllianceOverride,
    String strategyMode) {
  public RepulsorPlanningRequest {
    if (pose == null) pose = Pose2d.kZero;
    dynamicObstacles = dynamicObstacles == null ? List.of() : List.copyOf(dynamicObstacles);
    robotLengthMeters = Math.max(0.0, robotLengthMeters);
    robotWidthMeters = Math.max(0.0, robotWidthMeters);
    category = category == null ? CategorySpec.kScore : category;
    shooterReleaseHeightMeters = Math.max(0.0, shooterReleaseHeightMeters);
    fallbackAllianceOverride =
        fallbackAllianceOverride == null ? Optional.empty() : fallbackAllianceOverride;
    if (strategyMode == null || strategyMode.isBlank()) strategyMode = "default";
  }

  public RepulsorPlanningRequest(
      Pose2d pose,
      List<? extends Obstacle> dynamicObstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      CategorySpec category,
      boolean suppressFallback,
      double shooterReleaseHeightMeters,
      Alliance fallbackAllianceOverride,
      String strategyMode) {
    this(
        pose,
        dynamicObstacles,
        robotLengthMeters,
        robotWidthMeters,
        category,
        suppressFallback,
        shooterReleaseHeightMeters,
        Optional.ofNullable(fallbackAllianceOverride),
        strategyMode);
  }

  public static RepulsorPlanningRequest from(PlannerCalculationRequest request) {
    if (request == null) {
      return new RepulsorPlanningRequest(
          Pose2d.kZero,
          List.of(),
          0.0,
          0.0,
          CategorySpec.kScore,
          true,
          0.0,
          Optional.empty(),
          "default");
    }
    return new RepulsorPlanningRequest(
        request.pose(),
        request.dynamicObstacles(),
        request.robotLengthMeters(),
        request.robotWidthMeters(),
        request.category(),
        request.suppressFallback(),
        request.shooterReleaseHeightMeters(),
        Optional.empty(),
        "default");
  }

  public PlannerCalculationRequest toPlannerCalculationRequest() {
    return new PlannerCalculationRequest(
        pose,
        dynamicObstacles,
        robotLengthMeters,
        robotWidthMeters,
        category,
        suppressFallback,
        shooterReleaseHeightMeters);
  }
}
