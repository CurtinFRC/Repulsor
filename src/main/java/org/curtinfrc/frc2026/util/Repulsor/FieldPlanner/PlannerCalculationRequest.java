package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;

/** Immutable inputs for a single {@link FieldPlanner} calculation. */
public record PlannerCalculationRequest(
    Pose2d pose,
    List<? extends Obstacle> dynamicObstacles,
    double robotLengthMeters,
    double robotWidthMeters,
    CategorySpec category,
    boolean suppressFallback,
    double shooterReleaseHeightMeters) {

  public PlannerCalculationRequest {
    dynamicObstacles = dynamicObstacles == null ? List.of() : List.copyOf(dynamicObstacles);
    robotLengthMeters = Math.max(0.0, robotLengthMeters);
    robotWidthMeters = Math.max(0.0, robotWidthMeters);
    category = category == null ? CategorySpec.kScore : category;
    shooterReleaseHeightMeters = Math.max(0.0, shooterReleaseHeightMeters);
  }
}
