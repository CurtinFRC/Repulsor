package org.curtinfrc.frc2026.util.Repulsor.Offload;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

@SuppressWarnings("unused")
/**
 * Provides field planner offload entrypoints functionality for the Repulsor offload serialization
 * and native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class FieldPlannerOffloadEntrypoints {
  private FieldPlannerOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.FIELD_PLANNER_CALCULATE,
      version = 2,
      timeoutMs = 250,
      fallback = false)
  /**
   * Computes the calculate value for the current Repulsor planning state. Call this from periodic
   * planning or tests when a fresh decision is required; inputs should already be expressed in the
   * coordinate frame expected by the parameter names.
   *
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @param requestedGoalPose value used by this operation.
   * @param activeGoalPose value used by this operation.
   * @param dynamicObstacles obstacle set used for safety checks, costs, or replanning.
   * @param robot_x distance or field-coordinate value in meters.
   * @param robot_y distance or field-coordinate value in meters.
   * @param categoryName value used by this operation.
   * @param preferredAllianceName value used by this operation.
   * @param suppressFallback value used by this operation.
   * @param shooterReleaseHeightMeters distance or field-coordinate value in meters.
   * @return field planner calculate result dto result for calculate.
   */
  public static FieldPlannerCalculateResultDTO calculate(
      Pose2d pose,
      Pose2d requestedGoalPose,
      Pose2d activeGoalPose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      String categoryName,
      String preferredAllianceName,
      boolean suppressFallback,
      double shooterReleaseHeightMeters) {
    return OffloadExecutionContext.runWorker(
        () ->
            FieldPlannerOffloadLocalAccess.calculateLocal(
                pose,
                requestedGoalPose,
                activeGoalPose,
                dynamicObstacles,
                robot_x,
                robot_y,
                categoryName,
                preferredAllianceName,
                suppressFallback,
                shooterReleaseHeightMeters));
  }
}
