package org.curtinfrc.frc2026.util.Repulsor.Offload;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathing;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

@SuppressWarnings("unused")
/**
 * Provides field planner pathing offload entrypoints functionality for the Repulsor offload
 * serialization and native/JNI entrypoint boundary. Use this type from robot code, field profiles,
 * or tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public final class FieldPlannerPathingOffloadEntrypoints {
  private FieldPlannerPathingOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.FIELD_PLANNER_IS_CLEAR_PATH,
      version = 1,
      timeoutMs = 120,
      fallback = true)
  /**
   * Returns the is clear path value maintained by this Repulsor component.
   *
   * @param topicRoot value used by this operation.
   * @param start value used by this operation.
   * @param goal value used by this operation.
   * @param obstacles obstacle set used for safety checks, costs, or replanning.
   * @param robotLengthMeters distance or field-coordinate value in meters.
   * @param robotWidthMeters distance or field-coordinate value in meters.
   * @param publishSamples value used by this operation.
   * @return value produced by this operation.
   */
  public static boolean isClearPath(
      String topicRoot,
      Translation2d start,
      Translation2d goal,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      boolean publishSamples) {
    return OffloadExecutionContext.runWorker(
        () ->
            ExtraPathing.isClearPath(
                topicRoot,
                start,
                goal,
                obstacles,
                robotLengthMeters,
                robotWidthMeters,
                publishSamples));
  }

  @Offloadable(
      id = OffloadTaskIds.FIELD_PLANNER_ROBOT_INTERSECTS,
      version = 1,
      timeoutMs = 120,
      fallback = true)
  /**
   * Returns the robot intersects value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param robotLengthMeters distance or field-coordinate value in meters.
   * @param robotWidthMeters distance or field-coordinate value in meters.
   * @param obstacles obstacle set used for safety checks, costs, or replanning.
   * @return value produced by this operation.
   */
  public static boolean robotIntersects(
      Translation2d center,
      double robotLengthMeters,
      double robotWidthMeters,
      List<? extends Obstacle> obstacles) {
    return OffloadExecutionContext.runWorker(
        () -> ExtraPathing.robotIntersects(center, robotLengthMeters, robotWidthMeters, obstacles));
  }
}
