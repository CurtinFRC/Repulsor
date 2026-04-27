package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

/**
 * Provides drag shot planner local access functionality for the Repulsor projectile and
 * shot-planning layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class DragShotPlannerLocalAccess {
  private DragShotPlannerLocalAccess() {}

  /**
   * Computes the find best shot auto local value for the current Repulsor planning state.
   *
   * @param gamePiece value used by this operation.
   * @param targetFieldPosition value used by this operation.
   * @param targetHeightMeters distance or field-coordinate value in meters.
   * @param robotPose WPILib Pose2d in field-relative coordinates.
   * @param shooterReleaseHeightMeters distance or field-coordinate value in meters.
   * @param robotHalfLengthMeters distance or field-coordinate value in meters.
   * @param robotHalfWidthMeters distance or field-coordinate value in meters.
   * @param dynamicObstacles obstacle set used for safety checks, costs, or replanning.
   * @param constraints value used by this operation.
   * @return optional shot solution produced by this operation.
   */
  public static Optional<ShotSolution> findBestShotAutoLocal(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      Constraints constraints) {
    return DragShotPlannerCore.findBestShotAuto(
        gamePiece,
        targetFieldPosition,
        targetHeightMeters,
        robotPose,
        shooterReleaseHeightMeters,
        robotHalfLengthMeters,
        robotHalfWidthMeters,
        dynamicObstacles,
        constraints);
  }

  /**
   * Computes the calculate static shot angle and speed local value for the current Repulsor
   * planning state. Call this from periodic planning or tests when a fresh decision is required;
   * inputs should already be expressed in the coordinate frame expected by the parameter names.
   *
   * @param gamePiece value used by this operation.
   * @param shooterFieldPosition value used by this operation.
   * @param targetFieldPosition value used by this operation.
   * @param targetHeightMeters distance or field-coordinate value in meters.
   * @param shooterReleaseHeightMeters distance or field-coordinate value in meters.
   * @param constraints value used by this operation.
   * @return optional shot solution produced by this operation.
   */
  public static Optional<ShotSolution> calculateStaticShotAngleAndSpeedLocal(
      GamePiecePhysics gamePiece,
      Translation2d shooterFieldPosition,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      double shooterReleaseHeightMeters,
      Constraints constraints) {
    return DragShotPlannerCore.calculateStaticShotAngleAndSpeed(
        gamePiece,
        shooterFieldPosition,
        targetFieldPosition,
        targetHeightMeters,
        shooterReleaseHeightMeters,
        constraints);
  }
}
