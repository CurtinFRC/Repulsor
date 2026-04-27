/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Offload.DragShotPlannerOffloadEntrypoints_Offloaded;

/**
 * Provides drag shot planner functionality for the Repulsor projectile and shot-planning layer. Use
 * this type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public final class DragShotPlanner {
  private DragShotPlanner() {}

  /**
   * Returns the load game piece from deploy yaml value maintained by this Repulsor component.
   *
   * @param id value used by this operation.
   * @return game piece physics result for load game piece from deploy yaml.
   */
  public static GamePiecePhysics loadGamePieceFromDeployYaml(String id) {
    return DragShotPlannerCore.loadGamePieceFromDeployYaml(id);
  }

  /**
   * Returns the is shooter pose valid value maintained by this Repulsor component.
   *
   * @param shooterPos value used by this operation.
   * @param targetFieldPosition value used by this operation.
   * @param robotHalfLengthMeters distance or field-coordinate value in meters.
   * @param robotHalfWidthMeters distance or field-coordinate value in meters.
   * @param dynamicObstacles obstacle set used for safety checks, costs, or replanning.
   * @param checkBounds value used by this operation.
   * @return value produced by this operation.
   */
  public static boolean isShooterPoseValid(
      Translation2d shooterPos,
      Translation2d targetFieldPosition,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      boolean checkBounds) {
    return DragShotPlannerCore.isShooterPoseValid(
        shooterPos,
        targetFieldPosition,
        robotHalfLengthMeters,
        robotHalfWidthMeters,
        dynamicObstacles,
        checkBounds);
  }

  /**
   * Computes the find best shot from library value for the current Repulsor planning state.
   *
   * @param library distance or field-coordinate value in meters.
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
  public static Optional<ShotSolution> findBestShotFromLibrary(
      ShotLibrary library,
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      Constraints constraints) {
    return DragShotPlannerCore.findBestShotFromLibrary(
        library,
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
   * Computes the find best shot auto value for the current Repulsor planning state.
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
  public static Optional<ShotSolution> findBestShotAuto(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      Constraints constraints) {
    return DragShotPlannerOffloadEntrypoints_Offloaded.findBestShotAuto_offload(
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
   * Computes the find best shot auto async value for the current Repulsor planning state.
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
   * @return completable future containing an optional shot solution.
   */
  public static CompletableFuture<Optional<ShotSolution>> findBestShotAutoAsync(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      Constraints constraints) {
    return DragShotPlannerOffloadEntrypoints_Offloaded.findBestShotAuto_offloadAsync(
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
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      Constraints constraints) {
    return DragShotPlannerLocalAccess.findBestShotAutoLocal(
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
   * Computes the find best shot online refine value for the current Repulsor planning state.
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
   * @param state value used by this operation.
   * @param budgetNanos value used by this operation.
   * @return optional shot solution produced by this operation.
   */
  public static Optional<ShotSolution> findBestShotOnlineRefine(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      Constraints constraints,
      OnlineSearchState state,
      long budgetNanos) {
    return DragShotPlannerCore.findBestShotOnlineRefine(
        gamePiece,
        targetFieldPosition,
        targetHeightMeters,
        robotPose,
        shooterReleaseHeightMeters,
        robotHalfLengthMeters,
        robotHalfWidthMeters,
        dynamicObstacles,
        constraints,
        state,
        budgetNanos);
  }

  /**
   * Computes the calculate static shot angle and speed value for the current Repulsor planning
   * state. Call this from periodic planning or tests when a fresh decision is required; inputs
   * should already be expressed in the coordinate frame expected by the parameter names.
   *
   * @param gamePiece value used by this operation.
   * @param shooterFieldPosition value used by this operation.
   * @param targetFieldPosition value used by this operation.
   * @param targetHeightMeters distance or field-coordinate value in meters.
   * @param shooterReleaseHeightMeters distance or field-coordinate value in meters.
   * @param constraints value used by this operation.
   * @return optional shot solution produced by this operation.
   */
  public static Optional<ShotSolution> calculateStaticShotAngleAndSpeed(
      GamePiecePhysics gamePiece,
      Translation2d shooterFieldPosition,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      double shooterReleaseHeightMeters,
      Constraints constraints) {
    return DragShotPlannerOffloadEntrypoints_Offloaded.calculateStaticShotAngleAndSpeed_offload(
        gamePiece,
        shooterFieldPosition,
        targetFieldPosition,
        targetHeightMeters,
        shooterReleaseHeightMeters,
        constraints);
  }

  /**
   * Computes the calculate static shot angle and speed async value for the current Repulsor
   * planning state. Call this from periodic planning or tests when a fresh decision is required;
   * inputs should already be expressed in the coordinate frame expected by the parameter names.
   *
   * @param gamePiece value used by this operation.
   * @param shooterFieldPosition value used by this operation.
   * @param targetFieldPosition value used by this operation.
   * @param targetHeightMeters distance or field-coordinate value in meters.
   * @param shooterReleaseHeightMeters distance or field-coordinate value in meters.
   * @param constraints value used by this operation.
   * @return completable future containing an optional shot solution for calculate static shot angle
   *     and speed async.
   */
  public static CompletableFuture<Optional<ShotSolution>> calculateStaticShotAngleAndSpeedAsync(
      GamePiecePhysics gamePiece,
      Translation2d shooterFieldPosition,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      double shooterReleaseHeightMeters,
      Constraints constraints) {
    return DragShotPlannerOffloadEntrypoints_Offloaded
        .calculateStaticShotAngleAndSpeed_offloadAsync(
            gamePiece,
            shooterFieldPosition,
            targetFieldPosition,
            targetHeightMeters,
            shooterReleaseHeightMeters,
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
    return DragShotPlannerLocalAccess.calculateStaticShotAngleAndSpeedLocal(
        gamePiece,
        shooterFieldPosition,
        targetFieldPosition,
        targetHeightMeters,
        shooterReleaseHeightMeters,
        constraints);
  }
}
