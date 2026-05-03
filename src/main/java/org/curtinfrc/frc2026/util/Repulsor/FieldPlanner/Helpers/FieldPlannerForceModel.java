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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.NtRepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.RepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadExecutionContext;

/**
 * Provides field planner force model functionality for the Repulsor repulsor-field planner that
 * combines goals, obstacles, and force samples. Use this type from robot code, field profiles, or
 * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public final class FieldPlannerForceModel {
  private static final int ARROWS_X = isSimulationSafe() ? 40 : 0;
  private static final int ARROWS_Y = isSimulationSafe() ? 20 : 0;
  private static final int ARROWS_SIZE = (ARROWS_X + 1) * (ARROWS_Y + 1);
  private static final double CORNER_CHAMFER = 0.0;

  private final List<Obstacle> fieldObstacles;
  private final List<Obstacle> walls;
  private final double fieldLengthMeters;
  private final double fieldWidthMeters;
  private final ArrayList<Pose2d> arrows = new ArrayList<>(ARROWS_SIZE);
  private final Pose2d arrowBackstage =
      new Pose2d(-10, -10, edu.wpi.first.math.geometry.Rotation2d.kZero);

  /**
   * Returns the field planner force model value maintained by this Repulsor component.
   *
   * @param fieldObstacles obstacle set used for safety checks, costs, or replanning.
   * @param walls value used by this operation.
   * @param fieldLengthMeters distance or field-coordinate value in meters.
   * @param fieldWidthMeters distance or field-coordinate value in meters.
   */
  public FieldPlannerForceModel(
      List<Obstacle> fieldObstacles,
      List<Obstacle> walls,
      double fieldLengthMeters,
      double fieldWidthMeters) {
    this.fieldObstacles = fieldObstacles;
    this.walls = walls;
    this.fieldLengthMeters = fieldLengthMeters;
    this.fieldWidthMeters = fieldWidthMeters;
    for (int i = 0; i < ARROWS_SIZE; i++) arrows.add(new Pose2d());
  }

  private boolean isInsideChamferedField(Translation2d p) {
    double x = p.getX();
    double y = p.getY();
    double L = fieldLengthMeters;
    double W = fieldWidthMeters;

    if (x < 0.0 || x > L || y < 0.0 || y > W) return false;

    if (x <= CORNER_CHAMFER && y <= CORNER_CHAMFER && x + y < CORNER_CHAMFER) return false;

    if (x >= L - CORNER_CHAMFER && y <= CORNER_CHAMFER && (L - x) + y < CORNER_CHAMFER)
      return false;

    if (x <= CORNER_CHAMFER && y >= W - CORNER_CHAMFER && x + (W - y) < CORNER_CHAMFER)
      return false;

    if (x >= L - CORNER_CHAMFER && y >= W - CORNER_CHAMFER && (L - x) + (W - y) < CORNER_CHAMFER)
      return false;

    return true;
  }

  /**
   * Updates update arrows state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param goal value used by this operation.
   * @param dynamicObstacles obstacle set used for safety checks, costs, or replanning.
   */
  public void updateArrows(Translation2d goal, List<? extends Obstacle> dynamicObstacles) {
    if (isOffloadWorkerThread() || isRealSafe()) {
      return;
    }

    for (int x = 0; x <= ARROWS_X; x++) {
      for (int y = 0; y <= ARROWS_Y; y++) {
        var translation =
            new Translation2d(x * fieldLengthMeters / ARROWS_X, y * fieldWidthMeters / ARROWS_Y);

        int idx = x * (ARROWS_Y + 1) + y;

        if (!isInsideChamferedField(translation)) {
          arrows.set(idx, arrowBackstage);
          continue;
        }

        var force = Force.kZero;
        force = force.plus(sampleObstacleForce(translation, goal, dynamicObstacles));
        force = force.plus(sampleWallForce(translation, goal));
        force = force.plus(getGoalForce(translation, goal));

        if (force.getNorm() < 1e-6) {
          arrows.set(idx, arrowBackstage);
        } else {
          var rotation = force.getAngle();
          arrows.set(idx, new Pose2d(translation, rotation));
        }
      }
    }
  }

  /**
   * Returns the get arrows value maintained by this Repulsor component.
   *
   * @return array list of pose2d result for get arrows.
   */
  public ArrayList<Pose2d> getArrows() {
    return arrows;
  }

  private Force sampleWallForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : walls) force = force.plus(obs.sampleForceAtPosition(curLocation, target));
    return force;
  }

  private Force sampleObstacleForce(
      Translation2d curLocation, Translation2d target, List<? extends Obstacle> extra) {
    var force = Force.kZero;
    var dsBase = safeDriverStation();
    for (Obstacle obs : fieldObstacles)
      force = force.plus(obs.sampleForceAtPosition(curLocation, target));
    for (Obstacle obs : extra)
      force =
          force.plus(
              obs.sampleForceAtPosition(curLocation, target)
                  .times(
                      dsBase instanceof NtRepulsorDriverStation ds
                          ? ds.getConfigDouble("repulsion_scale")
                          : 1.0));
    return force;
  }

  /**
   * Returns the get goal force value maintained by this Repulsor component.
   *
   * @param curLocation value used by this operation.
   * @param goal value used by this operation.
   * @return value produced by this operation.
   */
  public Force getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) return new Force();
    var direction = displacement.getAngle();
    var mag =
        FieldPlanner.GOAL_STRENGTH
            * (1 + 1.0 / (0.0001 + displacement.getNorm() * displacement.getNorm()));
    return new Force(mag, direction);
  }

  /**
   * Returns the get wall force value maintained by this Repulsor component.
   *
   * @param curLocation value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  public Force getWallForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : walls) force = force.plus(obs.getForceAtPosition(curLocation, target));
    return force;
  }

  /**
   * Returns the get obstacle force value maintained by this Repulsor component.
   *
   * @param curLocation value used by this operation.
   * @param target value used by this operation.
   * @param extra value used by this operation.
   * @return value produced by this operation.
   */
  public Force getObstacleForce(
      Translation2d curLocation, Translation2d target, List<? extends Obstacle> extra) {
    var force = Force.kZero;
    var dsBase = safeDriverStation();
    for (Obstacle obs : fieldObstacles)
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    for (Obstacle obs : extra)
      force =
          force.plus(
              obs.getForceAtPosition(curLocation, target)
                  .times(
                      dsBase instanceof NtRepulsorDriverStation ds
                          ? ds.getConfigDouble("repulsion_scale")
                          : 1.0));
    return force;
  }

  /**
   * Returns the get obstacle force value maintained by this Repulsor component.
   *
   * @param curLocation value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  public Force getObstacleForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : fieldObstacles)
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    return force;
  }

  /**
   * Returns the get force value maintained by this Repulsor component.
   *
   * @param curLocation value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  public Force getForce(Translation2d curLocation, Translation2d target) {
    return getGoalForce(curLocation, target)
        .plus(getObstacleForce(curLocation, target))
        .plus(getWallForce(curLocation, target))
        .times(Math.min(1.0, curLocation.getDistance(target)));
  }

  private static boolean isOffloadWorkerThread() {
    return OffloadExecutionContext.isWorkerOrLegacyThread();
  }

  private static boolean isSimulationSafe() {
    if (isOffloadWorkerThread()) {
      return false;
    }
    try {
      return RobotBase.isSimulation();
    } catch (Throwable ignored) {
      return false;
    }
  }

  private static boolean isRealSafe() {
    if (isOffloadWorkerThread()) {
      return true;
    }
    try {
      return RobotBase.isReal();
    } catch (Throwable ignored) {
      return true;
    }
  }

  private static RepulsorDriverStation safeDriverStation() {
    try {
      return RepulsorDriverStation.getInstance();
    } catch (Throwable ignored) {
      return null;
    }
  }
}
