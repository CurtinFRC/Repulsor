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
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

/**
 * Provides drag shot planner candidate functionality for the Repulsor projectile and shot-planning
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
final class DragShotPlannerCandidate {
  /**
   * Configuration value for shooter position. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final Translation2d shooterPosition;

  /**
   * Configuration value for shooter yaw rad. Angles use WPILib rotation conventions; names ending
   * in degrees are degrees, otherwise radians are assumed by the API.
   */
  final double shooterYawRad;

  /**
   * Configuration value for speed. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final double speed;

  /**
   * Configuration value for angle rad. Angles use WPILib rotation conventions; names ending in
   * degrees are degrees, otherwise radians are assumed by the API.
   */
  final double angleRad;

  /**
   * Configuration value for time to plane. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  final double timeToPlane;

  /**
   * Configuration value for vertical error. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final double verticalError;

  /**
   * Configuration value for robot distance sq. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  final double robotDistanceSq;

  /**
   * Creates a drag shot planner candidate instance with the dependencies and tuning values used by
   * this Repulsor component.
   *
   * @param shooterPosition value used by this operation.
   * @param shooterYawRad value used by this operation.
   * @param speed value used by this operation.
   * @param angleRad value used by this operation.
   * @param timeToPlane value used by this operation.
   * @param verticalError value used by this operation.
   * @param robotDistanceSq value used by this operation.
   */
  DragShotPlannerCandidate(
      Translation2d shooterPosition,
      double shooterYawRad,
      double speed,
      double angleRad,
      double timeToPlane,
      double verticalError,
      double robotDistanceSq) {
    this.shooterPosition = shooterPosition;
    this.shooterYawRad = shooterYawRad;
    this.speed = speed;
    this.angleRad = angleRad;
    this.timeToPlane = timeToPlane;
    this.verticalError = verticalError;
    this.robotDistanceSq = robotDistanceSq;
  }

  /**
   * Returns the is better candidate value maintained by this Repulsor component.
   *
   * @param best value used by this operation.
   * @param next value used by this operation.
   * @param style value used by this operation.
   * @return value produced by this operation.
   */
  static boolean isBetterCandidate(
      DragShotPlannerCandidate best, DragShotPlannerCandidate next, Constraints.ShotStyle style) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.isBetterCandidate");
    try {
      if (best == null) {
        return true;
      }

      if (next.robotDistanceSq < best.robotDistanceSq - 1e-9) {
        return true;
      }
      if (best.robotDistanceSq < next.robotDistanceSq - 1e-9) {
        return false;
      }

      if (style == Constraints.ShotStyle.DIRECT || style == Constraints.ShotStyle.ARC) {
        double angleBest = Math.abs(best.angleRad);
        double angleNext = Math.abs(next.angleRad);
        double angleEps = 1e-3;
        if (style == Constraints.ShotStyle.DIRECT) {
          if (angleNext < angleBest - angleEps) {
            return true;
          }
          if (angleBest < angleNext - angleEps) {
            return false;
          }
        } else {
          if (angleNext > angleBest + angleEps) {
            return true;
          }
          if (angleBest > angleNext + angleEps) {
            return false;
          }
        }
      }

      if (next.speed < best.speed - DragShotPlannerConstants.EPS) {
        return true;
      }
      if (best.speed < next.speed - DragShotPlannerConstants.EPS) {
        return false;
      }

      return next.verticalError < best.verticalError - DragShotPlannerConstants.EPS;
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }
}
