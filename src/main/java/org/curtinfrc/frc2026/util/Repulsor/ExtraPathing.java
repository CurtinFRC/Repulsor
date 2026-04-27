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

package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers.ExtraPathingBounceListener;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers.ExtraPathingClearPath;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers.ExtraPathingCollision;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

/**
 * Provides extra pathing functionality for the Repulsor core Repulsor coordination layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public class ExtraPathing {

  /**
   * Returns the rect corners value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param length value used by this operation.
   * @param width value used by this operation.
   * @return value produced by this operation.
   */
  public static Translation2d[] rectCorners(Translation2d center, double length, double width) {
    return ExtraPathingCollision.rectCorners(center, length, width);
  }

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
    return ExtraPathingCollision.robotIntersects(
        center, robotLengthMeters, robotWidthMeters, obstacles);
  }

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
    return ExtraPathingClearPath.isClearPath(
        topicRoot, start, goal, obstacles, robotLengthMeters, robotWidthMeters, publishSamples);
  }

  /**
   * Provides bounce listener functionality for the Repulsor core Repulsor coordination layer. Use
   * this type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public class BounceListener extends ExtraPathingBounceListener {
    /**
     * Returns the bounce listener value maintained by this Repulsor component.
     *
     * @param bounceDistanceThreshold value used by this operation.
     * @param bounceHistoryLimit value used by this operation.
     */
    public BounceListener(double bounceDistanceThreshold, int bounceHistoryLimit) {
      super(bounceDistanceThreshold, bounceHistoryLimit);
    }
  }
}
