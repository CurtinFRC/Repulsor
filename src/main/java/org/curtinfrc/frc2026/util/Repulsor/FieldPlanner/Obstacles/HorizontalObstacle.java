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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

/**
 * Provides horizontal obstacle functionality for the Repulsor field-obstacle model used by the
 * repulsor planner. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class HorizontalObstacle extends Obstacle {
  /**
   * Configuration value for y. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double y;

  /**
   * Returns the horizontal obstacle value maintained by this Repulsor component.
   *
   * @param y distance or field-coordinate value in meters.
   * @param strength value used by this operation.
   * @param positive value used by this operation.
   */
  public HorizontalObstacle(double y, double strength, boolean positive) {
    super(strength, positive);
    this.y = y;
  }

  /**
   * Returns the get force at position value maintained by this Repulsor component.
   *
   * @param position value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    return new Force(0, distToForceMag(y - position.getY(), 1));
  }

  /**
   * Returns the intersects rectangle value maintained by this Repulsor component.
   *
   * @param rectCorners value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    for (Translation2d a : rectCorners) if (Math.abs(a.getY() - y) < 0.1) return true;
    return false;
  }
}
