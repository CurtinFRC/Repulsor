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
 * Provides attractor obstacle functionality for the Repulsor field-obstacle model used by the
 * repulsor planner. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class AttractorObstacle extends Obstacle {
  /**
   * Configuration value for center. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Translation2d center;

  /**
   * Configuration value for max range. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double maxRange;

  /**
   * Configuration value for soften. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double soften;

  /**
   * Configuration value for waypoint. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final boolean waypoint;

  /**
   * Returns the attractor obstacle value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param strength value used by this operation.
   * @param maxRange value used by this operation.
   * @param waypoint value used by this operation.
   */
  public AttractorObstacle(
      Translation2d center, double strength, double maxRange, boolean waypoint) {
    this(center, strength, maxRange, 0.18, waypoint);
  }

  /**
   * Returns the attractor obstacle value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param strength value used by this operation.
   * @param maxRange value used by this operation.
   * @param soften value used by this operation.
   * @param waypoint value used by this operation.
   */
  public AttractorObstacle(
      Translation2d center, double strength, double maxRange, double soften, boolean waypoint) {
    super(strength, true);
    this.center = center;
    this.maxRange = Math.max(0.0, maxRange);
    this.soften = Math.max(1e-6, soften);
    this.waypoint = waypoint;
  }

  /**
   * Returns the get force at position value maintained by this Repulsor component.
   *
   * @param position value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    double dist = position.getDistance(center);
    if (dist < EPS || dist > maxRange) return new Force();

    double magBase = strength / (soften + dist * dist);
    double magFalloff = strength / (soften + maxRange * maxRange);
    double mag = Math.max(magBase - magFalloff, 0.0);
    if (mag < EPS) return new Force();

    Translation2d toward = center.minus(position);
    if (toward.getNorm() < EPS) return new Force();

    return new Force(mag, toward.getAngle());
  }

  /**
   * Returns the intersects rectangle value maintained by this Repulsor component.
   *
   * @param rectCorners value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    return false;
  }
}
