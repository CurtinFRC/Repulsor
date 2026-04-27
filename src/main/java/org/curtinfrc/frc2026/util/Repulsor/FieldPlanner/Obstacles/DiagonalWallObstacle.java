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
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

/**
 * Provides diagonal wall obstacle functionality for the Repulsor field-obstacle model used by the
 * repulsor planner. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class DiagonalWallObstacle extends Obstacle {
  /**
   * Configuration value for a. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Translation2d a;

  /**
   * Configuration value for b. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Translation2d b;

  /**
   * Configuration value for max range. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double maxRange;

  /**
   * Returns the diagonal wall obstacle value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param strength value used by this operation.
   * @param maxRange value used by this operation.
   */
  public DiagonalWallObstacle(Translation2d a, Translation2d b, double strength, double maxRange) {
    super(strength, true);
    this.a = a;
    this.b = b;
    this.maxRange = maxRange;
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
    double dist = distanceFromPointToSegment(position, a, b);
    if (dist < EPS || dist > maxRange) return new Force();

    double mag = distToForceMag(dist, maxRange);

    Translation2d closest = closestPoint(position);
    Translation2d away = position.minus(closest);
    if (away.getNorm() < EPS) return new Force();

    Translation2d vec = new Translation2d(mag, away.getAngle());
    return new Force(vec.getNorm(), vec.getAngle());
  }

  private Translation2d closestPoint(Translation2d p) {
    Translation2d ab = b.minus(a);
    double abLenSq = ab.getNorm() * ab.getNorm();
    if (abLenSq < EPS) return a;
    double t = Math.max(0, Math.min(1, dot(p.minus(a), ab) / abLenSq));
    return a.plus(ab.times(t));
  }

  /**
   * Returns the intersects rectangle value maintained by this Repulsor component.
   *
   * @param rectCorners value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    for (Translation2d corner : rectCorners) {
      if (distanceFromPointToSegment(corner, a, b) < 0.1) return true;
    }
    if (FieldPlanner.isPointInPolygon(a, rectCorners)) return true;
    if (FieldPlanner.isPointInPolygon(b, rectCorners)) return true;
    return false;
  }
}
