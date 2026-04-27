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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

/**
 * Provides snowman obstacle functionality for the Repulsor field-obstacle model used by the
 * repulsor planner. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class SnowmanObstacle extends Obstacle {
  /**
   * Configuration value for loc. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public Translation2d loc;

  /**
   * Configuration value for radius. Distances use meters in WPILib field coordinates and should be
   * treated as tunable when sourced from profiles.
   */
  public double radius = 0.5;

  /**
   * Returns the snowman obstacle value maintained by this Repulsor component.
   *
   * @param loc value used by this operation.
   * @param strength value used by this operation.
   * @param radius value used by this operation.
   * @param positive value used by this operation.
   */
  public SnowmanObstacle(Translation2d loc, double strength, double radius, boolean positive) {
    super(strength, positive);
    this.loc = loc;
    this.radius = radius;
  }

  /**
   * Returns the get force at position value maintained by this Repulsor component.
   *
   * @param position value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    var targetToLoc = loc.minus(target);
    var targetToLocAngle = angleOr(targetToLoc, Rotation2d.kZero);

    var sidewaysCircle = new Translation2d(1, targetToLocAngle).plus(loc);

    var distLoc = Math.max(EPS, loc.getDistance(position));
    var sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position));
    var outwardsMag = distToForceMag(Math.max(0.01, distLoc - radius));

    var away = position.minus(loc);
    var awayAngle = angleOr(away, targetToLocAngle);

    var sidewaysTheta =
        target.minus(position).getAngle().minus(angleOr(position.minus(sidewaysCircle), awayAngle));
    double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
    var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);

    var combined =
        plus(new Translation2d(sideways, sidewaysAngle), new Translation2d(outwardsMag, awayAngle));
    if (combined.getNorm() < EPS) return new Force();
    return new Force(combined.getNorm(), combined.getAngle());
  }

  /**
   * Returns the intersects rectangle value maintained by this Repulsor component.
   *
   * @param rectCorners value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;
    for (Translation2d corner : rectCorners) if (corner.getDistance(loc) < radius) return true;
    for (int i = 0; i < rectCorners.length; i++) {
      Translation2d a = rectCorners[i];
      Translation2d b = rectCorners[(i + 1) % rectCorners.length];
      if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < radius) return true;
    }
    return false;
  }
}
