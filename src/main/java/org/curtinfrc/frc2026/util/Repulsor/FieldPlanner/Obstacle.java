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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Force;

/**
 * Provides obstacle functionality for the Repulsor repulsor-field planner that combines goals,
 * obstacles, and force samples. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public abstract class Obstacle {
  protected static final double EPS = 1e-9;

  /**
   * Configuration value for strength. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double strength = 1.0;

  /**
   * Configuration value for positive. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public boolean positive = true;

  /**
   * Returns the obstacle value maintained by this Repulsor component.
   *
   * @param strength value used by this operation.
   * @param positive value used by this operation.
   */
  public Obstacle(double strength, boolean positive) {
    this.strength = strength;
    this.positive = positive;
  }

  /**
   * Returns the get force at position value maintained by this Repulsor component.
   *
   * @param position value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  public abstract Force getForceAtPosition(Translation2d position, Translation2d target);

  protected double distToForceMag(double dist) {
    var forceMag = strength / (0.00001 + Math.abs(dist * dist));
    forceMag *= positive ? 1 : -1;
    return forceMag;
  }

  protected double distToForceMag(double dist, double falloff) {
    var original = strength / (0.00001 + Math.abs(dist * dist));
    var falloffMag = strength / (0.00001 + Math.abs(falloff * falloff));
    return Math.max(original - falloffMag, 0) * (positive ? 1 : -1);
  }

  protected static Rotation2d angleOr(Translation2d v, Rotation2d fallback) {
    return v.getNorm() > EPS ? v.getAngle() : fallback;
  }

  protected static Translation2d plus(Translation2d a, Translation2d b) {
    return new Translation2d(a.getX() + b.getX(), a.getY() + b.getY());
  }

  protected static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  protected static double distanceFromPointToSegment(
      Translation2d p, Translation2d a, Translation2d b) {
    Translation2d ap = p.minus(a);
    Translation2d ab = b.minus(a);
    double abLenSquared = ab.getNorm() * ab.getNorm();
    if (abLenSquared == 0) return ap.getNorm();
    double t = Math.max(0, Math.min(1, dot(ap, ab) / abLenSquared));
    Translation2d projection = a.plus(ab.times(t));
    return p.getDistance(projection);
  }

  /**
   * Returns the intersects rectangle value maintained by this Repulsor component.
   *
   * @param rectCorners value used by this operation.
   * @return value produced by this operation.
   */
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    return false;
  }
}
