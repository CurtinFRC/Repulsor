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

package org.curtinfrc.frc2026.util.Repulsor.Fields;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Immutable data record for field geometry values passed through the Repulsor field/profile
 * definition layer used to tune Repulsor for a specific game. Use this type from robot code, field
 * profiles, or tests when integrating the corresponding Repulsor subsystem. Coordinates are
 * field-relative unless a method documents robot-relative motion.
 *
 * @param lengthMeters component of the field geometry model
 * @param widthMeters record component for the field geometry snapshot
 */
public record FieldGeometry(double lengthMeters, double widthMeters) {
  public FieldGeometry {
    if (!Double.isFinite(lengthMeters) || lengthMeters <= 0.0) {
      throw new IllegalArgumentException("lengthMeters must be finite and > 0");
    }
    if (!Double.isFinite(widthMeters) || widthMeters <= 0.0) {
      throw new IllegalArgumentException("widthMeters must be finite and > 0");
    }
  }

  /**
   * Returns the center value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Translation2d center() {
    return new Translation2d(lengthMeters * 0.5, widthMeters * 0.5);
  }

  /**
   * Returns the diagonal meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double diagonalMeters() {
    return Math.hypot(lengthMeters, widthMeters);
  }

  /**
   * Returns the contains value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @return value produced by this operation.
   */
  public boolean contains(Translation2d point) {
    if (point == null) return false;
    double x = point.getX();
    double y = point.getY();
    return Double.isFinite(x)
        && Double.isFinite(y)
        && x >= 0.0
        && x <= lengthMeters
        && y >= 0.0
        && y <= widthMeters;
  }

  /**
   * Returns the clamp value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param marginMeters distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public Translation2d clamp(Translation2d point, double marginMeters) {
    if (point == null) return center();
    double margin = MathUtil.clamp(marginMeters, 0.0, Math.min(lengthMeters, widthMeters) * 0.5);
    return new Translation2d(
        MathUtil.clamp(point.getX(), margin, lengthMeters - margin),
        MathUtil.clamp(point.getY(), margin, widthMeters - margin));
  }
}
