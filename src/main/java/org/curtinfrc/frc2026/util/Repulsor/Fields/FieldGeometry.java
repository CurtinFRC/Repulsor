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

public record FieldGeometry(double lengthMeters, double widthMeters) {
  public FieldGeometry {
    if (!Double.isFinite(lengthMeters) || lengthMeters <= 0.0) {
      throw new IllegalArgumentException("lengthMeters must be finite and > 0");
    }
    if (!Double.isFinite(widthMeters) || widthMeters <= 0.0) {
      throw new IllegalArgumentException("widthMeters must be finite and > 0");
    }
  }

  public Translation2d center() {
    return new Translation2d(lengthMeters * 0.5, widthMeters * 0.5);
  }

  public double diagonalMeters() {
    return Math.hypot(lengthMeters, widthMeters);
  }

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

  public Translation2d clamp(Translation2d point, double marginMeters) {
    if (point == null) return center();
    double margin = MathUtil.clamp(marginMeters, 0.0, Math.min(lengthMeters, widthMeters) * 0.5);
    return new Translation2d(
        MathUtil.clamp(point.getX(), margin, lengthMeters - margin),
        MathUtil.clamp(point.getY(), margin, widthMeters - margin));
  }
}
