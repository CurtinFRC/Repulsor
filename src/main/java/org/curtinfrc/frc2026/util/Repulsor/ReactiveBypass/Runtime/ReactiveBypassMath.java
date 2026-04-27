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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides reactive bypass math functionality for the Repulsor runtime helper layer shared by
 * behaviours and planners. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
final class ReactiveBypassMath {
  private ReactiveBypassMath() {}

  /**
   * Returns the side of value maintained by this Repulsor component.
   *
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @param heading value used by this operation.
   * @param target value used by this operation.
   * @return value produced by this operation.
   */
  static int sideOf(Pose2d pose, Rotation2d heading, Translation2d target) {
    Translation2d to = target.minus(pose.getTranslation());
    double d = radDiff(heading.getRadians(), to.getAngle().getRadians());
    if (Math.abs(d) < 1e-6) return 0;
    return d > 0 ? +1 : -1;
  }

  /**
   * Returns the rad diff value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  static double radDiff(double a, double b) {
    double d = a - b;
    while (d > Math.PI) d -= 2 * Math.PI;
    while (d < -Math.PI) d += 2 * Math.PI;
    return d;
  }

  /**
   * Returns the clamp value maintained by this Repulsor component.
   *
   * @param v value used by this operation.
   * @param lo value used by this operation.
   * @param hi value used by this operation.
   * @return value produced by this operation.
   */
  static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  /**
   * Returns the clamp01 value maintained by this Repulsor component.
   *
   * @param v value used by this operation.
   * @return value produced by this operation.
   */
  static double clamp01(double v) {
    return clamp(v, 0.0, 1.0);
  }

  /**
   * Returns the lerp value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param t value used by this operation.
   * @return value produced by this operation.
   */
  static double lerp(double a, double b, double t) {
    return a + (b - a) * clamp01(t);
  }

  /**
   * Returns the heading of value maintained by this Repulsor component.
   *
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @return value produced by this operation.
   */
  static Rotation2d headingOf(Pose2d pose) {
    return pose.getRotation();
  }

  /**
   * Returns the project along value maintained by this Repulsor component.
   *
   * @param origin value used by this operation.
   * @param dir value used by this operation.
   * @param point value used by this operation.
   * @return value produced by this operation.
   */
  static double projectAlong(Translation2d origin, Rotation2d dir, Translation2d point) {
    Translation2d rel = point.minus(origin);
    return rel.getX() * Math.cos(dir.getRadians()) + rel.getY() * Math.sin(dir.getRadians());
  }
}
