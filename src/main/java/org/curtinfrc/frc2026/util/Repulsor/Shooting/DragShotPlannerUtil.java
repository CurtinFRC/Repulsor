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

/**
 * Provides drag shot planner util functionality for the Repulsor projectile and shot-planning
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
final class DragShotPlannerUtil {
  private DragShotPlannerUtil() {}

  /**
   * Runs close quietly in the Repulsor runtime.
   *
   * @param c value used by this operation.
   */
  static void closeQuietly(AutoCloseable c) {
    if (c == null) return;
    try {
      c.close();
    } catch (Exception ignored) {
    }
  }

  /**
   * Returns the estimate speed no drag value maintained by this Repulsor component.
   *
   * @param horizontalDistanceMeters distance or field-coordinate value in meters.
   * @param heightDeltaMeters distance or field-coordinate value in meters.
   * @param angleRad value used by this operation.
   * @return value produced by this operation.
   */
  static double estimateSpeedNoDrag(
      double horizontalDistanceMeters, double heightDeltaMeters, double angleRad) {
    double cos = Math.cos(angleRad);
    if (cos <= 1e-6) {
      return Double.NaN;
    }
    double sin = Math.sin(angleRad);
    double tan = sin / cos;
    double denom = horizontalDistanceMeters * tan - heightDeltaMeters;
    if (denom <= 1e-6) {
      return Double.NaN;
    }
    double v2 =
        9.81 * horizontalDistanceMeters * horizontalDistanceMeters / (2.0 * cos * cos * denom);
    if (v2 <= 0.0 || !Double.isFinite(v2)) {
      return Double.NaN;
    }
    return Math.sqrt(v2);
  }
}
