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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides field planner geometry functionality for the Repulsor repulsor-field planner that
 * combines goals, obstacles, and force samples. Use this type from robot code, field profiles, or
 * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public final class FieldPlannerGeometry {
  private FieldPlannerGeometry() {}

  /**
   * Returns the clamp01 value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public static double clamp01(double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }

  /**
   * Returns the smooth01 value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public static double smooth01(double x) {
    x = clamp01(x);
    return x * x * (3.0 - 2.0 * x);
  }

  /**
   * Returns the segment intersects polygon outer value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param poly distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public static boolean segmentIntersectsPolygonOuter(
      Translation2d a, Translation2d b, Translation2d[] poly) {
    if (isPointInPolygon(a, poly) || isPointInPolygon(b, poly)) return true;
    for (int i = 0; i < poly.length; i++) {
      Translation2d c = poly[i];
      Translation2d d = poly[(i + 1) % poly.length];
      if (segmentsIntersectOuter(a, b, c, d)) return true;
    }
    return false;
  }

  private static double orientOuter(Translation2d a, Translation2d b, Translation2d c) {
    return (b.getX() - a.getX()) * (c.getY() - a.getY())
        - (b.getY() - a.getY()) * (c.getX() - a.getX());
  }

  private static boolean onSegOuter(Translation2d a, Translation2d b, Translation2d p) {
    return p.getX() >= Math.min(a.getX(), b.getX()) - 1e-9
        && p.getX() <= Math.max(a.getX(), b.getX()) + 1e-9
        && p.getY() >= Math.min(a.getY(), b.getY()) - 1e-9
        && p.getY() <= Math.max(a.getY(), b.getY()) + 1e-9;
  }

  private static boolean segmentsIntersectOuter(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    double o1 = orientOuter(a, b, c);
    double o2 = orientOuter(a, b, d);
    double o3 = orientOuter(c, d, a);
    double o4 = orientOuter(c, d, b);

    if ((o1 > 0) != (o2 > 0) && (o3 > 0) != (o4 > 0)) return true;

    if (Math.abs(o1) < 1e-9 && onSegOuter(a, b, c)) return true;
    if (Math.abs(o2) < 1e-9 && onSegOuter(a, b, d)) return true;
    if (Math.abs(o3) < 1e-9 && onSegOuter(c, d, a)) return true;
    if (Math.abs(o4) < 1e-9 && onSegOuter(c, d, b)) return true;

    return false;
  }

  /**
   * Returns the is point in polygon value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param polygon value used by this operation.
   * @return value produced by this operation.
   */
  public static boolean isPointInPolygon(Translation2d point, Translation2d[] polygon) {
    int crossings = 0;
    for (int i = 0; i < polygon.length; i++) {
      Translation2d a = polygon[i];
      Translation2d b = polygon[(i + 1) % polygon.length];
      boolean cond1 = (a.getY() > point.getY()) != (b.getY() > point.getY());
      double slope =
          (b.getX() - a.getX()) * (point.getY() - a.getY()) / (b.getY() - a.getY()) + a.getX();
      if (cond1 && point.getX() < slope) crossings++;
    }
    return (crossings % 2 == 1);
  }

  /**
   * Returns the dot value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  public static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  /**
   * Returns the distance from point to segment value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  public static double distanceFromPointToSegment(
      Translation2d p, Translation2d a, Translation2d b) {
    Translation2d ap = p.minus(a);
    Translation2d ab = b.minus(a);
    double abLenSquared = ab.getNorm() * ab.getNorm();
    if (abLenSquared == 0) return ap.getNorm();
    double t = Math.max(0, Math.min(1, dot(ap, ab) / abLenSquared));
    Translation2d projection = a.plus(ab.times(t));
    return p.getDistance(projection);
  }
}
