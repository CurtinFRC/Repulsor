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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Internal.NearestPoint;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Internal._SpatialIndex;

/**
 * Provides field tracker collect objective math functionality for the Repulsor collection objective
 * runtime for tracked field resources. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class FieldTrackerCollectObjectiveMath {
  private static final double STICKY_SAME_M = 0.45;
  private static final double SWITCH_APPROACH_COS_MIN = Math.cos(Math.toRadians(78.0));
  private static final double STICKY_NEAR_DIST_M = 1.40;
  private static final double STICKY_FAR_DIST_M = 4.80;
  private static final double CANONICAL_SNAP_M = 0.45;
  private static final double HUB_FRONT_TRAP_X_HALF_M = 0.78;
  private static final double HUB_FRONT_TRAP_CENTERLINE_HALF_Y_M = 1.00;

  private FieldTrackerCollectObjectiveMath() {}

  /**
   * Returns the sticky same value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  public static boolean stickySame(Translation2d a, Translation2d b) {
    return samePoint(a, b, STICKY_SAME_M);
  }

  /**
   * Returns the sticky switched value maintained by this Repulsor component.
   *
   * @param prev value used by this operation.
   * @param next value used by this operation.
   * @return value produced by this operation.
   */
  public static boolean stickySwitched(Translation2d prev, Translation2d next) {
    if (prev == null && next != null) return true;
    if (prev == null || next == null) return false;
    return !stickySame(prev, next);
  }

  /**
   * Returns the canonicalize collect point value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param setpoints value used by this operation.
   * @return value produced by this operation.
   */
  public static Translation2d canonicalizeCollectPoint(Translation2d p, Translation2d[] setpoints) {
    if (p == null || setpoints == null || setpoints.length == 0) return p;

    Translation2d best = null;
    double bestD2 = Double.POSITIVE_INFINITY;

    for (Translation2d s : setpoints) {
      if (s == null) continue;
      double dx = p.getX() - s.getX();
      double dy = p.getY() - s.getY();
      double d2 = dx * dx + dy * dy;
      if (d2 < bestD2) {
        bestD2 = d2;
        best = s;
      }
    }

    if (best == null) return p;
    double snap2 = CANONICAL_SNAP_M * CANONICAL_SNAP_M;
    return bestD2 <= snap2 ? best : p;
  }

  /**
   * Returns the side sign x value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public static int sideSignX(double x) {
    return sideSignXBand(x, 0.10);
  }

  /**
   * Returns the side sign xband value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @param band value used by this operation.
   * @return value produced by this operation.
   */
  public static int sideSignXBand(double x, double band) {
    double mid = Constants.FIELD_LENGTH * 0.5;
    double b = Math.max(0.0, band);
    if (x < mid - b) return -1;
    if (x > mid + b) return 1;
    return 0;
  }

  /**
   * Returns the side sign value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return value produced by this operation.
   */
  public static int sideSign(Translation2d p) {
    if (p == null) return 0;
    return sideSignX(p.getX());
  }

  /**
   * Returns the same point value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param tol value used by this operation.
   * @return value produced by this operation.
   */
  static boolean samePoint(Translation2d a, Translation2d b, double tol) {
    if (a == null || b == null) return false;
    return a.getDistance(b) <= tol;
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
   * Returns the unit value maintained by this Repulsor component.
   *
   * @param v value used by this operation.
   * @return value produced by this operation.
   */
  public static Translation2d unit(Translation2d v) {
    double n = v.getNorm();
    if (n <= 1e-9) return new Translation2d(1.0, 0.0);
    return v.div(n);
  }

  /**
   * Returns the moved enough value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param minMove value used by this operation.
   * @return value produced by this operation.
   */
  static boolean movedEnough(Translation2d a, Translation2d b, double minMove) {
    if (a == null || b == null) return true;
    return a.getDistance(b) >= minMove;
  }

  /**
   * Returns the violates approach value maintained by this Repulsor component.
   *
   * @param approachHat value used by this operation.
   * @param toCandHat value used by this operation.
   * @return value produced by this operation.
   */
  static boolean violatesApproach(Translation2d approachHat, Translation2d toCandHat) {
    if (approachHat == null || toCandHat == null) return false;
    return dot(approachHat, toCandHat) < SWITCH_APPROACH_COS_MIN;
  }

  /**
   * Returns the unit or default value maintained by this Repulsor component.
   *
   * @param v value used by this operation.
   * @param def value used by this operation.
   * @return value produced by this operation.
   */
  public static Translation2d unitOrDefault(Translation2d v, Translation2d def) {
    double n = v.getNorm();
    if (n <= 1e-9) return def;
    return v.div(n);
  }

  /**
   * Computes the nearest point to value for the current Repulsor planning state.
   *
   * @param q value used by this operation.
   * @param pts value used by this operation.
   * @return nearest point result for nearest point to.
   */
  public static NearestPoint nearestPointTo(Translation2d q, Translation2d[] pts) {
    if (q == null || pts == null || pts.length == 0)
      return new NearestPoint(null, Double.POSITIVE_INFINITY);
    Translation2d best = null;
    double bestD = Double.POSITIVE_INFINITY;
    for (Translation2d p : pts) {
      if (p == null) continue;
      double d = q.getDistance(p);
      if (d < bestD) {
        bestD = d;
        best = p;
      }
    }
    return new NearestPoint(best, bestD);
  }

  /**
   * Returns the dist2 value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  static double dist2(Translation2d a, Translation2d b) {
    double dx = a.getX() - b.getX();
    double dy = a.getY() - b.getY();
    return dx * dx + dy * dy;
  }

  /**
   * Returns the center penalty value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return value produced by this operation.
   */
  static double centerPenalty(Translation2d p) {
    double dx = p.getX() - (Constants.FIELD_LENGTH * 0.5);
    double dy = p.getY() - (Constants.FIELD_WIDTH * 0.5);
    return Math.sqrt(dx * dx + dy * dy);
  }

  /**
   * Returns the build index value maintained by this Repulsor component.
   *
   * @param pts value used by this operation.
   * @param cellM value used by this operation.
   * @return spatial index result for build index.
   */
  static _SpatialIndex buildIndex(Translation2d[] pts, double cellM) {
    _SpatialIndex idx = new _SpatialIndex(cellM);
    for (Translation2d p : pts) {
      if (p != null) idx.add(p);
    }
    return idx;
  }

  /**
   * Returns the perp value maintained by this Repulsor component.
   *
   * @param v value used by this operation.
   * @return value produced by this operation.
   */
  public static Translation2d perp(Translation2d v) {
    return new Translation2d(-v.getY(), v.getX());
  }

  /**
   * Returns the now sfrom ns value maintained by this Repulsor component.
   *
   * @param ns value used by this operation.
   * @return value produced by this operation.
   */
  public static double nowSFromNs(long ns) {
    return ns / 1e9;
  }

  /**
   * Returns the hold sfor dist value maintained by this Repulsor component.
   *
   * @param d value used by this operation.
   * @return value produced by this operation.
   */
  public static double holdSForDist(double d) {
    double t = clamp01((d - STICKY_NEAR_DIST_M) / (STICKY_FAR_DIST_M - STICKY_NEAR_DIST_M));
    double min = lerp(0.70, 0.25, t);
    double max = lerp(3.80, 1.80, t);
    return Math.max(min, Math.min(max, lerp(max, min, 0.5)));
  }

  /**
   * Returns the switch margin for dist value maintained by this Repulsor component.
   *
   * @param d value used by this operation.
   * @return value produced by this operation.
   */
  public static double switchMarginForDist(double d) {
    double t = clamp01((d - STICKY_NEAR_DIST_M) / (STICKY_FAR_DIST_M - STICKY_NEAR_DIST_M));
    return lerp(1.45, 0.50, t);
  }

  /**
   * Returns the lerp value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param t value used by this operation.
   * @return value produced by this operation.
   */
  public static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  /**
   * Returns the clamp01 value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public static double clamp01(double x) {
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
  }

  /**
   * Returns the is hub front trap point value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param leftBandX1 value used by this operation.
   * @param rightBandX0 value used by this operation.
   * @return value produced by this operation.
   */
  public static boolean isHubFrontTrapPoint(
      Translation2d p, double leftBandX1, double rightBandX0) {
    if (p == null) return false;
    double dy = Math.abs(p.getY() - (Constants.FIELD_WIDTH * 0.5));
    if (dy > HUB_FRONT_TRAP_CENTERLINE_HALF_Y_M) return false;
    boolean nearLeftInner = Math.abs(p.getX() - leftBandX1) <= HUB_FRONT_TRAP_X_HALF_M;
    boolean nearRightInner = Math.abs(p.getX() - rightBandX0) <= HUB_FRONT_TRAP_X_HALF_M;
    return nearLeftInner || nearRightInner;
  }
}
