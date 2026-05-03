package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;

/** Pure geometry helpers for rectangle-like obstacles. */
final class RectangleGeometry {
  private RectangleGeometry() {}

  static Translation2d[] corners(
      Translation2d center, double halfX, double halfY, Rotation2d rotation) {
    Translation2d[] local =
        new Translation2d[] {
          new Translation2d(-halfX, -halfY),
          new Translation2d(halfX, -halfY),
          new Translation2d(halfX, halfY),
          new Translation2d(-halfX, halfY)
        };
    Translation2d[] out = new Translation2d[4];
    Rotation2d rot = rotation == null ? Rotation2d.kZero : rotation;
    for (int i = 0; i < 4; i++) out[i] = local[i].rotateBy(rot).plus(center);
    return out;
  }

  static Translation2d[] expandedCorners(
      Translation2d center, double halfX, double halfY, Rotation2d rotation, double pad) {
    Translation2d[] corners = corners(center, halfX, halfY, rotation);
    Translation2d[] out = new Translation2d[corners.length];
    for (int i = 0; i < corners.length; i++) {
      Translation2d v = corners[i].minus(center);
      double n = Math.max(1e-9, v.getNorm());
      Translation2d u = v.div(n);
      out[i] = center.plus(u.times(n + Math.max(0.0, pad)));
    }
    return out;
  }

  static boolean intersectsRectangle(Translation2d[] a, Translation2d[] b) {
    if (a == null || b == null || a.length < 3 || b.length < 3) return false;
    for (int i = 0; i < a.length; i++) {
      Translation2d a0 = a[i];
      Translation2d a1 = a[(i + 1) % a.length];
      for (int j = 0; j < b.length; j++) {
        Translation2d b0 = b[j];
        Translation2d b1 = b[(j + 1) % b.length];
        if (segmentsIntersect(a0, a1, b0, b1)) return true;
      }
    }
    for (Translation2d p : b) if (FieldPlanner.isPointInPolygon(p, a)) return true;
    for (Translation2d p : a) if (FieldPlanner.isPointInPolygon(p, b)) return true;
    return false;
  }

  static boolean segmentIntersectsPolygon(Translation2d a, Translation2d b, Translation2d[] poly) {
    if (poly == null || poly.length < 3) return false;
    if (FieldPlanner.isPointInPolygon(a, poly) || FieldPlanner.isPointInPolygon(b, poly))
      return true;
    for (int i = 0; i < poly.length; i++) {
      Translation2d c = poly[i];
      Translation2d d = poly[(i + 1) % poly.length];
      if (segmentsIntersect(a, b, c, d)) return true;
    }
    return false;
  }

  static double minSampleDistanceToPolygon(Translation2d a, Translation2d b, Translation2d[] poly) {
    int n = 7;
    double best = Double.POSITIVE_INFINITY;
    for (int i = 0; i <= n; i++) {
      double t = (double) i / (double) n;
      Translation2d p = lerp(a, b, t);
      double d = pointDistanceToPolygonEdges(p, poly);
      if (FieldPlanner.isPointInPolygon(p, poly)) d = -d;
      if (d < best) best = d;
    }
    return best;
  }

  static double pointDistanceToPolygonEdges(Translation2d p, Translation2d[] poly) {
    double best = Double.POSITIVE_INFINITY;
    for (int i = 0; i < poly.length; i++) {
      Translation2d a = poly[i];
      Translation2d b = poly[(i + 1) % poly.length];
      double d = pointSegmentDistance(p, a, b);
      if (d < best) best = d;
    }
    return best;
  }

  static Translation2d lerp(Translation2d a, Translation2d b, double t) {
    return new Translation2d(
        a.getX() + (b.getX() - a.getX()) * t, a.getY() + (b.getY() - a.getY()) * t);
  }

  static double pointSegmentDistance(Translation2d p, Translation2d a, Translation2d b) {
    double ax = a.getX();
    double ay = a.getY();
    double bx = b.getX();
    double by = b.getY();
    double px = p.getX();
    double py = p.getY();

    double vx = bx - ax;
    double vy = by - ay;
    double wx = px - ax;
    double wy = py - ay;

    double vv = vx * vx + vy * vy;
    if (vv < 1e-12) return Math.hypot(px - ax, py - ay);

    double t = (wx * vx + wy * vy) / vv;
    t = Math.max(0.0, Math.min(1.0, t));
    double cx = ax + vx * t;
    double cy = ay + vy * t;
    return Math.hypot(px - cx, py - cy);
  }

  static boolean segmentsIntersect(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    double o1 = orient(a, b, c);
    double o2 = orient(a, b, d);
    double o3 = orient(c, d, a);
    double o4 = orient(c, d, b);

    if ((o1 > 0) != (o2 > 0) && (o3 > 0) != (o4 > 0)) return true;

    if (Math.abs(o1) < 1e-9 && onSegment(a, b, c)) return true;
    if (Math.abs(o2) < 1e-9 && onSegment(a, b, d)) return true;
    if (Math.abs(o3) < 1e-9 && onSegment(c, d, a)) return true;
    if (Math.abs(o4) < 1e-9 && onSegment(c, d, b)) return true;

    return false;
  }

  private static double orient(Translation2d a, Translation2d b, Translation2d c) {
    return (b.getX() - a.getX()) * (c.getY() - a.getY())
        - (b.getY() - a.getY()) * (c.getX() - a.getX());
  }

  private static boolean onSegment(Translation2d a, Translation2d b, Translation2d p) {
    return p.getX() >= Math.min(a.getX(), b.getX()) - 1e-9
        && p.getX() <= Math.max(a.getX(), b.getX()) + 1e-9
        && p.getY() >= Math.min(a.getY(), b.getY()) - 1e-9
        && p.getY() <= Math.max(a.getY(), b.getY()) + 1e-9;
  }
}
