package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class RectangleGeometryTest {
  @Test
  void cornersMatchAxisAlignedRectangle() {
    Translation2d[] corners =
        RectangleGeometry.corners(new Translation2d(2.0, 3.0), 1.0, 0.5, Rotation2d.kZero);

    assertEquals(1.0, corners[0].getX(), 1e-9);
    assertEquals(2.5, corners[0].getY(), 1e-9);
    assertEquals(3.0, corners[2].getX(), 1e-9);
    assertEquals(3.5, corners[2].getY(), 1e-9);
  }

  @Test
  void segmentIntersectionDetectsPolygonCrossing() {
    Translation2d[] poly =
        RectangleGeometry.corners(new Translation2d(2.0, 2.0), 0.5, 0.5, Rotation2d.kZero);

    assertTrue(
        RectangleGeometry.segmentIntersectsPolygon(
            new Translation2d(1.0, 2.0), new Translation2d(3.0, 2.0), poly));
  }
}
