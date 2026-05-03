package org.curtinfrc.frc2026.util.Repulsor.Tracking.Internal;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class ObjectiveCacheTest {
  @Test
  void pointsAccessorIsDefensiveCopy() {
    ObjectiveCache cache = new ObjectiveCache();
    Translation2d[] points = new Translation2d[] {new Translation2d(1.0, 2.0)};

    cache.update(points, 42);
    points[0] = new Translation2d(9.0, 9.0);

    Translation2d[] snapshot = cache.points();
    snapshot[0] = new Translation2d(3.0, 3.0);

    assertNotSame(points, snapshot);
    assertEquals(1.0, cache.points()[0].getX(), 1e-9);
    assertEquals(2.0, cache.points()[0].getY(), 1e-9);
    assertEquals(42, cache.lastHash());
  }

  @Test
  void clearResetsCache() {
    ObjectiveCache cache = new ObjectiveCache();
    cache.update(new Translation2d[] {new Translation2d(1.0, 2.0)}, 42);

    cache.clear();

    assertEquals(0, cache.points().length);
    assertEquals(0, cache.lastHash());
  }
}
