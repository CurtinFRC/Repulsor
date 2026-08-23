package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class GamePieceLoaderNegativeCacheTest {
  private static final class StubGamePiece extends GamePiecePhysics {
    @Override
    public double massKg() {
      return 0.27;
    }

    @Override
    public double crossSectionAreaM2() {
      return 0.014;
    }

    @Override
    public double dragCoefficient() {
      return 0.95;
    }
  }

  @AfterEach
  void tearDown() {
    DragShotPlannerGamePieceLoader.resetForTest();
  }

  @Test
  void missingResourceIsResolvedOnlyOncePerJvm() {
    AtomicInteger attempts = new AtomicInteger();
    DragShotPlannerGamePieceLoader.setResolverForTest(
        id -> {
          attempts.incrementAndGet();
          throw new IllegalStateException("Missing game piece YAML: " + id);
        });

    IllegalStateException first =
        assertThrows(
            IllegalStateException.class,
            () -> DragShotPlannerGamePieceLoader.loadGamePieceFromDeployYaml("ghost-piece"));
    assertEquals(1, attempts.get());
    assertEquals("Missing game piece YAML: ghost-piece", first.getMessage());

    IllegalStateException second =
        assertThrows(
            IllegalStateException.class,
            () -> DragShotPlannerGamePieceLoader.loadGamePieceFromDeployYaml("ghost-piece"));
    assertEquals(1, attempts.get());
    assertEquals(first.getMessage(), second.getMessage());

    assertThrows(
        IllegalStateException.class,
        () -> DragShotPlannerGamePieceLoader.loadGamePieceFromDeployYaml("other-missing"));
    assertEquals(2, attempts.get());
  }

  @Test
  void successfulLoadIsCachedAcrossCalls() {
    AtomicInteger attempts = new AtomicInteger();
    StubGamePiece stub = new StubGamePiece();
    DragShotPlannerGamePieceLoader.setResolverForTest(
        id -> {
          attempts.incrementAndGet();
          return stub;
        });

    GamePiecePhysics first = DragShotPlannerGamePieceLoader.loadGamePieceFromDeployYaml("fuel");
    GamePiecePhysics second = DragShotPlannerGamePieceLoader.loadGamePieceFromDeployYaml("fuel");

    assertEquals(1, attempts.get());
    assertSame(stub, first);
    assertSame(stub, second);
  }

  @Test
  void blankIdsStillRejectWithoutTouchingResolver() {
    AtomicInteger attempts = new AtomicInteger();
    DragShotPlannerGamePieceLoader.setResolverForTest(
        id -> {
          attempts.incrementAndGet();
          throw new AssertionError("resolver must not run");
        });

    assertThrows(
        IllegalArgumentException.class,
        () -> DragShotPlannerGamePieceLoader.loadGamePieceFromDeployYaml(null));
    assertThrows(
        IllegalArgumentException.class,
        () -> DragShotPlannerGamePieceLoader.loadGamePieceFromDeployYaml(""));
    assertEquals(0, attempts.get());
  }
}
