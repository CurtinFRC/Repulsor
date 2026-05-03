package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class PredictiveFieldStateStateExtractionTest {
  @Test
  void trackStoreOwnsLegacyMapReferences() {
    PredictiveFieldStateOps ops = new PredictiveFieldStateOps();

    assertSame(ops.trackStore.allies(), ops.allyMap);
    assertSame(ops.trackStore.enemies(), ops.enemyMap);
  }

  @Test
  void collectStateSnapshotAndClearProvideEncapsulatedStateAccess() {
    PredictiveFieldStateOps ops = new PredictiveFieldStateOps();
    Translation2d target = new Translation2d(2.0, 3.0);
    ops.currentCollectTarget = target;
    ops.currentCollectScore = 4.5;
    ops.currentCollectUnits = 2.0;
    ops.collectProgressLastDist = 1.2;

    PredictiveCollectStateSnapshot snapshot = ops.collectStateSnapshot();

    assertSame(target, snapshot.currentCollectTarget());
    assertEquals(4.5, snapshot.currentCollectScore(), 1e-9);
    assertEquals(2.0, snapshot.currentCollectUnits(), 1e-9);

    ops.clearCollectState();

    PredictiveCollectStateSnapshot cleared = ops.collectStateSnapshot();
    assertEquals(-1e18, cleared.currentCollectScore(), 1e-9);
    assertTrue(Double.isInfinite(cleared.collectProgressLastDist()));
  }
}
