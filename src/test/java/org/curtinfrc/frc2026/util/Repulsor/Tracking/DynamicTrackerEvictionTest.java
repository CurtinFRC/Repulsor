package org.curtinfrc.frc2026.util.Repulsor.Tracking;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.junit.jupiter.api.Test;

class DynamicTrackerEvictionTest {
  @Test
  void ingestEvictsIdsSilentBeyondTwoSeconds() {
    FieldTrackerDynamicTracker tracker = new FieldTrackerDynamicTracker();

    long t0 = System.nanoTime() - 10_000_000_000L;
    Pose3d poseA = new Pose3d(1.0, 2.0, 0.0, new Rotation3d());
    Pose3d poseB = new Pose3d(3.0, 4.0, 0.0, new Rotation3d());
    Pose3d poseC = new Pose3d(5.0, 6.0, 0.0, new Rotation3d());

    tracker.ingestTracked("fuel-a", "fuel", poseA, t0);
    tracker.ingestTracked("fuel-stale", "fuel", poseB, t0);

    long laterNs = t0 + 3_000_000_000L;
    tracker.ingestTracked("fuel-c", "fuel", poseC, laterNs);

    List<DynamicObject> dyn = tracker.snapshotDynamics();

    assertEquals(1, dyn.size());
    assertEquals("fuel-c", dyn.get(0).id);
    assertTrue(trackedIdsAbsent(dyn, "fuel-a", "fuel-stale"));
  }

  @Test
  void refreshedIdSurvivesEviction() {
    FieldTrackerDynamicTracker tracker = new FieldTrackerDynamicTracker();

    long t0 = System.nanoTime() - 10_000_000_000L;
    tracker.ingestTracked("fuel-live", "fuel", new Pose3d(1.0, 1.0, 0.0, new Rotation3d()), t0);

    for (int i = 1; i <= 4; i++) {
      tracker.ingestTracked(
          "fuel-live",
          "fuel",
          new Pose3d(1.0 + i * 0.1, 1.0, 0.0, new Rotation3d()),
          t0 + i * 900_000_000L);
      List<DynamicObject> dyn = tracker.snapshotDynamics();
      assertEquals(1, dyn.size());
      assertEquals("fuel-live", dyn.get(0).id);
    }
  }

  private static boolean trackedIdsAbsent(List<DynamicObject> dyn, String... ids) {
    for (DynamicObject o : dyn) {
      for (String id : ids) {
        if (o.id.equals(id)) return false;
      }
    }
    return true;
  }
}
