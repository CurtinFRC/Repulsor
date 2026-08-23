package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class HoldTimeClampTest {
  private static final double STICKY_NEAR_DIST_M = 1.40;
  private static final double STICKY_FAR_DIST_M = 4.80;
  private static final double EPS = 1e-9;

  @Test
  void holdSForDistStaysWithinDistScaledBand() {
    double[] samples = {0.0, 0.5, 1.40, 2.25, 3.10, 4.80, 12.0};

    for (double d : samples) {
      double t =
          FieldTrackerCollectObjectiveMath.clamp01(
              (d - STICKY_NEAR_DIST_M) / (STICKY_FAR_DIST_M - STICKY_NEAR_DIST_M));
      double min = FieldTrackerCollectObjectiveMath.lerp(0.70, 0.25, t);
      double max = FieldTrackerCollectObjectiveMath.lerp(3.80, 1.80, t);
      double hold = FieldTrackerCollectObjectiveMath.holdSForDist(d);

      assertTrue(
          hold >= min - EPS && hold <= max + EPS,
          "hold " + hold + " outside [" + min + ", " + max + "] for d=" + d);
    }
  }

  @Test
  void holdSForDistDecreasesAsTargetMovesFartherAway() {
    double previous = FieldTrackerCollectObjectiveMath.holdSForDist(1.40);

    assertTrue(previous > 2.0, "close targets should hold longer than the far bound");

    for (double d = 1.40; d <= 4.800001; d += 0.34) {
      double hold = FieldTrackerCollectObjectiveMath.holdSForDist(d);
      assertTrue(
          hold <= previous + EPS,
          "hold should be monotonically non-increasing with distance at d=" + d);
      previous = hold;
    }

    assertTrue(
        FieldTrackerCollectObjectiveMath.holdSForDist(1.40)
            > FieldTrackerCollectObjectiveMath.holdSForDist(4.80),
        "close targets must hold strictly longer than far targets");
  }
}
