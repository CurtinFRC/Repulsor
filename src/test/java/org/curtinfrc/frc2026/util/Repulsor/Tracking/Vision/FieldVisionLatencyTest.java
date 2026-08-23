package org.curtinfrc.frc2026.util.Repulsor.Tracking.Vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldLayoutProvider;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.junit.jupiter.api.Test;

class FieldVisionLatencyTest {
  private static final double AGE_TOLERANCE_S = 0.05;

  private static final class CapturingTracker extends FieldTrackerCore {
    final List<Long> stampsNs = new ArrayList<>();

    CapturingTracker() {
      super(
          new FieldLayoutProvider() {
            @Override
            public GameElement[] build(FieldTrackerCore ft) {
              return new GameElement[0];
            }

            @Override
            public String gameName() {
              return "latency-test";
            }

            @Override
            public int gameYear() {
              return 2026;
            }

            @Override
            public FieldGeometry geometry() {
              return new FieldGeometry(16.5, 8.0);
            }
          });
    }

    @Override
    public void ingestTracked(String id, String type, Pose3d p, long nowNs) {
      stampsNs.add(nowNs);
    }
  }

  private static void publishFieldObject(String visionName) {
    var table = NetworkTableInstance.getDefault().getTable("FieldVision/" + visionName);
    table.getEntry("object_probe/frame").setString("field");
    table.getEntry("object_probe/type").setString("fuel");
    table.getEntry("object_probe/x").setDouble(4.0);
    table.getEntry("object_probe/y").setDouble(2.0);
    table.getEntry("object_probe/z").setDouble(0.3);
    table.getEntry("object_probe/yaw").setDouble(0.0);
  }

  @Test
  void zeroLatencyPreservesReceiptTimestampsAsObservationAgeZero() {
    CapturingTracker owner = new CapturingTracker();
    FieldVision vision =
        new FieldVision(owner, "zero-latency-" + Long.toUnsignedString(System.nanoTime()));
    publishFieldObject(vision.getName());

    long beforeNs = System.nanoTime();
    vision.update(new Pose2d());
    long afterNs = System.nanoTime();

    assertEquals(1, owner.stampsNs.size());
    long stampNs = owner.stampsNs.get(0);
    assertTrue(stampNs >= beforeNs);
    assertTrue(stampNs <= afterNs);
  }

  @Test
  void injectedLatencyBackdatesStampsByConfiguredAmount() {
    CapturingTracker owner = new CapturingTracker();
    FieldVision vision =
        new FieldVision(owner, "injected-latency-" + Long.toUnsignedString(System.nanoTime()));
    publishFieldObject(vision.getName());

    vision.setLatencySeconds(0.25);
    assertEquals(0.25, vision.getLatencySeconds(), 1e-9);

    vision.update(new Pose2d());
    long afterNs = System.nanoTime();

    assertEquals(1, owner.stampsNs.size());
    double observedAgeS = (afterNs - owner.stampsNs.get(0)) / 1e9;
    assertEquals(0.25, observedAgeS, AGE_TOLERANCE_S);
  }

  @Test
  void negativeAndNonFiniteLatencyClampToZero() {
    CapturingTracker owner = new CapturingTracker();
    FieldVision vision =
        new FieldVision(owner, "clamp-latency-" + Long.toUnsignedString(System.nanoTime()));
    publishFieldObject(vision.getName());

    vision.setLatencySeconds(-1.0);
    assertEquals(0.0, vision.getLatencySeconds(), 1e-9);
    vision.setLatencySeconds(Double.NaN);
    assertEquals(0.0, vision.getLatencySeconds(), 1e-9);

    long beforeNs = System.nanoTime();
    vision.update(new Pose2d());
    long afterNs = System.nanoTime();

    long stampNs = owner.stampsNs.get(0);
    assertTrue(stampNs >= beforeNs);
    assertTrue(stampNs <= afterNs);
  }
}
