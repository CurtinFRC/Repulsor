package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.lang.reflect.Field;
import java.util.ArrayDeque;
import org.junit.jupiter.api.Test;

class VibrationTrackerProjectionTest {
  private static final double DT = 0.02;

  private ReactiveBypassConfig cfg() {
    ReactiveBypassConfig cfg = new ReactiveBypassConfig();
    cfg.vibWindowS = 1.0;
    return cfg;
  }

  @Test
  void pureRotationProducesNearZeroDisplacementEvidence() throws Exception {
    ReactiveBypassVibrationTracker tracker = new ReactiveBypassVibrationTracker();
    ReactiveBypassConfig cfg = cfg();
    cfg.vibMaxDirFlips = 0;

    Translation2d fixed = new Translation2d(3.0, 2.0);
    for (int i = 0; i < 60; i++) {
      tracker.feedWindow(
          new Pose2d(fixed, Rotation2d.fromRadians(i * 0.1)),
          Rotation2d.fromRadians(i * 0.1),
          DT,
          cfg);
    }

    assertEquals(0.0, projectedS(tracker, "sPara"), 1e-9);
    assertEquals(0.0, projectedS(tracker, "sPerp"), 1e-9);
    assertTrue(
        tracker.isVibrating(cfg), "rotation-only motion must not accumulate displacement evidence");
    assertTrue(tracker.isForwardStuck(cfg, 1.0));
  }

  @Test
  void straightLineMotionProjectsAlongHeadingFrame() throws Exception {
    ReactiveBypassVibrationTracker tracker = new ReactiveBypassVibrationTracker();
    ReactiveBypassConfig cfg = cfg();

    for (int i = 0; i < 60; i++) {
      double x = 1.0 + i * 0.01;
      tracker.feedWindow(
          new Pose2d(new Translation2d(x, 1.0), Rotation2d.kZero), Rotation2d.kZero, DT, cfg);
    }

    assertEquals(0.59, projectedS(tracker, "sPara"), 1e-9);
    assertEquals(0.0, projectedS(tracker, "sPerp"), 1e-9);
    assertFalse(tracker.isForwardStuck(cfg, 1.0));

    cfg.vibMaxDirFlips = 0;
    assertFalse(tracker.isVibrating(cfg));
  }

  @Test
  void lateralMotionAccumulatesPerpNotPara() throws Exception {
    ReactiveBypassVibrationTracker tracker = new ReactiveBypassVibrationTracker();
    ReactiveBypassConfig cfg = cfg();

    for (int i = 0; i < 60; i++) {
      double y = 1.0 + i * 0.01;
      tracker.feedWindow(
          new Pose2d(new Translation2d(1.0, y), Rotation2d.kCCW_90deg),
          Rotation2d.kCCW_90deg,
          DT,
          cfg);
    }

    assertEquals(0.59, projectedS(tracker, "sPara"), 1e-9);
    assertEquals(0.0, projectedS(tracker, "sPerp"), 1e-9);
  }

  @Test
  void oscillationAlongHeadingStillCountsFlips() {
    ReactiveBypassVibrationTracker tracker = new ReactiveBypassVibrationTracker();
    ReactiveBypassConfig cfg = cfg();

    for (int i = 0; i < 30; i++) {
      double x = i % 2 == 0 ? 2.005 : 1.995;
      tracker.feedWindow(
          new Pose2d(new Translation2d(x, 2.0), Rotation2d.kZero), Rotation2d.kZero, DT, cfg);
    }

    assertTrue(tracker.isVibrating(cfg), "small oscillation must still register as vibration");
  }

  private static double projectedS(ReactiveBypassVibrationTracker tracker, String fieldName)
      throws Exception {
    Field dequeField = ReactiveBypassVibrationTracker.class.getDeclaredField("vib");
    dequeField.setAccessible(true);
    @SuppressWarnings("unchecked")
    ArrayDeque<Object> deque = (ArrayDeque<Object>) dequeField.get(tracker);
    Object last = deque.getLast();
    Field value = last.getClass().getDeclaredField(fieldName);
    value.setAccessible(true);
    return value.getDouble(last);
  }
}
