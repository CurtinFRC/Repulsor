package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import org.junit.jupiter.api.Test;

class BypassInflationTest {
  private static final double EPS = 1e-9;

  private static Translation2d[] captureRect(
      ReactiveBypassConfig cfg, Pose2d pose, Rotation2d heading, double robotX, double robotY) {
    AtomicReference<Translation2d[]> captured = new AtomicReference<>();
    Function<Translation2d[], Boolean> probe =
        rect -> {
          captured.set(rect);
          return false;
        };
    ReactiveBypassProbing.robotTouchDynamic(cfg, pose, heading, robotX, robotY, probe);
    return captured.get();
  }

  @Test
  void probeRectangleEqualsBaseFullSizePlusTwoInflationPerAxisAtHeadingZero() {
    ReactiveBypassConfig cfg = new ReactiveBypassConfig();
    cfg.inflationMeters = 0.07;
    Pose2d pose = new Pose2d(new Translation2d(4.0, 3.0), Rotation2d.kZero);
    double robotX = 0.85;
    double robotY = 0.60;

    Translation2d[] rect = captureRect(cfg, pose, Rotation2d.kZero, robotX, robotY);

    assertEquals(robotX + 2.0 * cfg.inflationMeters, spanX(rect), EPS);
    assertEquals(robotY + 2.0 * cfg.inflationMeters, spanY(rect), EPS);
  }

  @Test
  void inflationAddsExactlyInflationMetersToEachSideOfTheBaseFootprint() {
    ReactiveBypassConfig cfg = new ReactiveBypassConfig();
    cfg.inflationMeters = 0.12;
    double centerX = 4.0;
    double centerY = 3.0;
    Pose2d pose = new Pose2d(new Translation2d(centerX, centerY), Rotation2d.kZero);
    double robotX = 0.85;
    double robotY = 0.60;

    Translation2d[] rect = captureRect(cfg, pose, Rotation2d.kZero, robotX, robotY);

    assertEquals(centerX - (robotX / 2.0 + cfg.inflationMeters), minX(rect), EPS);
    assertEquals(centerX + (robotX / 2.0 + cfg.inflationMeters), maxX(rect), EPS);
    assertEquals(centerY - (robotY / 2.0 + cfg.inflationMeters), minY(rect), EPS);
    assertEquals(centerY + (robotY / 2.0 + cfg.inflationMeters), maxY(rect), EPS);
  }

  private static double spanX(Translation2d[] rect) {
    return maxX(rect) - minX(rect);
  }

  private static double spanY(Translation2d[] rect) {
    return maxY(rect) - minY(rect);
  }

  private static double minX(Translation2d[] rect) {
    double min = Double.POSITIVE_INFINITY;
    for (Translation2d c : rect) min = Math.min(min, c.getX());
    return min;
  }

  private static double maxX(Translation2d[] rect) {
    double max = Double.NEGATIVE_INFINITY;
    for (Translation2d c : rect) max = Math.max(max, c.getX());
    return max;
  }

  private static double minY(Translation2d[] rect) {
    double min = Double.POSITIVE_INFINITY;
    for (Translation2d c : rect) min = Math.min(min, c.getY());
    return min;
  }

  private static double maxY(Translation2d[] rect) {
    double max = Double.NEGATIVE_INFINITY;
    for (Translation2d c : rect) max = Math.max(max, c.getY());
    return max;
  }
}
