package org.curtinfrc.frc2026.util.Repulsor.Tuning;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap;
import org.junit.jupiter.api.Test;

class DriveTuningHeatStepTest {
  private static final double DT = 0.02;

  @Test
  void slowDownStepIsVelocityTimesDtNotRawVelocity() {
    DriveTuningHeat tuning =
        new DriveTuningHeat(() -> new Pose2d(), (Heatmap) null).withBaseMaxSpeed(2.0);
    double step = tuning.baseStepMeters(10.0, true);
    assertTrue(step <= 2.0 * DT + 1e-9, "step was " + step);
    assertTrue(step > 0.0);
  }

  @Test
  void capFollowsRaisedBaseMaxSpeed() {
    DriveTuningHeat tuning =
        new DriveTuningHeat(() -> new Pose2d(), (Heatmap) null).withBaseMaxSpeed(8.0);
    double step = tuning.baseStepMeters(10.0, true);
    assertEquals(8.0 * DT, step, 1e-9);
  }

  @Test
  void missingHeatmapFallsBackToFullSpeed() {
    DriveTuningHeat tuning =
        new DriveTuningHeat(() -> new Pose2d(), (Heatmap) null).withBaseMaxSpeed(3.5);
    assertEquals(
        3.5,
        tuning.maxLinearSpeedMps(new Pose2d()),
        1e-9,
        "empty heatmap must scale speed by 1.0");
  }

  @Test
  void populatedHeatmapStillScalesSpeed() {
    Heatmap heat =
        Heatmap.builder()
            .block("slow", new edu.wpi.first.math.geometry.Translation2d(0, 0), 4.0, 4.0, 0.25)
            .build();
    DriveTuningHeat tuning = new DriveTuningHeat(() -> new Pose2d(), heat).withBaseMaxSpeed(4.0);
    assertEquals(1.0, tuning.maxLinearSpeedMps(new Pose2d()), 1e-9);
  }

  @Test
  void nearGoalBrakesToSmallSteps() {
    DriveTuningHeat tuning =
        new DriveTuningHeat(() -> new Pose2d(), (Heatmap) null).withBaseMaxSpeed(5.0);
    double step = tuning.baseStepMeters(0.005, true);
    assertTrue(step <= 0.02, "near-goal step was " + step);
  }

  @Test
  void nonSlowDownStepIsCappedByDt() {
    DriveTuningHeat tuning =
        new DriveTuningHeat(() -> new Pose2d(), (Heatmap) null).withBaseMaxSpeed(6.0);
    assertEquals(6.0 * DT, tuning.baseStepMeters(10.0, false), 1e-9);
    assertEquals(
        6.0 * DT,
        tuning.baseStepMeters(0.5, false),
        1e-9,
        "non-slowDown step is capped by dt even near goal");
  }
}
