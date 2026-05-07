package org.curtinfrc.frc2026.util.Repulsor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.junit.jupiter.api.Test;

class RepulsorStrategyPresetRuntimeTest {
  @Test
  void appliesNamedStrategyPresetAcrossRuntimeSubsystems() {
    Repulsor repulsor =
        new Repulsor(
            new FakeDrive(),
            Repulsor.UsageType.kFullAuto,
            0.45,
            0.45,
            () -> false,
            new Rebuilt2026());

    List<String> names = repulsor.availableStrategyPresets();
    assertTrue(names.contains("fastCollect"));
    assertTrue(names.contains("safeCycle"));
    assertEquals("safeCycle", repulsor.currentStrategyPreset());

    assertTrue(repulsor.applyStrategyPreset("fastCollect"));

    assertEquals("fastCollect", repulsor.currentStrategyPreset());
    assertEquals(2, repulsor.autoPathRuntimeConfig().collectGoalUnits());
    assertFalse(repulsor.getFieldPlanner().getWaypointConfig().bandTransitionStagingEnabled());
    assertEquals(
        0.30, FieldTrackerCore.getInstance().collectPlannerTuning().switchCooldownSeconds(), 1e-9);
    assertEquals(
        0.12, FieldTrackerCore.getInstance().objectiveSelectionConfig().switchScoreMargin(), 1e-9);
  }

  @Test
  void missingPresetFallsBackToDefaultWithoutThrowing() {
    Repulsor repulsor =
        new Repulsor(
            new FakeDrive(),
            Repulsor.UsageType.kFullAuto,
            0.45,
            0.45,
            () -> false,
            new Rebuilt2026());

    assertTrue(repulsor.applyStrategyPreset("fastCollect"));
    assertFalse(repulsor.applyStrategyPreset("doesNotExist"));

    assertEquals("safeCycle", repulsor.currentStrategyPreset());
    assertEquals(
        0.90, FieldTrackerCore.getInstance().collectPlannerTuning().switchCooldownSeconds(), 1e-9);
  }

  private static final class FakeDrive extends DriveRepulsor {
    private final PIDController omega = new PIDController(1.0, 0.0, 0.0);

    @Override
    public void runVelocity(ChassisSpeeds speeds) {}

    @Override
    public Pose2d getPose() {
      return Pose2d.kZero;
    }

    @Override
    public PIDController getOmegaPID() {
      return omega;
    }
  }
}
