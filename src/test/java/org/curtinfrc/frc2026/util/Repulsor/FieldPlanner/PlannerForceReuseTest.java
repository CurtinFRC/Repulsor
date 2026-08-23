package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadExecutionContext;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.junit.jupiter.api.Test;

class PlannerForceReuseTest {
  @Test
  void calculateProducesFiniteOutputForUnblockedStraightLineFixture() {
    FieldPlanner planner = new FieldPlanner();
    planner.setRequestedGoal(new Pose2d(6.0, 2.0, Rotation2d.kZero));

    RepulsorSample sample =
        OffloadExecutionContext.runWorker(
            () ->
                planner.calculate(
                    new Pose2d(2.0, 2.0, Rotation2d.kZero),
                    List.of(),
                    0.18,
                    0.18,
                    CategorySpec.kScore,
                    false,
                    0.0));

    assertNotNull(sample);
    assertNotNull(sample.goal());
    assertTrue(Double.isFinite(sample.vxMetersPerSecond()));
    assertTrue(Double.isFinite(sample.vyMetersPerSecond()));
    assertTrue(Double.isFinite(sample.omegaRadians()));
  }

  @Test
  void runtimeConfigExposesDocumentedDefaults() {
    FieldPlanner planner = new FieldPlanner();
    FieldPlannerRuntimeConfig config = planner.getRuntimeConfig();

    assertEquals(
        FieldPlannerRuntimeConfig.defaultRerouteCandidateRadiusMeters(),
        config.rerouteCandidateRadiusMeters(),
        1e-9);
    assertEquals(
        FieldPlannerRuntimeConfig.defaultRerouteCandidateCount(), config.rerouteCandidateCount());
    assertEquals(
        FieldPlannerRuntimeConfig.defaultFallbackAllianceWhenUnknown(),
        config.fallbackAllianceWhenUnknown());
  }

  @Test
  void runtimeConfigDefaultsMatchDocumentedValues() {
    FieldPlannerRuntimeConfig config = FieldPlannerRuntimeConfig.defaults();

    assertEquals(3.5, config.rerouteCandidateRadiusMeters(), 1e-9);
    assertEquals(8, config.rerouteCandidateCount());
    assertEquals(Alliance.kRed, config.fallbackAllianceWhenUnknown());
    assertTrue(config.globalFallbackEnabled());
    assertEquals(2.0, config.forceThroughGoalDistanceMeters(), 1e-9);
    assertEquals(0.7, config.forceThroughWallDistanceMeters(), 1e-9);
  }

  @Test
  void runtimeConfigLegacyConstructorKeepsNewFieldsAtDefaults() {
    FieldPlannerRuntimeConfig config = new FieldPlannerRuntimeConfig(false, null, -1.0, -1.0);

    assertFalse(config.globalFallbackEnabled());
    assertNotNull(config.globalFallbackConfig());
    assertEquals(0.0, config.forceThroughGoalDistanceMeters(), 1e-9);
    assertEquals(0.0, config.forceThroughWallDistanceMeters(), 1e-9);
    assertEquals(3.5, config.rerouteCandidateRadiusMeters(), 1e-9);
    assertEquals(8, config.rerouteCandidateCount());
    assertEquals(Alliance.kRed, config.fallbackAllianceWhenUnknown());
  }

  @Test
  void runtimeConfigCanonicalConstructorClampsInvalidValues() {
    FieldPlannerRuntimeConfig config =
        new FieldPlannerRuntimeConfig(true, null, 1.0, 1.0, -3.0, 0, Alliance.kBlue);

    assertEquals(0.0, config.rerouteCandidateRadiusMeters(), 1e-9);
    assertEquals(
        FieldPlannerRuntimeConfig.defaultRerouteCandidateCount(), config.rerouteCandidateCount());
    assertEquals(Alliance.kBlue, config.fallbackAllianceWhenUnknown());
  }

  @Test
  void rerouteTuningReadsSystemPropertyOverrides() {
    String oldRadius = System.getProperty("repulsor.fieldplanner.reroute.radiusMeters");
    String oldCount = System.getProperty("repulsor.fieldplanner.reroute.candidateCount");
    String oldAlliance = System.getProperty("repulsor.fieldplanner.reroute.fallbackAlliance");
    try {
      System.setProperty("repulsor.fieldplanner.reroute.radiusMeters", "2.25");
      System.setProperty("repulsor.fieldplanner.reroute.candidateCount", "5");
      System.setProperty("repulsor.fieldplanner.reroute.fallbackAlliance", "blue");

      FieldPlannerRuntimeConfig config = FieldPlannerRuntimeConfig.defaults();

      assertEquals(2.25, config.rerouteCandidateRadiusMeters(), 1e-9);
      assertEquals(5, config.rerouteCandidateCount());
      assertEquals(Alliance.kBlue, config.fallbackAllianceWhenUnknown());
    } finally {
      restoreProperty("repulsor.fieldplanner.reroute.radiusMeters", oldRadius);
      restoreProperty("repulsor.fieldplanner.reroute.candidateCount", oldCount);
      restoreProperty("repulsor.fieldplanner.reroute.fallbackAlliance", oldAlliance);
    }
  }

  private static void restoreProperty(String key, String value) {
    if (value == null) {
      System.clearProperty(key);
    } else {
      System.setProperty(key, value);
    }
  }
}
