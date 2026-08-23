package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class WaypointConfigDefaultsTest {
  private static final double EPS = 1e-12;

  @Test
  void defaultsKeepAllBuiltInStrategiesEnabled() {
    FieldPlannerWaypointConfig config = FieldPlannerWaypointConfig.defaults();
    assertTrue(config.bandTransitionStagingEnabled());
    assertTrue(config.occludingGateStagingEnabled());
    assertTrue(config.centerReturnStagingEnabled());
  }

  @Test
  void defaultsRegisterNoCustomPolicies() {
    assertTrue(FieldPlannerWaypointConfig.defaults().customPolicies().isEmpty());
  }

  @Test
  void defaultPlacementMatchesHistoricalGoalManagerLiterals() {
    FieldPlannerWaypointPlacement placement = FieldPlannerWaypointPlacement.defaults();
    assertEquals(0.50, placement.entryReachEnterMeters(), EPS);
    assertEquals(0.45, placement.exitReachEnterMeters(), EPS);
    assertEquals(0.55, placement.reachExitMeters(), EPS);
    assertEquals(0.85, placement.entryPassWithinMeters(), EPS);
    assertEquals(0.06, placement.entryPassedProjectionMeters(), EPS);
    assertEquals(0.10, placement.exitPassedProjectionMeters(), EPS);
    assertEquals(0.40, placement.gateClearExitMeters(), EPS);
    assertEquals(0.35, placement.passedGateHysteresisMeters(), EPS);
    assertEquals(0.05, placement.goalSideProjectionMeters(), EPS);
    assertEquals(0.45, placement.centerReturnExitAdvanceScale(), EPS);
    assertEquals(
        FieldPlannerWaypointPlacement.defaults(),
        FieldPlannerWaypointConfig.defaults().placement(),
        "defaults() must carry the default placement");
  }

  @Test
  void compatibilityConstructorKeepsDefaultPlacementAndNoPolicies() {
    FieldPlannerWaypointConfig legacy =
        new FieldPlannerWaypointConfig(
            true, true, true, 3.648981, 1.5, 0.25, 0.28, 0.45, 1.05, 1.40, 3.0, 4.2, 0.70, 2.40,
            2.0, 0.35);
    assertEquals(FieldPlannerWaypointPlacement.defaults(), legacy.placement());
    assertTrue(legacy.customPolicies().isEmpty());
  }

  @Test
  void withersPreserveCustomPoliciesAndPlacement() {
    FieldPlannerWaypointPolicy policy = context -> List.of();
    FieldPlannerWaypointPlacement placement =
        new FieldPlannerWaypointPlacement(0.6, 0.5, 0.7, 0.9, 0.08, 0.12, 0.5, 0.4, 0.06, 0.5);
    FieldPlannerWaypointConfig config =
        FieldPlannerWaypointConfig.defaults()
            .withCustomPolicies(policy)
            .withPlacement(placement)
            .withBandTransitionStagingEnabled(false)
            .withOccludingGateStagingEnabled(false)
            .withCenterReturnStagingEnabled(false)
            .withLeadThroughMeters(0.5, 1.2)
            .withCenterBandMeters(2.5);

    assertFalse(config.bandTransitionStagingEnabled());
    assertFalse(config.occludingGateStagingEnabled());
    assertFalse(config.centerReturnStagingEnabled());
    assertEquals(1, config.customPolicies().size());
    assertSame(policy, config.customPolicies().get(0));
    assertEquals(placement, config.placement());
    assertEquals(2.5, config.centerBandMeters(), EPS);
    assertEquals(0.5, config.leadThroughMinMeters(), EPS);
  }

  @Test
  void withCustomPoliciesAppendsAndIgnoresNulls() {
    FieldPlannerWaypointPolicy first = context -> List.of();
    FieldPlannerWaypointPolicy second = context -> List.of();
    FieldPlannerWaypointConfig config =
        FieldPlannerWaypointConfig.defaults()
            .withCustomPolicies(first)
            .withCustomPolicies(null, second);

    assertEquals(2, config.customPolicies().size());
    assertSame(first, config.customPolicies().get(0));
    assertSame(second, config.customPolicies().get(1));

    FieldPlannerWaypointConfig normalized =
        new FieldPlannerWaypointConfig(
            true,
            true,
            true,
            3.648981,
            1.5,
            0.25,
            0.28,
            0.45,
            1.05,
            1.40,
            3.0,
            4.2,
            0.70,
            2.40,
            2.0,
            0.35,
            null,
            null);
    assertTrue(normalized.customPolicies().isEmpty());
    assertEquals(FieldPlannerWaypointPlacement.defaults(), normalized.placement());
  }

  @Test
  void placementNormalizationFallsBackToDefaultsOnInvalidValues() {
    FieldPlannerWaypointPlacement invalid =
        new FieldPlannerWaypointPlacement(
            Double.NaN, -1.0, -0.01, Double.NaN, -1.0, Double.NaN, -1.0, Double.NaN, -1.0,
            Double.NaN);
    assertEquals(FieldPlannerWaypointPlacement.defaults(), invalid);

    FieldPlannerWaypointConfig withInvalid =
        FieldPlannerWaypointConfig.defaults().withPlacement(null);
    assertEquals(FieldPlannerWaypointPlacement.defaults(), withInvalid.placement());
  }
}
