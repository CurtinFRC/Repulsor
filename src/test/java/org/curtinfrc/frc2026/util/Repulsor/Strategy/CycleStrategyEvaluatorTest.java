package org.curtinfrc.frc2026.util.Repulsor.Strategy;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Decision;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Inputs;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Intent;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Tuning;
import org.junit.jupiter.api.Test;

class CycleStrategyEvaluatorTest {
  @Test
  void inactiveHubPrefersCenterTransferWhenCenterFuelExists() {
    Decision decision =
        CycleStrategyEvaluator.decide(
            new Inputs(
                false,
                0.0,
                4.0,
                summary("side", 2.0, 1.2),
                summary("center", 4.0, 2.0),
                2.0,
                3.0,
                4.0,
                Intent.FALLBACK),
            Tuning.defaults());

    assertEquals(Intent.TRANSFER_FOR_LATER_SCORE, decision.intent());
  }

  @Test
  void activeHubScoresNearbyAllianceFuelWhenDeadlineIsShort() {
    Decision decision =
        CycleStrategyEvaluator.decide(
            new Inputs(
                true,
                4.0,
                4.0,
                summary("side", 2.0, 0.8),
                summary("center", 8.0, 4.5),
                0.8,
                4.5,
                4.0,
                Intent.FALLBACK),
            Tuning.defaults());

    assertEquals(Intent.SCORE_AVAILABLE_RESOURCES, decision.intent());
  }

  @Test
  void activeHubCanStillChooseCenterWhenItIsMuchRicherAndReachable() {
    Decision decision =
        CycleStrategyEvaluator.decide(
            new Inputs(
                true,
                24.0,
                5.2,
                summary("side", 1.0, 2.0),
                summary("center", 12.0, 2.4),
                2.0,
                2.4,
                3.0,
                Intent.FALLBACK),
            Tuning.defaults());

    assertEquals(Intent.TRANSFER_FOR_LATER_SCORE, decision.intent());
  }

  private static ResourceRegionSummary summary(String id, double units, double distance) {
    return new ResourceRegionSummary(
        id, units, units, new Translation2d(distance, 0.0), distance, 0.0, 0.0);
  }
}
