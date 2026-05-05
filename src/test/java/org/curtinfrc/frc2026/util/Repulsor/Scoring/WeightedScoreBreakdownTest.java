package org.curtinfrc.frc2026.util.Repulsor.Scoring;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class WeightedScoreBreakdownTest {
  @Test
  void totalsNamedGainAndCostTerms() {
    WeightedScoreBreakdown breakdown =
        WeightedScoreBreakdown.of(
            WeightedScoreTerm.gain("advantage", 2.0, 3.0),
            WeightedScoreTerm.cost("distance", 4.0, 0.5));

    assertEquals(4.0, breakdown.total(), 1e-9);
    assertEquals(6.0, breakdown.term("advantage").orElseThrow().contribution(), 1e-9);
    assertEquals(-2.0, breakdown.term("distance").orElseThrow().contribution(), 1e-9);
  }

  @Test
  void sanitizesNonFiniteValuesAndDefensivelyCopiesTerms() {
    WeightedScoreTerm bad = WeightedScoreTerm.gain("bad", Double.NaN, Double.POSITIVE_INFINITY);
    WeightedScoreBreakdown breakdown = WeightedScoreBreakdown.of(bad);

    assertEquals(0.0, breakdown.total(), 1e-9);
    assertTrue(breakdown.term("bad").isPresent());
  }
}
