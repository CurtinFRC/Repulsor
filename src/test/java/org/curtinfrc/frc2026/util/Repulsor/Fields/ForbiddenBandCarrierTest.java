package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.CollectPlannerTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.ForbiddenBandTuning;
import org.junit.jupiter.api.Test;

class ForbiddenBandCarrierTest {
  private static final double EPS = 1e-12;

  @Test
  void defaultCarrierValuesEqualLegacyConstants() {
    ForbiddenBandTuning defaults = ForbiddenBandTuning.defaults();

    assertEquals(0.6, defaults.forbidMarginMeters(), EPS);
    assertEquals(4.625594, defaults.trenchSquareCenterXMeters(), EPS);
    assertEquals(3.63982, defaults.bumpRectCenterOffsetMeters(), EPS);
    assertEquals(1.1938 * 0.5, defaults.bandHalfWidthMeters(), EPS);
    assertEquals(1.1938 * 0.5 + 0.6, defaults.effectiveHalfWidthMeters(), EPS);
  }

  @Test
  void rebuilt2026ProviderMatchesLegacyDefaults() {
    ForbiddenBandTuning provided = Rebuilt2026.forbiddenBandTuning();
    ForbiddenBandTuning legacy = ForbiddenBandTuning.defaults();

    assertEquals(legacy.forbidMarginMeters(), provided.forbidMarginMeters(), EPS);
    assertEquals(legacy.trenchSquareCenterXMeters(), provided.trenchSquareCenterXMeters(), EPS);
    assertEquals(legacy.bumpRectCenterOffsetMeters(), provided.bumpRectCenterOffsetMeters(), EPS);
    assertEquals(legacy.bandHalfWidthMeters(), provided.bandHalfWidthMeters(), EPS);
  }

  @Test
  void rebuilt2026ProfileFeedsCarrierIntoPlannerTuning() {
    FieldProfileConfig cfg = Rebuilt2026.defaultProfileConfig();
    ForbiddenBandTuning flowed = cfg.collectPlanner.toCollectPlannerTuning().forbiddenBands();
    ForbiddenBandTuning expected = Rebuilt2026.forbiddenBandTuning();

    assertEquals(expected.forbidMarginMeters(), flowed.forbidMarginMeters(), EPS);
    assertEquals(expected.trenchSquareCenterXMeters(), flowed.trenchSquareCenterXMeters(), EPS);
    assertEquals(expected.bumpRectCenterOffsetMeters(), flowed.bumpRectCenterOffsetMeters(), EPS);
    assertEquals(expected.bandHalfWidthMeters(), flowed.bandHalfWidthMeters(), EPS);
  }

  @Test
  void customProfileValuesFlowThroughTuning() {
    FieldProfileConfig.CollectPlannerConfig config = new FieldProfileConfig.CollectPlannerConfig();
    config.forbidMarginMeters = 0.75;
    config.forbiddenBandSquareCenterXMeters = 5.5;
    config.forbiddenBandRectCenterOffsetMeters = 3.0;
    config.forbiddenBandHalfWidthMeters = 0.7;

    CollectPlannerTuning tuning = config.toCollectPlannerTuning();
    ForbiddenBandTuning bands = tuning.forbiddenBands();

    assertEquals(0.75, bands.forbidMarginMeters(), EPS);
    assertEquals(5.5, bands.trenchSquareCenterXMeters(), EPS);
    assertEquals(3.0, bands.bumpRectCenterOffsetMeters(), EPS);
    assertEquals(0.7, bands.bandHalfWidthMeters(), EPS);
  }

  @Test
  void nullCarrierFallsBackToLegacyDefaults() {
    ForbiddenBandTuning bands =
        new CollectPlannerTuning(0.40, 2.2, 0.30, 0.25, 0.40, 0.70, 0.14, null, null)
            .forbiddenBands();
    ForbiddenBandTuning legacy = ForbiddenBandTuning.defaults();

    assertEquals(legacy.effectiveHalfWidthMeters(), bands.effectiveHalfWidthMeters(), EPS);
  }
}
