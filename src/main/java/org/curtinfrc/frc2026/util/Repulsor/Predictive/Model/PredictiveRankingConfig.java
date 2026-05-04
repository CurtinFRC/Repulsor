package org.curtinfrc.frc2026.util.Repulsor.Predictive.Model;

import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateOps;

/** Tunable weights used when ranking field-object setpoints. */
public record PredictiveRankingConfig(
    double advantageGain,
    double distanceCost,
    double pressureCost,
    double congestionCost,
    double capacityGain,
    double headingGain,
    double hysteresisBonus,
    double hysteresisPersistSeconds) {
  public static PredictiveRankingConfig defaults() {
    return new PredictiveRankingConfig(
        PredictiveFieldStateOps.ADV_GAIN,
        PredictiveFieldStateOps.DIST_COST,
        PredictiveFieldStateOps.PRESSURE_GAIN,
        PredictiveFieldStateOps.CONGEST_COST,
        PredictiveFieldStateOps.CAPACITY_GAIN,
        PredictiveFieldStateOps.HEADING_GAIN,
        PredictiveFieldStateOps.HYST_BONUS,
        PredictiveFieldStateOps.HYST_PERSIST_S);
  }

  public PredictiveRankingConfig {
    advantageGain = finiteNonNegative(advantageGain, PredictiveFieldStateOps.ADV_GAIN);
    distanceCost = finiteNonNegative(distanceCost, PredictiveFieldStateOps.DIST_COST);
    pressureCost = finiteNonNegative(pressureCost, PredictiveFieldStateOps.PRESSURE_GAIN);
    congestionCost = finiteNonNegative(congestionCost, PredictiveFieldStateOps.CONGEST_COST);
    capacityGain = finiteNonNegative(capacityGain, PredictiveFieldStateOps.CAPACITY_GAIN);
    headingGain = finiteNonNegative(headingGain, PredictiveFieldStateOps.HEADING_GAIN);
    hysteresisBonus = finiteNonNegative(hysteresisBonus, PredictiveFieldStateOps.HYST_BONUS);
    hysteresisPersistSeconds =
        finiteNonNegative(hysteresisPersistSeconds, PredictiveFieldStateOps.HYST_PERSIST_S);
  }

  public PredictiveRankingConfig withDistanceCost(double value) {
    return new PredictiveRankingConfig(
        advantageGain,
        value,
        pressureCost,
        congestionCost,
        capacityGain,
        headingGain,
        hysteresisBonus,
        hysteresisPersistSeconds);
  }

  public PredictiveRankingConfig withCapacityGain(double value) {
    return new PredictiveRankingConfig(
        advantageGain,
        distanceCost,
        pressureCost,
        congestionCost,
        value,
        headingGain,
        hysteresisBonus,
        hysteresisPersistSeconds);
  }

  private static double finiteNonNegative(double value, double fallback) {
    return Double.isFinite(value) && value >= 0.0 ? value : fallback;
  }
}
