package org.curtinfrc.frc2026.util.Repulsor.Predictive.Model;

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
  public static final double DEFAULT_ADVANTAGE_GAIN = 1.1;
  public static final double DEFAULT_DISTANCE_COST = 0.10;
  public static final double DEFAULT_PRESSURE_COST = 0.72;
  public static final double DEFAULT_CONGESTION_COST = 0.95;
  public static final double DEFAULT_CAPACITY_GAIN = 0.45;
  public static final double DEFAULT_HEADING_GAIN = 0.18;
  public static final double DEFAULT_HYSTERESIS_BONUS = 0.22;
  public static final double DEFAULT_HYSTERESIS_PERSIST_SECONDS = 0.8;

  public static PredictiveRankingConfig defaults() {
    return new PredictiveRankingConfig(
        DEFAULT_ADVANTAGE_GAIN,
        DEFAULT_DISTANCE_COST,
        DEFAULT_PRESSURE_COST,
        DEFAULT_CONGESTION_COST,
        DEFAULT_CAPACITY_GAIN,
        DEFAULT_HEADING_GAIN,
        DEFAULT_HYSTERESIS_BONUS,
        DEFAULT_HYSTERESIS_PERSIST_SECONDS);
  }

  public PredictiveRankingConfig {
    advantageGain = finiteNonNegative(advantageGain, DEFAULT_ADVANTAGE_GAIN);
    distanceCost = finiteNonNegative(distanceCost, DEFAULT_DISTANCE_COST);
    pressureCost = finiteNonNegative(pressureCost, DEFAULT_PRESSURE_COST);
    congestionCost = finiteNonNegative(congestionCost, DEFAULT_CONGESTION_COST);
    capacityGain = finiteNonNegative(capacityGain, DEFAULT_CAPACITY_GAIN);
    headingGain = finiteNonNegative(headingGain, DEFAULT_HEADING_GAIN);
    hysteresisBonus = finiteNonNegative(hysteresisBonus, DEFAULT_HYSTERESIS_BONUS);
    hysteresisPersistSeconds =
        finiteNonNegative(hysteresisPersistSeconds, DEFAULT_HYSTERESIS_PERSIST_SECONDS);
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
