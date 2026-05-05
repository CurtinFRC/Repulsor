package org.curtinfrc.frc2026.util.Repulsor.Scoring;

/** One named raw score value and the weight applied to it. */
public record WeightedScoreTerm(String name, double value, double weight) {
  public WeightedScoreTerm {
    if (name == null || name.isBlank()) name = "term";
    if (!Double.isFinite(value)) value = 0.0;
    if (!Double.isFinite(weight)) weight = 0.0;
  }

  public double contribution() {
    return value * weight;
  }

  public static WeightedScoreTerm gain(String name, double value, double weight) {
    return new WeightedScoreTerm(name, value, Math.max(0.0, weight));
  }

  public static WeightedScoreTerm cost(String name, double value, double weight) {
    return new WeightedScoreTerm(name, value, -Math.max(0.0, weight));
  }
}
