package org.curtinfrc.frc2026.util.Repulsor.Predictive.Model;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Scoring.WeightedScoreBreakdown;
import org.curtinfrc.frc2026.util.Repulsor.Scoring.WeightedScoreTerm;

/** Detailed score terms for the latest predictive field-object ranking pass. */
public record PredictiveRankingBreakdown(
    String levelId, Translation2d target, WeightedScoreBreakdown breakdown) {
  public PredictiveRankingBreakdown {
    if (levelId == null || levelId.isBlank()) levelId = "objective";
    if (target == null) target = new Translation2d();
    if (breakdown == null) breakdown = WeightedScoreBreakdown.empty();
  }

  public PredictiveRankingBreakdown(
      String levelId,
      Translation2d target,
      double totalScore,
      double advantageTerm,
      double distanceTerm,
      double pressureTerm,
      double congestionTerm,
      double capacityTerm,
      double headingTerm,
      double hysteresisTerm) {
    this(
        levelId,
        target,
        WeightedScoreBreakdown.of(
            new WeightedScoreTerm("advantage", 1.0, advantageTerm),
            new WeightedScoreTerm("distance", 1.0, distanceTerm),
            new WeightedScoreTerm("pressure", 1.0, pressureTerm),
            new WeightedScoreTerm("congestion", 1.0, congestionTerm),
            new WeightedScoreTerm("capacity", 1.0, capacityTerm),
            new WeightedScoreTerm("heading", 1.0, headingTerm),
            new WeightedScoreTerm("hysteresis", 1.0, hysteresisTerm),
            new WeightedScoreTerm(
                "residual",
                1.0,
                residual(
                    totalScore,
                    advantageTerm,
                    distanceTerm,
                    pressureTerm,
                    congestionTerm,
                    capacityTerm,
                    headingTerm,
                    hysteresisTerm))));
  }

  public double totalScore() {
    return breakdown.total();
  }

  public double advantageTerm() {
    return contribution("advantage");
  }

  public double distanceTerm() {
    return contribution("distance");
  }

  public double pressureTerm() {
    return contribution("pressure");
  }

  public double congestionTerm() {
    return contribution("congestion");
  }

  public double capacityTerm() {
    return contribution("capacity");
  }

  public double headingTerm() {
    return contribution("heading");
  }

  public double hysteresisTerm() {
    return contribution("hysteresis");
  }

  private double contribution(String termName) {
    return breakdown.term(termName).map(WeightedScoreTerm::contribution).orElse(0.0);
  }

  private static double residual(
      double totalScore,
      double advantageTerm,
      double distanceTerm,
      double pressureTerm,
      double congestionTerm,
      double capacityTerm,
      double headingTerm,
      double hysteresisTerm) {
    double expected =
        advantageTerm
            + distanceTerm
            + pressureTerm
            + congestionTerm
            + capacityTerm
            + headingTerm
            + hysteresisTerm;
    double residual = totalScore - expected;
    return Math.abs(residual) < 1e-9 ? 0.0 : residual;
  }
}
