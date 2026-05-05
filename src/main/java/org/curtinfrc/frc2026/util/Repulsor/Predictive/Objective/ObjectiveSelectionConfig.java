package org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective;

/** Tuning for turning a ranked objective list into a hold/switch decision. */
public record ObjectiveSelectionConfig(
    int candidateLimit, double switchScoreMargin, boolean holdCurrentWhenRanked) {
  public static final int DEFAULT_CANDIDATE_LIMIT = 8;
  public static final double DEFAULT_SWITCH_SCORE_MARGIN = 0.15;

  public ObjectiveSelectionConfig {
    candidateLimit = Math.max(1, candidateLimit);
    switchScoreMargin = Double.isFinite(switchScoreMargin) ? Math.max(0.0, switchScoreMargin) : 0.0;
  }

  public static ObjectiveSelectionConfig defaults() {
    return new ObjectiveSelectionConfig(DEFAULT_CANDIDATE_LIMIT, DEFAULT_SWITCH_SCORE_MARGIN, true);
  }
}
