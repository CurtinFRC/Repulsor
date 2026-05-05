package org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective;

import java.util.Comparator;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

/** Converts ranked predictive candidates into a reusable objective hold/switch decision. */
public final class ObjectiveSelector {
  private ObjectiveSelector() {}

  public static ObjectiveSelectionDecision select(
      List<Candidate> rankedCandidates,
      RepulsorSetpoint currentObjective,
      ObjectiveSelectionConfig config) {
    ObjectiveSelectionConfig safeConfig =
        config == null ? ObjectiveSelectionConfig.defaults() : config;
    if (rankedCandidates == null || rankedCandidates.isEmpty()) {
      return ObjectiveSelectionDecision.none("no_ranked_candidates");
    }

    Candidate best =
        rankedCandidates.stream()
            .filter(ObjectiveSelector::valid)
            .max(Comparator.comparingDouble(candidate -> candidate.score))
            .orElse(null);
    if (best == null) {
      return ObjectiveSelectionDecision.none("no_valid_candidates");
    }

    Candidate current = findCurrent(rankedCandidates, currentObjective);
    if (!safeConfig.holdCurrentWhenRanked() || currentObjective == null || current == null) {
      return new ObjectiveSelectionDecision(
          ObjectiveSelectionDecision.Mode.SWITCH_TO_BEST,
          best,
          current,
          best,
          0.0,
          currentObjective == null ? "no_current_objective" : "current_not_ranked");
    }

    double scoreDelta = best.score - current.score;
    if (sameSetpoint(best, current)) {
      return new ObjectiveSelectionDecision(
          ObjectiveSelectionDecision.Mode.HOLD_CURRENT,
          current,
          current,
          best,
          0.0,
          "current_is_best");
    }

    if (scoreDelta <= safeConfig.switchScoreMargin()) {
      return new ObjectiveSelectionDecision(
          ObjectiveSelectionDecision.Mode.HOLD_CURRENT,
          current,
          current,
          best,
          scoreDelta,
          "within_switch_margin");
    }

    return new ObjectiveSelectionDecision(
        ObjectiveSelectionDecision.Mode.SWITCH_TO_BEST,
        best,
        current,
        best,
        scoreDelta,
        "best_exceeds_margin");
  }

  private static Candidate findCurrent(
      List<Candidate> rankedCandidates, RepulsorSetpoint currentObjective) {
    if (rankedCandidates == null || currentObjective == null) return null;
    return rankedCandidates.stream()
        .filter(ObjectiveSelector::valid)
        .filter(candidate -> sameSetpoint(currentObjective, candidate.setpoint))
        .findFirst()
        .orElse(null);
  }

  private static boolean sameSetpoint(Candidate a, Candidate b) {
    return a != null && b != null && sameSetpoint(a.setpoint, b.setpoint);
  }

  private static boolean sameSetpoint(RepulsorSetpoint a, RepulsorSetpoint b) {
    return a != null
        && b != null
        && a.point() == b.point()
        && a.height() == b.height()
        && a.levelId().equals(b.levelId());
  }

  private static boolean valid(Candidate candidate) {
    return candidate != null && candidate.setpoint != null && Double.isFinite(candidate.score);
  }
}
