package org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective;

import java.util.Map;
import org.curtinfrc.frc2026.util.Repulsor.Diagnostics.RepulsorDecisionEntry;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;

/** Explains the selected objective and whether the selector held or switched. */
public record ObjectiveSelectionDecision(
    Mode mode,
    Candidate selected,
    Candidate currentCandidate,
    Candidate bestCandidate,
    double scoreDelta,
    String reason) {
  public enum Mode {
    NONE,
    HOLD_CURRENT,
    SWITCH_TO_BEST
  }

  public ObjectiveSelectionDecision {
    if (mode == null) mode = Mode.NONE;
    if (!Double.isFinite(scoreDelta)) scoreDelta = 0.0;
    if (reason == null || reason.isBlank()) reason = mode.name().toLowerCase();
  }

  public boolean hasSelection() {
    return selected != null;
  }

  public boolean switched() {
    return mode == Mode.SWITCH_TO_BEST;
  }

  public static ObjectiveSelectionDecision none(String reason) {
    return new ObjectiveSelectionDecision(Mode.NONE, null, null, null, 0.0, reason);
  }

  public RepulsorDecisionEntry asDecisionEntry(String layer) {
    return new RepulsorDecisionEntry(
        layer,
        mode.name().toLowerCase(),
        reason,
        describeCandidate(selected),
        describeCandidate(currentCandidate),
        selected != null ? selected.score : 0.0,
        scoreDelta,
        Map.of("best", describeCandidate(bestCandidate), "switched", Boolean.toString(switched())));
  }

  private static String describeCandidate(Candidate candidate) {
    if (candidate == null) return "";
    String level = candidate.setpoint != null ? candidate.setpoint.levelId() : "candidate";
    if (candidate.targetXY == null) return level;
    return String.format(
        "%s@%.3f,%.3f", level, candidate.targetXY.getX(), candidate.targetXY.getY());
  }
}
