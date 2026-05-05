package org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective;

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
}
