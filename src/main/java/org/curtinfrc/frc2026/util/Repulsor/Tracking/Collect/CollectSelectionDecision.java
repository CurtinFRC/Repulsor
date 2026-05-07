package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Map;
import org.curtinfrc.frc2026.util.Repulsor.Diagnostics.RepulsorDecisionEntry;
import org.curtinfrc.frc2026.util.Repulsor.Scoring.WeightedScoreBreakdown;

/**
 * Explains one collect-objective selection update without owning low-level path/bypass behavior.
 */
public record CollectSelectionDecision(
    Translation2d selected,
    Translation2d previous,
    String reason,
    WeightedScoreBreakdown score,
    boolean liveEvidenceRequired,
    boolean liveEvidenceFound,
    boolean staleObservationPresent,
    boolean canonicalized,
    boolean relockedToLiveEvidence,
    boolean trapPenaltyApplied,
    boolean stickyHeld,
    boolean switched) {
  public CollectSelectionDecision {
    if (reason == null || reason.isBlank()) reason = "unspecified";
    if (score == null) score = WeightedScoreBreakdown.empty();
  }

  public static CollectSelectionDecision empty() {
    return new CollectSelectionDecision(
        null,
        null,
        "empty",
        WeightedScoreBreakdown.empty(),
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false);
  }

  public RepulsorDecisionEntry asDecisionEntry(String layer) {
    return RepulsorDecisionEntry.of(
            layer, switched ? "switch" : stickyHeld ? "hold" : "select", reason)
        .withSelection(format(selected), format(previous))
        .withScore(score.total(), scoreDelta())
        .withMetadata(
            Map.of(
                "liveEvidenceRequired",
                Boolean.toString(liveEvidenceRequired),
                "liveEvidenceFound",
                Boolean.toString(liveEvidenceFound),
                "staleObservationPresent",
                Boolean.toString(staleObservationPresent),
                "canonicalized",
                Boolean.toString(canonicalized),
                "relockedToLiveEvidence",
                Boolean.toString(relockedToLiveEvidence),
                "trapPenaltyApplied",
                Boolean.toString(trapPenaltyApplied),
                "stickyHeld",
                Boolean.toString(stickyHeld),
                "switched",
                Boolean.toString(switched)));
  }

  public double scoreDelta() {
    return score.total();
  }

  private static String format(Translation2d point) {
    if (point == null) return "";
    return String.format("%.3f,%.3f", point.getX(), point.getY());
  }
}
