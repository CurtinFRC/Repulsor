package org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.Setpoints;
import org.junit.jupiter.api.Test;

class ObjectiveSelectorTest {
  private final RepulsorSetpoint current =
      new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET);
  private final RepulsorSetpoint challenger =
      new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE);

  @Test
  void holdsCurrentWhenBestDoesNotBeatSwitchMargin() {
    Candidate currentCandidate = candidate(current, 10.0);
    Candidate bestCandidate = candidate(challenger, 10.10);

    ObjectiveSelectionDecision decision =
        ObjectiveSelector.select(
            List.of(bestCandidate, currentCandidate),
            current,
            new ObjectiveSelectionConfig(8, 0.15, true));

    assertEquals(ObjectiveSelectionDecision.Mode.HOLD_CURRENT, decision.mode());
    assertSame(currentCandidate, decision.selected());
    assertSame(bestCandidate, decision.bestCandidate());
    assertEquals(0.10, decision.scoreDelta(), 1e-9);
    assertFalse(decision.switched());
  }

  @Test
  void holdsSemanticallyEquivalentCurrentSetpoint() {
    RepulsorSetpoint sameCurrent =
        new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, "net", HeightSetpoint.NET);
    Candidate currentCandidate = candidate(current, 10.0);
    Candidate bestCandidate = candidate(challenger, 10.10);

    ObjectiveSelectionDecision decision =
        ObjectiveSelector.select(
            List.of(bestCandidate, currentCandidate),
            sameCurrent,
            new ObjectiveSelectionConfig(8, 0.15, true));

    assertEquals(ObjectiveSelectionDecision.Mode.HOLD_CURRENT, decision.mode());
    assertSame(currentCandidate, decision.selected());
  }

  @Test
  void switchesWhenBestBeatsSwitchMargin() {
    Candidate currentCandidate = candidate(current, 10.0);
    Candidate bestCandidate = candidate(challenger, 10.30);

    ObjectiveSelectionDecision decision =
        ObjectiveSelector.select(
            List.of(currentCandidate, bestCandidate),
            current,
            new ObjectiveSelectionConfig(8, 0.15, true));

    assertEquals(ObjectiveSelectionDecision.Mode.SWITCH_TO_BEST, decision.mode());
    assertSame(bestCandidate, decision.selected());
    assertTrue(decision.switched());
    assertEquals(0.30, decision.scoreDelta(), 1e-9);
  }

  @Test
  void switchesToBestWhenCurrentIsMissingOrHoldDisabled() {
    Candidate bestCandidate = candidate(challenger, 5.0);

    ObjectiveSelectionDecision missingCurrent =
        ObjectiveSelector.select(
            List.of(bestCandidate), current, ObjectiveSelectionConfig.defaults());
    assertEquals(ObjectiveSelectionDecision.Mode.SWITCH_TO_BEST, missingCurrent.mode());
    assertEquals("current_not_ranked", missingCurrent.reason());

    ObjectiveSelectionDecision holdDisabled =
        ObjectiveSelector.select(
            List.of(bestCandidate, candidate(current, 4.99)),
            current,
            new ObjectiveSelectionConfig(8, 100.0, false));
    assertEquals(ObjectiveSelectionDecision.Mode.SWITCH_TO_BEST, holdDisabled.mode());
  }

  @Test
  void returnsNoneForNoUsableCandidates() {
    ObjectiveSelectionDecision decision =
        ObjectiveSelector.select(
            List.of(candidate(null, 10.0)), current, ObjectiveSelectionConfig.defaults());

    assertEquals(ObjectiveSelectionDecision.Mode.NONE, decision.mode());
    assertFalse(decision.hasSelection());
  }

  private static Candidate candidate(RepulsorSetpoint setpoint, double score) {
    return new Candidate(setpoint, new Translation2d(score, 0.0), 0.0, 0.0, 0.0, 0.0, 0.0, score);
  }
}
