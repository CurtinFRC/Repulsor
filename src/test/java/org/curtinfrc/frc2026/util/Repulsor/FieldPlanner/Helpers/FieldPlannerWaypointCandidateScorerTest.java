package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Scoring.WeightedScoreBreakdown;
import org.junit.jupiter.api.Test;

class FieldPlannerWaypointCandidateScorerTest {
  @Test
  void breakdownTotalMatchesLegacyScoreFormula() {
    FieldPlannerWaypointContext context =
        new FieldPlannerWaypointContext(
            new Translation2d(0.0, 0.0),
            new Pose2d(4.0, 0.0, Rotation2d.kZero),
            List.of(),
            List.of(),
            8.0,
            4.0,
            FieldPlannerWaypointObjectiveRole.SCORE,
            FieldPlannerWaypointConfig.defaults(),
            false,
            null,
            false,
            null,
            false);
    FieldPlannerWaypointCandidate candidate =
        new FieldPlannerWaypointCandidate(
            "upper", new Translation2d(2.0, 1.0), null, null, 0.7, true);
    FieldPlannerWaypointScoringConfig config =
        new FieldPlannerWaypointScoringConfig(1.0, 0.35, 0.2, 1.0);

    double legacyScore = FieldPlannerWaypointCandidateScorer.score(context, candidate, config);
    WeightedScoreBreakdown breakdown =
        FieldPlannerWaypointCandidateScorer.scoreBreakdown(context, candidate, config);

    assertEquals(legacyScore, breakdown.total(), 1e-9);
    assertEquals(4, breakdown.terms().size());
    assertTrue(breakdown.term("detour").orElseThrow().contribution() <= 0.0);
    assertTrue(breakdown.term("goalAlignment").orElseThrow().contribution() > 0.0);
  }

  @Test
  void bestCandidateCarriesExplanationBreakdown() {
    FieldPlannerWaypointContext context =
        new FieldPlannerWaypointContext(
            new Translation2d(0.0, 0.0),
            new Pose2d(4.0, 0.0, Rotation2d.kZero),
            List.of(),
            List.of(),
            8.0,
            4.0,
            FieldPlannerWaypointObjectiveRole.SCORE,
            FieldPlannerWaypointConfig.defaults(),
            false,
            null,
            false,
            null,
            false);
    FieldPlannerWaypointCandidate lowPreference =
        new FieldPlannerWaypointCandidate(
            "low", new Translation2d(2.0, 1.0), null, null, 0.0, true);
    FieldPlannerWaypointCandidate highPreference =
        new FieldPlannerWaypointCandidate(
            "high", new Translation2d(2.0, 1.0), null, null, 2.0, true);

    FieldPlannerWaypointCandidateScorer.ScoredCandidate best =
        FieldPlannerWaypointCandidateScorer.best(
                context,
                List.of(lowPreference, highPreference),
                FieldPlannerWaypointScoringConfig.defaults())
            .orElseThrow();

    assertSame(highPreference, best.candidate());
    assertEquals(best.score(), best.breakdown().total(), 1e-9);
    assertTrue(best.breakdown().term("preference").isPresent());
  }
}
