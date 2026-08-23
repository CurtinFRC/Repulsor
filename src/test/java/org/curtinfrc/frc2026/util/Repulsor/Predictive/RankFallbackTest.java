package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PredictiveRankingConfig;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElementModel;
import org.junit.jupiter.api.Test;

class RankFallbackTest {
  private static final double EPS = 1e-9;

  private static GameElement genericElement(Alliance alliance, int id, double x, double y) {
    return new GameElement(
        alliance,
        id,
        new GameElementModel(new Pose3d(x, y, 0.0, null)),
        gameObject -> true,
        null,
        CategorySpec.kScore);
  }

  @Test
  void tiedFallbackScoresKeepInsertionOrderStably() {
    PredictiveFieldStateRuntime predictor = new PredictiveFieldStateRuntime();
    GameElement first = genericElement(Alliance.kBlue, 1, 2.0, 2.0);
    GameElement second = genericElement(Alliance.kBlue, 2, 8.0, 2.0);

    predictor.setWorld(List.of(first, second), Alliance.kBlue);
    predictor.configureRanking(new PredictiveRankingConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    List<Candidate> ranked =
        predictor.rank(new Translation2d(5.0, 2.0), 3.0, CategorySpec.kScore, 4);

    assertEquals(2, ranked.size());
    assertEquals(0.0, ranked.get(0).score, EPS);
    assertEquals(0.0, ranked.get(1).score, EPS);
    assertEquals(2.0, ranked.get(0).targetXY.getX(), EPS);
    assertEquals(8.0, ranked.get(1).targetXY.getX(), EPS);

    List<Candidate> reranked =
        predictor.rank(new Translation2d(5.0, 2.0), 3.0, CategorySpec.kScore, 4);
    assertEquals(ranked.get(0).targetXY, reranked.get(0).targetXY);
    assertEquals(ranked.get(1).targetXY, reranked.get(1).targetXY);
  }

  @Test
  void fallbackRankingPenalizesEnemyHeldTargets() {
    PredictiveFieldStateRuntime predictor = new PredictiveFieldStateRuntime();
    predictor.setWorld(
        List.of(
            genericElement(Alliance.kBlue, 1, 2.0, 2.0),
            genericElement(Alliance.kBlue, 2, 8.0, 2.0)),
        Alliance.kBlue);
    predictor.updateEnemy(7, new Translation2d(8.4, 2.0), null, null);

    List<Candidate> ranked =
        predictor.rank(new Translation2d(1.0, 2.0), 3.0, CategorySpec.kScore, 4);

    assertEquals(2, ranked.size());
    assertTrue(ranked.get(0).score > ranked.get(1).score);
    assertEquals(2.0, ranked.get(0).targetXY.getX(), EPS);
    assertEquals(8.0, ranked.get(1).targetXY.getX(), EPS);
    assertTrue(Double.isFinite(ranked.get(0).ourEtaS));
    assertTrue(Double.isFinite(ranked.get(1).enemyEtaS));
  }

  @Test
  void fallbackCandidatesExposeSyntheticSetpointsWithCategoryLevelId() {
    PredictiveFieldStateRuntime predictor = new PredictiveFieldStateRuntime();
    predictor.setWorld(List.of(genericElement(Alliance.kBlue, 3, 3.0, 2.0)), Alliance.kBlue);

    List<Candidate> ranked =
        predictor.rank(new Translation2d(1.0, 2.0), 3.0, CategorySpec.kScore, 4);

    assertEquals(1, ranked.size());
    assertEquals("score", ranked.get(0).setpoint.levelId());
    assertEquals(3.0, ranked.get(0).setpoint.getBlue(null).getX(), EPS);
    assertEquals(2.0, ranked.get(0).setpoint.getBlue(null).getY(), EPS);
  }
}
