package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.junit.jupiter.api.Test;

class CoarseGlobalPlannerLazyTest {
  private static final Translation2d START = new Translation2d(1.0, 2.0);
  private static final Pose2d GOAL = new Pose2d(5.0, 2.0, Rotation2d.kZero);

  private static List<RectangleObstacle> walledBoxAroundGoal() {
    return List.of(
        RectangleObstacle.simple(new Translation2d(4.2, 2.0), 0.7, 2.4, 1.0, 1.0, 1.0),
        RectangleObstacle.simple(new Translation2d(5.8, 2.0), 0.7, 2.4, 1.0, 1.0, 1.0),
        RectangleObstacle.simple(new Translation2d(5.0, 2.8), 2.4, 0.7, 1.0, 1.0, 1.0),
        RectangleObstacle.simple(new Translation2d(5.0, 1.2), 2.4, 0.7, 1.0, 1.0, 1.0));
  }

  @Test
  void goalSealedInsideWalledBoxExhaustsSearchAndReportsNoRoute() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.35, 0.8, 5000, 1.0));

    var waypoint = planner.nextWaypoint(START, GOAL, walledBoxAroundGoal(), 0.18, 0.18, 6.0, 4.0);

    assertFalse(waypoint.isPresent(), "sealed box should yield no waypoint");
    assertFalse(planner.lastStats().found());
    assertFalse(planner.lastStats().timedOut());
    assertFalse(planner.lastStats().exhaustedNodeBudget());
    assertTrue(planner.lastStats().expandedNodes() > 0);
    assertEquals(
        CoarseGlobalPlannerFailureReason.NO_ROUTE,
        planner.lastStats().failureReason(),
        "stats=" + planner.lastStats());
  }

  @Test
  void partialRouteFallbackStillProducesActiveWaypointForSealedBox() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(
            new CoarseGlobalPlannerConfig(0.35, 0.8, 5000, 1.0)
                .withPartialRouteFallback(true, 0.6, 0.0));

    var waypoint = planner.nextWaypoint(START, GOAL, walledBoxAroundGoal(), 0.18, 0.18, 6.0, 4.0);

    assertTrue(waypoint.isPresent(), "partial route should make safe progress, stats="
        + planner.lastStats());
    assertTrue(planner.lastStats().found());
    assertEquals(
        CoarseGlobalPlannerFailureReason.PARTIAL_ROUTE_USED,
        planner.lastStats().failureReason());
    assertTrue(
        waypoint.get().getTranslation().getDistance(GOAL.getTranslation())
            < START.getDistance(GOAL.getTranslation()),
        "partial route should progress toward the sealed goal");
    assertTrue(
        waypoint.get().getTranslation().getDistance(START) > 1e-9,
        "active waypoint should not collapse onto the start pose");
  }

  @Test
  void nodeBudgetTripStillReportsNodeBudgetFailureWithLazyPartialEvaluation() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(
            new CoarseGlobalPlannerConfig(0.25, 1.0, 1, 1.0)
                .withPartialRouteFallback(true, 0.0, 0.0));

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(0.5, 0.5),
            new Pose2d(5.5, 3.5, Rotation2d.kZero),
            List.of(),
            0.18,
            0.18,
            6.0,
            4.0);

    assertFalse(waypoint.isPresent());
    assertTrue(planner.lastStats().exhaustedNodeBudget());
    assertEquals(1, planner.lastStats().expandedNodes());
    assertEquals(CoarseGlobalPlannerFailureReason.NODE_BUDGET, planner.lastStats().failureReason());
  }
}
