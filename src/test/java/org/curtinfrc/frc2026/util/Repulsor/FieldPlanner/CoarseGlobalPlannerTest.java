package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PredictedDynamicObstacleEnvelope;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Force;
import org.junit.jupiter.api.Test;

class CoarseGlobalPlannerTest {
  @Test
  void findsWaypointAroundBlockingRectangle() {
    CoarseGlobalPlanner planner = deterministicPlanner();
    RectangleObstacle block =
        RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.9, 2.0, 1.0, 1.0, 1.0);

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 2.0),
            new Pose2d(5.0, 2.0, Rotation2d.kZero),
            List.of(block),
            0.18,
            0.18,
            6.0,
            4.0);

    assertTrue(waypoint.isPresent());
    assertTrue(
        Math.abs(waypoint.get().getY() - 2.0) > 0.2,
        "Waypoint should leave the blocked centerline");
    assertTrue(planner.lastStats().rawPathNodes() >= planner.lastStats().pathNodes());
  }

  @Test
  void routeQualityBaselinePrefersDeterministicCorridorSideAcrossRepeatedCalls() {
    CoarseGlobalPlanner planner = deterministicPlanner();
    List<Obstacle> obstacles =
        List.of(
            RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.75, 0.85, 1.0, 1.0, 1.0),
            RectangleObstacle.simple(new Translation2d(3.0, 3.15), 0.75, 0.55, 1.0, 1.0, 1.0));

    Double firstSide = null;
    for (int i = 0; i < 6; i++) {
      var waypoint =
          planner.nextWaypoint(
              new Translation2d(1.0, 2.0),
              new Pose2d(5.0, 2.0, Rotation2d.kZero),
              obstacles,
              0.18,
              0.18,
              6.0,
              4.0);

      assertTrue(waypoint.isPresent());
      double side = Math.signum(waypoint.get().getY() - 2.0);
      assertTrue(Math.abs(side) > 0.0, "route should choose a corridor side");
      if (firstSide == null) firstSide = side;
      assertEquals(firstSide, side, 0.0, "identical calls should not flip corridor sides");
      assertTrue(planner.lastStats().rawPathNodes() >= planner.lastStats().pathNodes());
    }
  }

  @Test
  void weightedObstacleClearanceCostCanPreferSaferCorridor() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(
            new CoarseGlobalPlannerConfig(
                0.35, 0.8, 5000, 1.0, 0.0, new CoarseRouteCostConfig(1.0, 15.0, 0.0, 0.05)));
    List<Obstacle> obstacles =
        List.of(
            RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.75, 0.85, 1.0, 1.0, 1.0),
            new SoftPenaltyObstacle(new Translation2d(3.0, 3.0), 18.0));

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 2.0),
            new Pose2d(5.0, 2.0, Rotation2d.kZero),
            obstacles,
            0.18,
            0.18,
            6.0,
            4.0);

    assertTrue(waypoint.isPresent());
    assertTrue(
        waypoint.get().getY() < 2.0, "weighted cost should avoid the penalized upper corridor");
    assertTrue(planner.lastStats().routeCostBreakdown().total() > 0.0);
    assertTrue(planner.lastStats().routeCostBreakdown().distanceCost() > 0.0);
    assertTrue(planner.lastStats().routeCostBreakdown().obstacleClearanceCost() > 0.0);
    assertTrue(planner.lastStats().routeClearanceMetrics().minRouteClearanceMeters() > 0.0);
    assertTrue(
        planner.lastStats().routeClearanceMetrics().averageRouteClearanceMeters()
            >= planner.lastStats().routeClearanceMetrics().minRouteClearanceMeters());
  }

  @Test
  void clearanceFieldDynamicObstacleNearCorridorShiftsRouteToAlternateSide() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(
            new CoarseGlobalPlannerConfig(
                0.35, 0.8, 5000, 1.0, 0.0, new CoarseRouteCostConfig(1.0, 12.0, 0.0, 0.05)));
    List<Obstacle> obstacles =
        List.of(
            RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.75, 0.85, 1.0, 1.0, 1.0),
            RectangleObstacle.simple(new Translation2d(3.0, 3.0), 0.20, 0.25, 1.0, 1.0, 1.0));

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 2.0),
            new Pose2d(5.0, 2.0, Rotation2d.kZero),
            obstacles,
            0.18,
            0.18,
            6.0,
            4.0);

    assertTrue(waypoint.isPresent());
    assertTrue(
        waypoint.get().getY() < 2.0,
        "clearance field cost should route away from the upper dynamic obstacle");
    assertTrue(planner.lastStats().routeCostBreakdown().obstacleClearanceCost() > 0.0);
    assertTrue(planner.lastStats().routeClearanceMetrics().minRouteClearanceMeters() > 0.0);
  }

  @Test
  void predictedDynamicEnvelopeBiasesRouteAwayFromNearFutureConflictWithoutHardBlocking() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(
            new CoarseGlobalPlannerConfig(
                0.35, 0.8, 5000, 1.0, 0.0, new CoarseRouteCostConfig(1.0, 18.0, 0.0, 0.05)));
    List<Obstacle> obstacles =
        List.of(
            RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.75, 0.85, 1.0, 1.0, 1.0),
            new PredictedDynamicObstacleEnvelope(
                new Translation2d(3.0, 3.0), 0.45, 0.45, 8.0, 1.0, 1));

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 2.0),
            new Pose2d(5.0, 2.0, Rotation2d.kZero),
            obstacles,
            0.18,
            0.18,
            6.0,
            4.0);

    assertTrue(waypoint.isPresent());
    assertTrue(
        waypoint.get().getY() < 2.0,
        "soft prediction should bias route away from the near-future upper conflict");
    assertTrue(planner.lastStats().routeCostBreakdown().obstacleClearanceCost() > 0.0);
    assertTrue(planner.lastStats().routeClearanceMetrics().minRouteClearanceMeters() > 0.0);
  }

  @Test
  void weightedWallClearanceCostProducesBreakdownForNearWallRoutes() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(
            new CoarseGlobalPlannerConfig(
                0.25, 0.8, 5000, 1.0, 0.0, new CoarseRouteCostConfig(1.0, 0.0, 0.5, 0.05)));

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 0.30),
            new Pose2d(5.0, 0.30, Rotation2d.kZero),
            List.of(),
            0.18,
            0.18,
            6.0,
            4.0);

    assertTrue(waypoint.isPresent());
    assertTrue(planner.lastStats().routeCostBreakdown().wallClearanceCost() > 0.0);
    assertTrue(
        planner.lastStats().routeCostBreakdown().total()
            >= planner.lastStats().routeCostBreakdown().distanceCost());
    assertTrue(planner.lastStats().routeClearanceMetrics().minRouteClearanceMeters() > 0.0);
    assertTrue(
        planner.lastStats().routeClearanceMetrics().averageRouteClearanceMeters()
            >= planner.lastStats().routeClearanceMetrics().minRouteClearanceMeters());
  }

  @Test
  void returnsEmptyWhenGoalCellIsBlocked() {
    CoarseGlobalPlanner planner = deterministicPlanner();
    RectangleObstacle block =
        RectangleObstacle.simple(new Translation2d(5.0, 2.0), 1.0, 1.0, 1.0, 1.0, 1.0);

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 2.0),
            new Pose2d(5.0, 2.0, Rotation2d.kZero),
            List.of(block),
            0.18,
            0.18,
            6.0,
            4.0);

    assertFalse(waypoint.isPresent());
    assertEquals(
        CoarseGlobalPlannerFailureReason.GOAL_BLOCKED, planner.lastStats().failureReason());
  }

  @Test
  void recordsSearchStatsWhenRouteIsFound() {
    CoarseGlobalPlanner planner = deterministicPlanner();

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 1.0),
            new Pose2d(5.0, 3.0, Rotation2d.kZero),
            List.of(),
            0.18,
            0.18,
            6.0,
            4.0);

    assertTrue(waypoint.isPresent());
    assertTrue(planner.lastStats().found());
    assertTrue(planner.lastStats().expandedNodes() > 0);
    assertFalse(planner.lastStats().timedOut());
    assertFalse(planner.lastStats().exhaustedNodeBudget());
    assertEquals(CoarseGlobalPlannerFailureReason.NONE, planner.lastStats().failureReason());
    assertTrue(planner.lastStats().rawPathNodes() >= planner.lastStats().pathNodes());
    assertEquals(
        2, planner.lastStats().pathNodes(), "clear routes should smooth to direct segments");
  }

  @Test
  void rawAndSmoothedPathNodeCountsExposeRoutePostProcessing() {
    CoarseGlobalPlanner planner = deterministicPlanner();
    RectangleObstacle block =
        RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.9, 2.0, 1.0, 1.0, 1.0);

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 2.0),
            new Pose2d(5.0, 2.0, Rotation2d.kZero),
            List.of(block),
            0.18,
            0.18,
            6.0,
            4.0);

    assertTrue(waypoint.isPresent());
    assertTrue(planner.lastStats().rawPathNodes() > planner.lastStats().pathNodes());
    assertTrue(planner.lastStats().pathNodes() >= 2);
  }

  @Test
  void lookaheadStopsBeforeSharpTurnInsteadOfSkippingToRouteEnd() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.35, 2.0, 5000, 1.0));
    RectangleObstacle block =
        RectangleObstacle.simple(new Translation2d(3.0, 2.0), 0.9, 2.0, 1.0, 1.0, 1.0);

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 2.0),
            new Pose2d(5.0, 2.0, Rotation2d.kZero),
            List.of(block),
            0.18,
            0.18,
            6.0,
            4.0);

    assertTrue(waypoint.isPresent());
    assertEquals(
        CoarseGlobalPlannerWaypointReason.BEFORE_SHARP_TURN,
        planner.lastStats().selectedWaypointReason());
    assertTrue(planner.lastStats().selectedWaypointIndex() >= 1);
    assertTrue(waypoint.get().getTranslation().getDistance(new Translation2d(5.0, 2.0)) > 0.5);
  }

  @Test
  void repeatedSimilarRoutesKeepPreviousGlobalWaypointWhenStillValid() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.35, 0.8, 5000, 1.0));
    Pose2d goal = new Pose2d(5.0, 3.0, Rotation2d.kZero);

    var first =
        planner.nextWaypoint(new Translation2d(1.0, 1.0), goal, List.of(), 0.18, 0.18, 6.0, 4.0);
    var second =
        planner.nextWaypoint(new Translation2d(1.05, 1.0), goal, List.of(), 0.18, 0.18, 6.0, 4.0);

    assertTrue(first.isPresent());
    assertTrue(second.isPresent());
    assertEquals(first.get().getX(), second.get().getX(), 1e-9);
    assertEquals(first.get().getY(), second.get().getY(), 1e-9);
    assertEquals(
        CoarseGlobalPlannerWaypointReason.HYSTERESIS_KEEP,
        planner.lastStats().selectedWaypointReason());
  }

  @Test
  void honorsExpandedNodeBudget() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.25, 1.0, 1, 1.0));

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

  @Test
  void clearanceBufferAddsStrategySpecificFieldMargin() {
    CoarseGlobalPlanner noBuffer =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.25, 0.8, 5000, 1.0));
    CoarseGlobalPlanner buffered =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.25, 0.8, 5000, 1.0, 0.25));

    var unbufferedWaypoint =
        noBuffer.nextWaypoint(
            new Translation2d(0.30, 0.30),
            new Pose2d(2.0, 2.0, Rotation2d.kZero),
            List.of(),
            0.18,
            0.18,
            6.0,
            4.0);
    var bufferedWaypoint =
        buffered.nextWaypoint(
            new Translation2d(0.30, 0.30),
            new Pose2d(2.0, 2.0, Rotation2d.kZero),
            List.of(),
            0.18,
            0.18,
            6.0,
            4.0);

    assertTrue(unbufferedWaypoint.isPresent());
    assertFalse(bufferedWaypoint.isPresent());
    assertEquals(
        CoarseGlobalPlannerFailureReason.START_BLOCKED, buffered.lastStats().failureReason());
  }

  @Test
  void wallEdgeRouteIsRejectedWhenClearanceMarginWouldClipFieldBoundary() {
    CoarseGlobalPlanner noBuffer =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.25, 0.8, 5000, 1.0));
    CoarseGlobalPlanner buffered =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.25, 0.8, 5000, 1.0, 0.30));

    var unbufferedWaypoint =
        noBuffer.nextWaypoint(
            new Translation2d(1.0, 0.30),
            new Pose2d(5.0, 0.30, Rotation2d.kZero),
            List.of(),
            0.18,
            0.18,
            6.0,
            4.0);
    var bufferedWaypoint =
        buffered.nextWaypoint(
            new Translation2d(1.0, 0.30),
            new Pose2d(5.0, 0.30, Rotation2d.kZero),
            List.of(),
            0.18,
            0.18,
            6.0,
            4.0);

    assertTrue(unbufferedWaypoint.isPresent());
    assertFalse(bufferedWaypoint.isPresent());
    assertEquals(
        CoarseGlobalPlannerFailureReason.START_BLOCKED, buffered.lastStats().failureReason());
  }

  @Test
  void defaultConfigReadsSystemProperties() {
    String oldCell = System.getProperty("repulsor.fieldplanner.globalFallback.cellMeters");
    try {
      System.setProperty("repulsor.fieldplanner.globalFallback.cellMeters", "0.42");
      assertEquals(0.42, CoarseGlobalPlannerConfig.defaults().cellMeters(), 1e-9);
    } finally {
      if (oldCell == null) {
        System.clearProperty("repulsor.fieldplanner.globalFallback.cellMeters");
      } else {
        System.setProperty("repulsor.fieldplanner.globalFallback.cellMeters", oldCell);
      }
    }
  }

  @Test
  void defaultLikeConfigRoutesRepresentativeRebuilt2026CorridorCases() {
    Rebuilt2026 field = new Rebuilt2026();
    ArrayList<Obstacle> obstacles = new ArrayList<>();
    obstacles.addAll(field.fieldObstacles());
    obstacles.addAll(field.walls());

    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.55, 1.4, 1200, 0.050));
    List<Scenario> scenarios =
        List.of(
            new Scenario(new Translation2d(1.4, 1.0), new Pose2d(7.6, 1.0, Rotation2d.kZero)),
            new Scenario(
                new Translation2d(15.0, field.geometry().widthMeters() - 1.0),
                new Pose2d(9.0, field.geometry().widthMeters() - 1.0, Rotation2d.kZero)));

    for (Scenario scenario : scenarios) {
      var waypoint =
          planner.nextWaypoint(
              scenario.start(),
              scenario.goal(),
              obstacles,
              0.18,
              0.18,
              field.geometry().lengthMeters(),
              field.geometry().widthMeters());

      assertTrue(waypoint.isPresent(), "should route representative Rebuilt2026 case");
      assertFalse(planner.lastStats().timedOut());
      assertFalse(planner.lastStats().exhaustedNodeBudget());
      assertTrue(planner.lastStats().expandedNodes() <= 1200);
    }
  }

  private static CoarseGlobalPlanner deterministicPlanner() {
    return new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.35, 0.8, 5000, 1.0));
  }

  private record Scenario(Translation2d start, Pose2d goal) {}

  private static final class SoftPenaltyObstacle extends Obstacle {
    private final Translation2d center;
    private final double strength;

    private SoftPenaltyObstacle(Translation2d center, double strength) {
      super(strength, true);
      this.center = center;
      this.strength = strength;
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      double distance = Math.max(0.1, position.getDistance(center));
      return new Force(strength / (distance * distance), Rotation2d.kZero);
    }
  }
}
