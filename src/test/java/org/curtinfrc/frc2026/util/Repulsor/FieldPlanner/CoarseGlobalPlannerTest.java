package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
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
}
