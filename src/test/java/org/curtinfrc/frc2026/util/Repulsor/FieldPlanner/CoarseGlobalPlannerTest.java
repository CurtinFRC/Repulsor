package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.junit.jupiter.api.Test;

class CoarseGlobalPlannerTest {
  @Test
  void findsWaypointAroundBlockingRectangle() {
    CoarseGlobalPlanner planner = new CoarseGlobalPlanner(0.35, 0.8);
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
  }

  @Test
  void returnsEmptyWhenGoalCellIsBlocked() {
    CoarseGlobalPlanner planner = new CoarseGlobalPlanner(0.35, 0.8);
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
  }
}
