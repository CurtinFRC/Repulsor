package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers.ExtraPathingCollision;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.IntakeFootprint;
import org.junit.jupiter.api.Test;

class FootprintConventionTest {
  private static final double EPS = 1e-9;

  private static double spanX(Translation2d[] rect) {
    double min = Double.POSITIVE_INFINITY;
    double max = Double.NEGATIVE_INFINITY;
    for (Translation2d c : rect) {
      min = Math.min(min, c.getX());
      max = Math.max(max, c.getX());
    }
    return max - min;
  }

  private static double spanY(Translation2d[] rect) {
    double min = Double.POSITIVE_INFINITY;
    double max = Double.NEGATIVE_INFINITY;
    for (Translation2d c : rect) {
      min = Math.min(min, c.getY());
      max = Math.max(max, c.getY());
    }
    return max - min;
  }

  @Test
  void robotRectSpansExactlyFullLengthAndWidthAtHeadingZero() {
    Translation2d[] rect =
        FieldPlanner.robotRect(new Translation2d(2.0, 2.0), Rotation2d.kZero, 0.8, 0.4);
    assertEquals(0.8, spanX(rect), EPS);
    assertEquals(0.4, spanY(rect), EPS);
    assertEquals(1.6, minX(rect), EPS);
    assertEquals(2.4, maxX(rect), EPS);
    assertEquals(1.8, minY(rect), EPS);
    assertEquals(2.2, maxY(rect), EPS);
  }

  @Test
  void robotRectRotatedNinetyDegreesSwapsExtentsWithoutRescaling() {
    Translation2d[] rect =
        FieldPlanner.robotRect(
            new Translation2d(2.0, 2.0), Rotation2d.fromRadians(Math.PI / 2.0), 0.8, 0.4);
    assertEquals(0.4, spanX(rect), 1e-9);
    assertEquals(0.8, spanY(rect), 1e-9);
  }

  @Test
  void axisAlignedRectCornersTreatInputsAsFullSize() {
    Translation2d[] corners =
        ExtraPathingCollision.rectCorners(new Translation2d(2.0, 2.0), 0.8, 0.4);
    assertEquals(0.8, spanX(corners), EPS);
    assertEquals(0.4, spanY(corners), EPS);
  }

  @Test
  void intakeFootprintRobotRectDerivesHalvesFromFullLengths() {
    IntakeFootprint footprint = IntakeFootprint.robotRect(0.8, 0.4);
    assertTrue(footprint.containsPointRobotFrame(new Translation2d(0.39, 0.19)));
    assertTrue(footprint.containsPointRobotFrame(new Translation2d(-0.39, -0.19)));
    assertFalse(footprint.containsPointRobotFrame(new Translation2d(0.41, 0.0)));
    assertFalse(footprint.containsPointRobotFrame(new Translation2d(0.0, 0.21)));
  }

  @Test
  void globalPlannerTreatsCorridorOfFullRobotWidthPlusMarginAsPassable() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.25, 0.8, 5000, 1.0));
    double gap = 0.70;
    List<Obstacle> walls =
        List.of(corridorWall(2.5, 0.0, 2.0 - gap / 2.0), corridorWall(2.5, 2.0 + gap / 2.0, 4.0));

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 2.0),
            new Pose2d(5.0, 2.0, Rotation2d.kZero),
            walls,
            0.60,
            0.60,
            6.0,
            4.0);

    assertTrue(waypoint.isPresent(), "corridor wider than full robot width should be passable");
    assertEquals(CoarseGlobalPlannerFailureReason.NONE, planner.lastStats().failureReason());
  }

  @Test
  void globalPlannerBlocksCorridorSlightlyNarrowerThanFullRobotWidth() {
    CoarseGlobalPlanner planner =
        new CoarseGlobalPlanner(new CoarseGlobalPlannerConfig(0.25, 0.8, 5000, 1.0));
    double gap = 0.50;
    List<Obstacle> walls =
        List.of(corridorWall(2.5, 0.0, 2.0 - gap / 2.0), corridorWall(2.5, 2.0 + gap / 2.0, 4.0));

    var waypoint =
        planner.nextWaypoint(
            new Translation2d(1.0, 2.0),
            new Pose2d(5.0, 2.0, Rotation2d.kZero),
            walls,
            0.60,
            0.60,
            6.0,
            4.0);

    assertFalse(waypoint.isPresent(), "corridor narrower than full robot width must be blocked");
    assertEquals(CoarseGlobalPlannerFailureReason.NO_ROUTE, planner.lastStats().failureReason());
  }

  private static Obstacle corridorWall(double xCenter, double yFrom, double yTo) {
    return RectangleObstacle.simple(
        new Translation2d(xCenter, (yFrom + yTo) / 2.0), 1.0, yTo - yFrom, 1.0, 1.0, 1.0);
  }

  private static double minX(Translation2d[] rect) {
    double min = Double.POSITIVE_INFINITY;
    for (Translation2d c : rect) min = Math.min(min, c.getX());
    return min;
  }

  private static double maxX(Translation2d[] rect) {
    double max = Double.NEGATIVE_INFINITY;
    for (Translation2d c : rect) max = Math.max(max, c.getX());
    return max;
  }

  private static double minY(Translation2d[] rect) {
    double min = Double.POSITIVE_INFINITY;
    for (Translation2d c : rect) min = Math.min(min, c.getY());
    return min;
  }

  private static double maxY(Translation2d[] rect) {
    double max = Double.NEGATIVE_INFINITY;
    for (Translation2d c : rect) max = Math.max(max, c.getY());
    return max;
  }
}
