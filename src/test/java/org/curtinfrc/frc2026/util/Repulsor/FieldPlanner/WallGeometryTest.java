package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldLayoutProvider;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultDriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultTurnTuning;
import org.junit.jupiter.api.Test;

class WallGeometryTest {
  private static final double SMALL_FIELD_LENGTH = 6.0;
  private static final double SMALL_FIELD_WIDTH = 3.0;

  private static final class SmallFieldProvider
      implements FieldLayoutProvider, FieldPlanner.ObstacleProvider {
    private final RectangleObstacle obstacle =
        RectangleObstacle.simple(new Translation2d(3.0, 1.5), 0.9, 0.9, 1.0, 1.0, 1.0);

    @Override
    public GameElement[] build(FieldTrackerCore ft) {
      return new GameElement[0];
    }

    @Override
    public String gameName() {
      return "wall-geometry-test";
    }

    @Override
    public int gameYear() {
      return 2026;
    }

    @Override
    public double fieldLengthMeters() {
      return SMALL_FIELD_LENGTH;
    }

    @Override
    public double fieldWidthMeters() {
      return SMALL_FIELD_WIDTH;
    }

    @Override
    public List<Obstacle> fieldObstacles() {
      return List.of(obstacle);
    }

    @Override
    public List<Obstacle> walls() {
      return List.of();
    }
  }

  @Test
  void plannerWiresFieldGeometryIntoRectangleObstacles() {
    SmallFieldProvider provider = new SmallFieldProvider();
    FieldPlanner planner =
        new FieldPlanner(new DefaultTurnTuning(), new DefaultDriveTuning(), provider);

    assertEquals(1, planner.getObstacles().size());
    RectangleObstacle wired = (RectangleObstacle) planner.getObstacles().get(0);
    assertSame(provider.obstacle, wired);
    assertEquals(SMALL_FIELD_LENGTH, wired.fieldLengthMeters(), 1e-9);
    assertEquals(SMALL_FIELD_WIDTH, wired.fieldWidthMeters(), 1e-9);
  }

  @Test
  void unwiredRectangleObstacleDefaultsToGlobalConstantsGeometry() {
    RectangleObstacle obstacle =
        RectangleObstacle.simple(new Translation2d(3.0, 1.5), 0.9, 0.9, 1.0, 1.0, 1.0);
    assertEquals(Constants.FIELD_LENGTH, obstacle.fieldLengthMeters(), 1e-9);
    assertEquals(Constants.FIELD_WIDTH, obstacle.fieldWidthMeters(), 1e-9);
  }
}
