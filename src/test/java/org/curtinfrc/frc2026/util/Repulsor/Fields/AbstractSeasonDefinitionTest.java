/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.HorizontalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.SquareObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateLocalAccess;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.CollectPlannerTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class AbstractSeasonDefinitionTest {
  @AfterEach
  void clearInjectedPredictiveProfiles() {
    PredictiveFieldStateLocalAccess.clearDefaultCollectionProfile();
    PredictiveFieldStateLocalAccess.clearDefaultRecoveryProfile();
  }

  @Test
  void geometryFlowsFromConstructorParams() {
    MinimalSeason season = new MinimalSeason();
    assertEquals(new FieldGeometry(12.0, 8.0), season.geometry());
    assertEquals(12.0, season.fieldLengthMeters(), 1e-9);
    assertEquals(8.0, season.fieldWidthMeters(), 1e-9);
    assertEquals(new Translation2d(6.0, 4.0), season.geometry().center());
    assertNotNull(season.aprilTagLayout());
    assertEquals("TEST", season.gameName());
    assertEquals(2099, season.gameYear());
    assertNull(season.getHeatmap());
  }

  @Test
  void trackerConstructionFlowsGeometryAndResourceWiring() {
    MinimalSeason season = new MinimalSeason();
    FieldTrackerCore tracker = new FieldTrackerCore(season);

    assertEquals(season.geometry(), tracker.getPredictor().getFieldGeometry());
    assertTrue(tracker.getPredictor().isCollectResourceType("widget"));
    assertFalse(tracker.getPredictor().isCollectResourceType("unknown"));
    assertEquals(CollectPlannerTuning.defaults(), tracker.collectPlannerTuning());
    assertEquals(2, tracker.getFieldMap().length);

    ResourceSpec spec = season.resourceSpec();
    assertEquals(0.09, spec.radiusM, 1e-9);
    assertEquals(1.0, spec.unitValue, 1e-9);
    assertEquals(0.90, spec.sigmaM, 1e-9);
  }

  @Test
  void configureTrackerInstallsPredictiveProfiles() {
    MinimalSeason season = new MinimalSeason();
    new FieldTrackerCore(season);

    var collection = PredictiveFieldStateLocalAccess.defaultCollectionProfile();
    assertEquals("widget", collection.defaultResourceType());
    assertEquals(season.geometry(), collection.fieldGeometry());

    var recovery = PredictiveFieldStateLocalAccess.defaultRecoveryProfile();
    assertEquals("widget", recovery.resourceType());
    assertEquals(season.geometry(), recovery.fieldGeometry());
    assertEquals(0.5, recovery.allianceZoneXMaxFraction(), 1e-9);
  }

  @Test
  void obstacleAndWallHooksAreEmptyByDefaultAndMutablePrePlannerUse() {
    MinimalSeason minimal = new MinimalSeason();
    assertTrue(minimal.fieldObstacles().isEmpty());
    assertTrue(minimal.walls().isEmpty());
    assertTrue(minimal.fieldObstaclesHook().isEmpty());
    assertTrue(minimal.wallsHook().isEmpty());
    assertTrue(
        minimal
            .fieldObstaclesHook()
            .add(new SquareObstacle(new Translation2d(6.0, 4.0), 1.0, 3.0, 2.0)));

    HookedSeason hooked = new HookedSeason();
    assertTrue(
        hooked
            .fieldObstaclesHook()
            .add(new SquareObstacle(new Translation2d(6.0, 4.0), 1.0, 3.0, 2.0)));
    assertTrue(hooked.wallsHook().add(new HorizontalObstacle(0.0, 2.0, true)));
    assertEquals(1, hooked.fieldObstacles().size());
    assertEquals(1, hooked.walls().size());
    assertTrue(hooked.validateProfile().isEmpty());
  }

  @Test
  void actionProfileExposesGenericTransferRoleWithPlaceholderTarget() {
    MinimalSeason season = new MinimalSeason();
    FieldActionProfile profile = season.actionProfile();
    assertEquals(1, profile.projectileShots().size());

    var shot = profile.transferProjectileShot();
    assertTrue(shot.isPresent());
    assertEquals("transfer_to_score", shot.get().id());
    assertEquals(FieldActionProfile.ActionRole.TRANSFER_TO_SCORE, season.projectileActionRole());
    assertEquals(new Translation2d(3.0, 4.0), shot.get().target(DriverStation.Alliance.Blue));
    assertEquals(new Translation2d(9.0, 4.0), shot.get().target(DriverStation.Alliance.Red));
    assertEquals(HeightSetpoint.NONE, shot.get().routeMechanismSetpoint());
    assertEquals("none", shot.get().routeLevel());
    assertFalse(shot.get().movingShotEnabled());
    assertNotNull(shot.get().gamePiecePhysics());
    assertNotNull(shot.get().constraints());
  }

  @Test
  void customActionAndSetpointsResolveWhenProvided() {
    HookedSeason season = new HookedSeason();

    var shot = season.actionProfile().projectileShot("customTransfer");
    assertTrue(shot.isPresent());
    assertEquals(new Translation2d(6.0, 4.0), shot.get().target(DriverStation.Alliance.Blue));
    assertEquals(new Translation2d(6.0, 4.0), shot.get().target(DriverStation.Alliance.Red));
    assertEquals(HeightSetpoint.L2, shot.get().routeMechanismSetpoint());
    assertEquals("l2", shot.get().routeLevel());

    var score = season.defaultScoreSetpoint();
    assertTrue(score.isPresent());
    RepulsorSetpoint scoreSp = score.get();
    assertEquals(new Pose2d(10.5, 6.5, Rotation2d.kZero), scoreSp.getBlue(SetpointContext.EMPTY));
    assertEquals("score.target", scoreSp.levelId());
    assertEquals(SetpointType.kScore, scoreSp.point().type());

    var collect = season.defaultCollectSetpoint();
    assertTrue(collect.isPresent());
    assertEquals(1.5, collect.get().getBlue(SetpointContext.EMPTY).getX(), 1e-9);
    assertEquals("collect.station", collect.get().levelId());
    assertEquals(SetpointType.kHumanPlayer, collect.get().point().type());

    assertTrue(season.validateProfile().isEmpty());
  }

  @Test
  void defaultSetpointsAreEmptyWithoutCtorPoses() {
    MinimalSeason season = new MinimalSeason();
    assertTrue(season.defaultScoreSetpoint().isEmpty());
    assertTrue(season.defaultCollectSetpoint().isEmpty());
  }

  @Test
  void constructorRejectsInvalidGeometry() {
    assertThrows(IllegalArgumentException.class, () -> new BadGeometrySeason());
  }

  private static final class MinimalSeason extends AbstractSeasonDefinition {
    private MinimalSeason() {
      super(
          "TEST",
          2099,
          12.0,
          8.0,
          AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
          "widget",
          0.09,
          1.0,
          0.90,
          0.5);
    }

    @Override
    public GameElement[] build(FieldTrackerCore ft) {
      return new FieldMapBuilder(ft)
          .begin()
          .alliance(Alliance.kBlue)
          .capacity(2)
          .pose(new Pose3d(9.0, 4.0, 0.0, null))
          .primitivePipe(FieldMapBuilder.small(), 0, 0, 0)
          .category(FieldMapBuilder.CategorySpec.kScore)
          .add()
          .begin()
          .alliance(Alliance.kBlue)
          .capacity(4)
          .pose(new Pose3d(1.5, 1.0, 0.0, null))
          .primitivePipe(FieldMapBuilder.medium(), 0, 0, 0)
          .filterType("widget")
          .category(FieldMapBuilder.CategorySpec.kCollect)
          .add()
          .build();
    }
  }

  private static final class HookedSeason extends AbstractSeasonDefinition {
    private final List<Obstacle> obstacles = new ArrayList<>();
    private final List<Obstacle> walls = new ArrayList<>();

    private HookedSeason() {
      super(
          "TEST",
          2099,
          12.0,
          8.0,
          null,
          "widget",
          0.09,
          1.0,
          0.90,
          0.5,
          "customTransfer",
          FieldActionProfile.ActionRole.TRANSFER_TO_SCORE,
          alliance -> new Translation2d(6.0, 4.0),
          0.5,
          HeightSetpoint.L2,
          new Pose2d(10.5, 6.5, Rotation2d.kZero),
          new Pose2d(1.5, 6.5, Rotation2d.kZero));
    }

    @Override
    protected List<Obstacle> fieldObstaclesHook() {
      return obstacles;
    }

    @Override
    protected List<Obstacle> wallsHook() {
      return walls;
    }

    @Override
    public GameElement[] build(FieldTrackerCore ft) {
      return new FieldMapBuilder(ft).build();
    }
  }

  private static final class BadGeometrySeason extends AbstractSeasonDefinition {
    private BadGeometrySeason() {
      super("BAD", 2099, 12.0, 0.0, null, "widget", 0.09, 1.0, 0.9, 0.5);
    }

    @Override
    public GameElement[] build(FieldTrackerCore ft) {
      return new GameElement[0];
    }
  }
}
