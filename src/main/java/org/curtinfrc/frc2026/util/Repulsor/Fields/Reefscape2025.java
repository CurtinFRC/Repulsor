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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.DiagonalWallObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.HorizontalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.TeardropObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.VerticalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.Setpoints;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;

public final class Reefscape2025 implements FieldDefinition {
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public static final double FIELD_LENGTH_M = 17.548;
  public static final double FIELD_WIDTH_M = 8.052;

  private static final double CORNER_CHAMFER = 1.5;

  private static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new TeardropObstacle(new Translation2d(4.49, 4.00), 1.2, 2.2, 1.03, 3.0, 2.0),
          new TeardropObstacle(new Translation2d(13.08, 4.00), 1.2, 2.2, 1.03, 3.0, 2.0));

  @Override
  public List<Obstacle> fieldObstacles() {
    return FIELD_OBSTACLES;
  }

  @Override
  public List<Obstacle> walls() {
    return List.of(
        new HorizontalObstacle(0.0, 2.0, true),
        new HorizontalObstacle(FIELD_WIDTH_M, 1.4, false),
        new VerticalObstacle(0.0, 2.0, true),
        new VerticalObstacle(FIELD_LENGTH_M, 1.4, false),
        new DiagonalWallObstacle(
            new Translation2d(0.0, CORNER_CHAMFER),
            new Translation2d(CORNER_CHAMFER, 0.0),
            2.0,
            2.0),
        new DiagonalWallObstacle(
            new Translation2d(FIELD_LENGTH_M - CORNER_CHAMFER, 0.0),
            new Translation2d(FIELD_LENGTH_M, CORNER_CHAMFER),
            2.0,
            2.0),
        new DiagonalWallObstacle(
            new Translation2d(0.0, FIELD_WIDTH_M - CORNER_CHAMFER),
            new Translation2d(CORNER_CHAMFER, FIELD_WIDTH_M),
            2.0,
            2.0),
        new DiagonalWallObstacle(
            new Translation2d(FIELD_LENGTH_M - CORNER_CHAMFER, FIELD_WIDTH_M),
            new Translation2d(FIELD_LENGTH_M, FIELD_WIDTH_M - CORNER_CHAMFER),
            2.0,
            2.0));
  }

  @Override
  public GameElement[] build(FieldTrackerCore ft) {
    var b = new FieldMapBuilder(ft);

    List<Pose3d> blueFaces =
        Arrays.asList(
            new Pose3d(1.00, 0.00, 0, null),
            new Pose3d(0.50, 0.86, 0, null),
            new Pose3d(-0.50, 0.86, 0, null),
            new Pose3d(-1.00, 0.00, 0, null),
            new Pose3d(-0.50, -0.86, 0, null),
            new Pose3d(0.50, -0.86, 0, null));

    List<RepulsorSetpoint> reefSP =
        Arrays.asList(
            new RepulsorSetpoint(Setpoints.Reefscape2025.A, HeightSetpoint.L2),
            new RepulsorSetpoint(Setpoints.Reefscape2025.B, HeightSetpoint.L2),
            new RepulsorSetpoint(Setpoints.Reefscape2025.C, HeightSetpoint.L2),
            new RepulsorSetpoint(Setpoints.Reefscape2025.D, HeightSetpoint.L2),
            new RepulsorSetpoint(Setpoints.Reefscape2025.E, HeightSetpoint.L2),
            new RepulsorSetpoint(Setpoints.Reefscape2025.F, HeightSetpoint.L2));

    b.bulk(
        blueFaces,
        Alliance.kBlue,
        1,
        FieldMapBuilder.small(),
        0,
        go -> true,
        reefSP,
        CategorySpec.kScore);

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(6)
        .pose(new Pose3d(2.50, 3.00, 0, null))
        .primitivePipe(FieldMapBuilder.medium(), 0, 0, 0)
        .filterType("coral")
        .related(
            new RepulsorSetpoint(Setpoints.Reefscape2025.LEFT_HP, HeightSetpoint.CORAL_STATION))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(6)
        .pose(new Pose3d(2.50, -3.00, 0, null))
        .primitivePipe(FieldMapBuilder.medium(), 0, 0, 0)
        .filterType("coral")
        .related(
            new RepulsorSetpoint(Setpoints.Reefscape2025.RIGHT_HP, HeightSetpoint.CORAL_STATION))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(3)
        .pose(new Pose3d(0.75, 3.80, 0, null))
        .category(CategorySpec.kEndgame)
        .add();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(3)
        .pose(new Pose3d(0.75, -3.80, 0, null))
        .category(CategorySpec.kEndgame)
        .add();

    List<Pose3d> redFaces =
        Arrays.asList(
            new Pose3d(15.00, 0.00, 0, null),
            new Pose3d(14.50, 0.86, 0, null),
            new Pose3d(13.50, 0.86, 0, null),
            new Pose3d(13.00, 0.00, 0, null),
            new Pose3d(13.50, -0.86, 0, null),
            new Pose3d(14.50, -0.86, 0, null));

    b.bulk(
        redFaces,
        Alliance.kRed,
        1,
        FieldMapBuilder.small(),
        0,
        go -> true,
        reefSP,
        CategorySpec.kScore);

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(6)
        .pose(new Pose3d(11.50, 3.00, 0, null))
        .primitivePipe(FieldMapBuilder.medium(), 0, 0, 0)
        .filterType("coral")
        .related(
            new RepulsorSetpoint(Setpoints.Reefscape2025.LEFT_HP, HeightSetpoint.CORAL_STATION))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(6)
        .pose(new Pose3d(11.50, -3.00, 0, null))
        .primitivePipe(FieldMapBuilder.medium(), 0, 0, 0)
        .filterType("coral")
        .related(
            new RepulsorSetpoint(Setpoints.Reefscape2025.RIGHT_HP, HeightSetpoint.CORAL_STATION))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(3)
        .pose(new Pose3d(15.75, 3.80, 0, null))
        .category(CategorySpec.kEndgame)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(3)
        .pose(new Pose3d(15.75, -3.80, 0, null))
        .category(CategorySpec.kEndgame)
        .add();

    return b.build();
  }

  @Override
  public Heatmap getHeatmap() {
    return Heatmap.builder().build();
  }

  @Override
  public String gameName() {
    return "REEFSCAPE";
  }

  @Override
  public int gameYear() {
    return 2025;
  }

  @Override
  public AprilTagFieldLayout aprilTagLayout() {
    return APRIL_TAG_LAYOUT;
  }

  @Override
  public double fieldLengthMeters() {
    return FIELD_LENGTH_M;
  }

  @Override
  public double fieldWidthMeters() {
    return FIELD_WIDTH_M;
  }

  @Override
  public Optional<RepulsorSetpoint> defaultCollectSetpoint() {
    return Optional.of(
        new RepulsorSetpoint(Setpoints.Reefscape2025.LEFT_HP, HeightSetpoint.CORAL_STATION));
  }

  @Override
  public Optional<RepulsorSetpoint> defaultScoreSetpoint() {
    return Optional.of(new RepulsorSetpoint(Setpoints.Reefscape2025.A, HeightSetpoint.L2));
  }

  @Override
  public void configureTracker(FieldTrackerCore ft) {
    ft.setCollectResourceTypes(Set.of("coral"));
    ft.configureCollectResourceProfile("coral", new ResourceSpec(0.10, 1.0, 0.95));
  }
}
