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

/**
 * Provides reefscape2025 functionality for the Repulsor field/profile definition layer used to tune
 * Repulsor for a specific game. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class Reefscape2025 implements FieldDefinition {
  /**
   * Configuration value for april tag layout. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  /**
   * Configuration value for field length m. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  public static final double FIELD_LENGTH_M = 17.548;

  /**
   * Configuration value for field width m. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  public static final double FIELD_WIDTH_M = 8.052;

  private final FieldProfileConfig profile;
  private final FieldGeometry geometry;

  private static final double CORNER_CHAMFER = 1.5;

  private static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new TeardropObstacle(new Translation2d(4.49, 4.00), 1.2, 2.2, 1.03, 3.0, 2.0),
          new TeardropObstacle(new Translation2d(13.08, 4.00), 1.2, 2.2, 1.03, 3.0, 2.0));

  /** Returns the reefscape2025 value maintained by this Repulsor component. */
  public Reefscape2025() {
    this(FieldProfileYamlLoader.loadOrDefault("reefscape2025", defaultProfileConfig()));
  }

  /**
   * Creates a reefscape2025 instance with the dependencies and tuning values used by this Repulsor
   * component.
   *
   * @param profile value used by this operation.
   */
  Reefscape2025(FieldProfileConfig profile) {
    this.profile = profile;
    this.geometry = profile.fieldGeometry(FIELD_LENGTH_M, FIELD_WIDTH_M);
  }

  private static FieldProfileConfig defaultProfileConfig() {
    FieldProfileConfig cfg = new FieldProfileConfig();
    cfg.id = "reefscape2025";
    cfg.gameName = "REEFSCAPE";
    cfg.gameYear = 2025;
    cfg.geometry.lengthMeters = FIELD_LENGTH_M;
    cfg.geometry.widthMeters = FIELD_WIDTH_M;

    FieldProfileConfig.ResourceConfig coral = new FieldProfileConfig.ResourceConfig();
    coral.radiusMeters = 0.10;
    coral.unitValue = 1.0;
    coral.sigmaMeters = 0.95;
    cfg.resources.put("coral", coral);
    return cfg;
  }

  /**
   * Returns the field obstacles value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public List<Obstacle> fieldObstacles() {
    return FIELD_OBSTACLES;
  }

  /**
   * Returns the walls value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public List<Obstacle> walls() {
    return List.of(
        new HorizontalObstacle(0.0, 2.0, true),
        new HorizontalObstacle(geometry.widthMeters(), 1.4, false),
        new VerticalObstacle(0.0, 2.0, true),
        new VerticalObstacle(geometry.lengthMeters(), 1.4, false),
        new DiagonalWallObstacle(
            new Translation2d(0.0, CORNER_CHAMFER),
            new Translation2d(CORNER_CHAMFER, 0.0),
            2.0,
            2.0),
        new DiagonalWallObstacle(
            new Translation2d(geometry.lengthMeters() - CORNER_CHAMFER, 0.0),
            new Translation2d(geometry.lengthMeters(), CORNER_CHAMFER),
            2.0,
            2.0),
        new DiagonalWallObstacle(
            new Translation2d(0.0, geometry.widthMeters() - CORNER_CHAMFER),
            new Translation2d(CORNER_CHAMFER, geometry.widthMeters()),
            2.0,
            2.0),
        new DiagonalWallObstacle(
            new Translation2d(geometry.lengthMeters() - CORNER_CHAMFER, geometry.widthMeters()),
            new Translation2d(geometry.lengthMeters(), geometry.widthMeters() - CORNER_CHAMFER),
            2.0,
            2.0));
  }

  /**
   * Builds the WPILib command sequence for the current behaviour context.
   *
   * @param ft value used by this operation.
   * @return game element[] result for build.
   */
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

  /**
   * Returns the get heatmap value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public Heatmap getHeatmap() {
    return Heatmap.builder().build();
  }

  /**
   * Returns the game name value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public String gameName() {
    return profile.gameName;
  }

  /**
   * Returns the game year value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public int gameYear() {
    return profile.gameYear;
  }

  /**
   * Returns the april tag layout value maintained by this Repulsor component.
   *
   * @return april tag field layout result for april tag layout.
   */
  @Override
  public AprilTagFieldLayout aprilTagLayout() {
    return APRIL_TAG_LAYOUT;
  }

  /**
   * Returns the field length meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public double fieldLengthMeters() {
    return geometry.lengthMeters();
  }

  /**
   * Returns the field width meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public double fieldWidthMeters() {
    return geometry.widthMeters();
  }

  /**
   * Returns the geometry value maintained by this Repulsor component.
   *
   * @return field geometry result for geometry.
   */
  @Override
  public FieldGeometry geometry() {
    return geometry;
  }

  /**
   * Returns the default collect setpoint value maintained by this Repulsor component.
   *
   * @return optional repulsor setpoint produced by this operation.
   */
  @Override
  public Optional<RepulsorSetpoint> defaultCollectSetpoint() {
    return Optional.of(
        new RepulsorSetpoint(
            Setpoints.Reefscape2025.LEFT_HP, "coral.station", HeightSetpoint.CORAL_STATION));
  }

  /**
   * Returns the default score setpoint value maintained by this Repulsor component.
   *
   * @return optional repulsor setpoint produced by this operation.
   */
  @Override
  public Optional<RepulsorSetpoint> defaultScoreSetpoint() {
    return Optional.of(
        new RepulsorSetpoint(Setpoints.Reefscape2025.A, "reef.l2", HeightSetpoint.L2));
  }

  /**
   * Updates configure tracker state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param ft value used by this operation.
   */
  @Override
  public void configureTracker(FieldTrackerCore ft) {
    ft.setCollectResourceTypes(profile.resources.keySet());
    for (var entry : profile.resources.entrySet()) {
      FieldProfileConfig.ResourceConfig resource = entry.getValue();
      ft.configureCollectResourceProfile(
          entry.getKey(),
          new ResourceSpec(resource.radiusMeters, resource.unitValue, resource.sigmaMeters));
    }
  }
}
