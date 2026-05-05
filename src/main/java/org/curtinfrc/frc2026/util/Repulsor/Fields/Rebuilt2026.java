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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlannerRuntimeConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.CorridorCenterlineRail;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.HorizontalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.VerticalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldActionProfile.ActionRole;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldActionProfile.ProjectileShotAction;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointUtil;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.Setpoints;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.Specific._Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;

/**
 * Provides rebuilt2026 functionality for the Repulsor field/profile definition layer used to tune
 * Repulsor for a specific game. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class Rebuilt2026 implements FieldDefinition {
  /**
   * Configuration value for april tag layout. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /**
   * Configuration value for field length m. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  public static final double FIELD_LENGTH_M = 16.540988;

  public static final double FIELD_WIDTH_M = APRIL_TAG_LAYOUT.getFieldWidth();
  private final FieldProfileConfig profile;
  private final FieldGeometry geometry;

  private static final double CORNER_CHAMFER = 0;

  private static final double GRID_Z_MIN_M = -0.05;
  private static final double GRID_Z_MAX_M = 0.35;

  private static final double GRID_CELL_M = 0.75;
  private static final String GAME_PIECE_ID_FUEL = "fuel";
  private static final double HUB_OPENING_FRONT_EDGE_HEIGHT_M = 1.43;
  private static final Constraints DEFAULT_HUB_SHOT_CONSTRAINTS =
      new Constraints(0, 30, 60, 90.0, Constraints.ShotStyle.ARC);

  /** Returns the rebuilt2026 value maintained by this Repulsor component. */
  public Rebuilt2026() {
    this(FieldProfileYamlLoader.loadOrDefault("rebuilt2026", defaultProfileConfig()));
  }

  /**
   * Creates a rebuilt2026 instance with the dependencies and tuning values used by this Repulsor
   * component.
   *
   * @param profile value used by this operation.
   */
  Rebuilt2026(FieldProfileConfig profile) {
    this.profile = profile;
    this.geometry = profile.fieldGeometry(FIELD_LENGTH_M, FIELD_WIDTH_M);
  }

  private static FieldProfileConfig defaultProfileConfig() {
    FieldProfileConfig cfg = new FieldProfileConfig();
    cfg.id = "rebuilt2026";
    cfg.gameName = "REBUILT";
    cfg.gameYear = 2026;
    cfg.geometry.lengthMeters = FIELD_LENGTH_M;
    cfg.geometry.widthMeters = FIELD_WIDTH_M;

    FieldProfileConfig.ResourceConfig fuel = new FieldProfileConfig.ResourceConfig();
    fuel.radiusMeters = 0.075;
    fuel.unitValue = 1.0;
    fuel.sigmaMeters = 0.95;
    cfg.resources.put("fuel", fuel);

    FieldProfileConfig.ProjectileShotConfig fuelTransfer =
        new FieldProfileConfig.ProjectileShotConfig();
    fuelTransfer.enabled = true;
    fuelTransfer.role = ActionRole.TRANSFER_TO_SCORE.name();
    fuelTransfer.gamePieceId = GAME_PIECE_ID_FUEL;
    fuelTransfer.target.kind = "alliance_side";
    fuelTransfer.target.blueXMeters = FIELD_LENGTH_M * 0.25;
    fuelTransfer.target.blueYMeters = FIELD_WIDTH_M * 0.5;
    fuelTransfer.targetHeightMeters = 0.35;
    configureDefaultShotConstraints(fuelTransfer, 4.0, 30.0, 5.0, 45.0, "DIRECT");
    fuelTransfer.routeLevel = "alliance.transfer";
    fuelTransfer.routeMechanismSetpoint = HeightSetpoint.NET.name();
    fuelTransfer.behindTargetMeters = 2.3;
    fuelTransfer.lateralOffsetsMeters = new double[] {0.0, 0.45, -0.45, 0.9, -0.9};
    fuelTransfer.fieldMarginMeters = 0.28;
    configureDefaultMovingShot(fuelTransfer);
    fuelTransfer.fallbackGamePiece.massKg = 0.27;
    fuelTransfer.fallbackGamePiece.crossSectionAreaM2 = 0.014;
    fuelTransfer.fallbackGamePiece.dragCoefficient = 0.95;
    cfg.projectileShots.put("fuelTransferToAllianceSide", fuelTransfer);

    FieldProfileConfig.ProjectileShotConfig fuelScore =
        new FieldProfileConfig.ProjectileShotConfig();
    fuelScore.enabled = true;
    fuelScore.role = ActionRole.SCORE.name();
    fuelScore.gamePieceId = GAME_PIECE_ID_FUEL;
    fuelScore.target.kind = "hub";
    fuelScore.targetHeightMeters = HUB_OPENING_FRONT_EDGE_HEIGHT_M;
    configureDefaultShotConstraints(
        fuelScore,
        DEFAULT_HUB_SHOT_CONSTRAINTS.minLaunchSpeedMetersPerSecond(),
        DEFAULT_HUB_SHOT_CONSTRAINTS.maxLaunchSpeedMetersPerSecond(),
        DEFAULT_HUB_SHOT_CONSTRAINTS.minLaunchAngleDeg(),
        DEFAULT_HUB_SHOT_CONSTRAINTS.maxLaunchAngleDeg(),
        DEFAULT_HUB_SHOT_CONSTRAINTS.shotStyle().name());
    fuelScore.routeLevel = "hub";
    fuelScore.routeMechanismSetpoint = HeightSetpoint.NET.name();
    fuelScore.behindTargetMeters = 2.95;
    fuelScore.lateralOffsetsMeters = new double[] {0.0, 0.45, -0.45, 0.9, -0.9};
    fuelScore.fieldMarginMeters = 0.28;
    configureDefaultMovingShot(fuelScore);
    fuelScore.fallbackGamePiece.massKg = 0.27;
    fuelScore.fallbackGamePiece.crossSectionAreaM2 = 0.014;
    fuelScore.fallbackGamePiece.dragCoefficient = 0.95;
    cfg.projectileShots.put("fuelScoreHub", fuelScore);

    FieldProfileConfig.RebuiltCorridorConfig corridor = cfg.rebuiltCorridor;
    corridor.rectWidthMeters = 0.5929315;
    corridor.rectHeightMeters = 5.711800;
    corridor.rectOffsetFromCenterMeters = 3.648981;
    corridor.edgeOffsetMeters = 0.48;
    corridor.rectStrength = 4.2;
    corridor.rectRangeXMeters = 1.55;
    corridor.rectRangeYMeters = 2.0;
    corridor.biasStrength = 0.3;
    corridor.biasRangeMeters = 1.2;
    corridor.bypassStrengthScale = 1.2;
    corridor.bypassRangeMeters = 1.4;
    corridor.sidePullDxMeters = 1.0;
    corridor.sideBiasStrengthScale = 0.35;
    corridor.sideBiasRangeScale = 0.70;
    corridor.sideBypassStrengthScale = 0.75;
    corridor.sideBypassRangeScale = 0.75;
    corridor.railXWindowMeters = 1.4;
    corridor.railMinHalfWidthMeters = 0.24;
    corridor.railHalfWidthGapScale = 0.42;
    corridor.railStrength = 1.8;
    corridor.railMaxForce = 2.6;
    corridor.centerRailMinWindowMeters = 1.2;
    corridor.centerRailWindowScale = 0.45;
    corridor.outerRailXOffsetScale = 0.9;
    corridor.outerRailWindowMeters = 1.0;
    return cfg;
  }

  private static double positive(Double value, double fallback) {
    return value != null && Double.isFinite(value) && value > 0.0 ? value : fallback;
  }

  @Override
  public FieldPlannerWaypointConfig waypointConfig() {
    return profile.waypointing == null
        ? FieldPlannerWaypointConfig.defaults()
        : profile.waypointing.toFieldPlannerWaypointConfig();
  }

  @Override
  public FieldPlannerRuntimeConfig fieldPlannerRuntimeConfig() {
    return profile.plannerRuntime == null
        ? FieldPlannerRuntimeConfig.defaults()
        : profile.plannerRuntime.toFieldPlannerRuntimeConfig();
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

    Pose2d blueOutpost = Setpoints.Rebuilt2026.OUTPOST_COLLECT.approximateBluePose();
    Pose2d redOutpost = Setpoints.Rebuilt2026.OUTPOST_COLLECT.approximateRedPose();

    Pose2d blueHub = Setpoints.Rebuilt2026.HUB_SHOOT.approximateBluePose();
    Pose2d redHub = Setpoints.Rebuilt2026.HUB_SHOOT.approximateRedPose();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(999)
        .pose(new Pose3d(blueHub.getX(), blueHub.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET))
        .category(CategorySpec.kScore)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(999)
        .pose(new Pose3d(redHub.getX(), redHub.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET))
        .category(CategorySpec.kScore)
        .add();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(999)
        .pose(new Pose3d(blueOutpost.getX(), blueOutpost.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(999)
        .pose(new Pose3d(redOutpost.getX(), redOutpost.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE))
        .category(CategorySpec.kCollect)
        .add();

    double cx = geometry.lengthMeters() * 0.5;
    double cy = geometry.widthMeters() * 0.5;

    double x0 = 0.0;
    double x1 = geometry.lengthMeters();
    double y0 = 0.0;
    double y1 = geometry.widthMeters();

    int nx = (int) Math.floor((x1 - x0) / GRID_CELL_M);
    int ny = (int) Math.floor((y1 - y0) / GRID_CELL_M);

    for (int ix = 0; ix <= nx; ix++) {
      double x = x0 + ix * GRID_CELL_M + GRID_CELL_M * 0.5;
      for (int iy = 0; iy <= ny; iy++) {
        double y = y0 + iy * GRID_CELL_M + GRID_CELL_M * 0.5;

        b.begin()
            .alliance(Alliance.kBlue)
            .capacity(999)
            .pose(new Pose3d(x, y, 0, null))
            .primitiveFloorSquare(GRID_CELL_M, GRID_Z_MIN_M, GRID_Z_MAX_M)
            .filterType("fuel")
            .category(CategorySpec.kCollect)
            .add();

        b.begin()
            .alliance(Alliance.kRed)
            .capacity(999)
            .pose(new Pose3d(x, y, 0, null))
            .primitiveFloorSquare(GRID_CELL_M, GRID_Z_MIN_M, GRID_Z_MAX_M)
            .filterType("fuel")
            .category(CategorySpec.kCollect)
            .add();
      }
    }

    return b.build();
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
            Setpoints.Rebuilt2026.CENTER_COLLECT, "center.collect", HeightSetpoint.NONE));
  }

  /**
   * Returns the default score setpoint value maintained by this Repulsor component.
   *
   * @return optional repulsor setpoint produced by this operation.
   */
  @Override
  public Optional<RepulsorSetpoint> defaultScoreSetpoint() {
    return Optional.of(
        new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, "hub", HeightSetpoint.NET));
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
    ft.configurePredictiveRanking(profile.predictiveRanking.toPredictiveRankingConfig());
    ft.configureObjectiveSelection(
        profile.objectiveSelection == null
            ? null
            : profile.objectiveSelection.toObjectiveSelectionConfig());
    ft.setCollectResourceTypes(profile.resources.keySet());
    for (var entry : profile.resources.entrySet()) {
      FieldProfileConfig.ResourceConfig resource = entry.getValue();
      ft.configureCollectResourceProfile(
          entry.getKey(),
          new ResourceSpec(resource.radiusMeters, resource.unitValue, resource.sigmaMeters));
    }
  }

  /**
   * Returns the action profile value maintained by this Repulsor component.
   *
   * @return field action profile result for action profile.
   */
  @Override
  public FieldActionProfile actionProfile() {
    var actions = new java.util.LinkedHashMap<String, ProjectileShotAction>();
    for (var entry : profile.projectileShots.entrySet()) {
      FieldProfileConfig.ProjectileShotConfig shot = entry.getValue();
      if (shot == null || !Boolean.TRUE.equals(shot.enabled)) {
        continue;
      }
      actions.put(entry.getKey(), projectileShotAction(entry.getKey(), shot));
    }

    FieldProfileConfig.ShuttleShotConfig legacy = profile.shuttleShot;
    if (legacy != null && Boolean.TRUE.equals(legacy.enabled)) {
      actions.putIfAbsent("legacyShuttleShot", projectileShotAction("legacyShuttleShot", legacy));
    }

    return new FieldActionProfile(actions);
  }

  private ProjectileShotAction projectileShotAction(
      String id, FieldProfileConfig.ProjectileShotConfig shot) {
    return new ProjectileShotAction(
        id,
        actionRole(shot.role),
        targetForAlliance(shot),
        loadProjectileGamePiece(shot),
        shot.targetHeightMeters,
        shot.constraints != null
            ? shot.constraints.constraints(DEFAULT_HUB_SHOT_CONSTRAINTS)
            : DEFAULT_HUB_SHOT_CONSTRAINTS,
        shot.routeLevel,
        routeMechanismSetpoint(shot.routeMechanismSetpoint),
        shot.behindTargetMeters,
        shot.lateralOffsetsMeters,
        shot.fieldMarginMeters,
        shot.movingShot == null || !Boolean.FALSE.equals(shot.movingShot.enabled),
        shot.movingShot != null ? shot.movingShot.solverConfig() : null);
  }

  private static void configureDefaultShotConstraints(
      FieldProfileConfig.ProjectileShotConfig shot,
      double minSpeed,
      double maxSpeed,
      double minAngleDeg,
      double maxAngleDeg,
      String shotStyle) {
    shot.constraints.minLaunchSpeedMetersPerSecond = minSpeed;
    shot.constraints.maxLaunchSpeedMetersPerSecond = maxSpeed;
    shot.constraints.minLaunchAngleDegrees = minAngleDeg;
    shot.constraints.maxLaunchAngleDegrees = maxAngleDeg;
    shot.constraints.shotStyle = shotStyle;
  }

  private static void configureDefaultMovingShot(FieldProfileConfig.ProjectileShotConfig shot) {
    shot.movingShot.enabled = true;
    shot.movingShot.releaseLatencySeconds = 0.08;
    shot.movingShot.minFlightPredictionSeconds = 0.10;
    shot.movingShot.maxFlightPredictionSeconds = 0.45;
    shot.movingShot.defaultFlightPredictionSeconds = 0.18;
    shot.movingShot.maxCompensatedSpeedMetersPerSecond = 4.5;
    shot.movingShot.maxReleaseSpeedMetersPerSecond = 4.5;
    shot.movingShot.yawToleranceDegrees = 13.0;
    shot.movingShot.maxVerticalErrorMeters = 0.20;
    shot.movingShot.iterations = 3;
  }

  private Function<DriverStation.Alliance, Translation2d> targetForAlliance(
      FieldProfileConfig.ProjectileShotConfig shot) {
    return alliance -> projectileTarget(shot, alliance);
  }

  private Translation2d projectileTarget(
      FieldProfileConfig.ProjectileShotConfig shot, DriverStation.Alliance alliance) {
    FieldProfileConfig.TargetConfig target = shot.target;
    String kind = target == null || target.kind == null ? "hub" : target.kind.trim().toLowerCase();
    if ("hub".equals(kind)) {
      return _Rebuilt2026.hubAimpointForAlliance(alliance);
    }

    Translation2d blueTarget;
    if (target != null && target.blueXMeters != null && target.blueYMeters != null) {
      blueTarget = new Translation2d(target.blueXMeters, target.blueYMeters);
    } else if ("alliance_side".equals(kind)) {
      blueTarget = new Translation2d(geometry.lengthMeters() * 0.25, geometry.widthMeters() * 0.5);
    } else {
      blueTarget = geometry.center();
    }
    return alliance == DriverStation.Alliance.Red ? SetpointUtil.flipToRed(blueTarget) : blueTarget;
  }

  private static ActionRole actionRole(String value) {
    if (value == null || value.isBlank()) return ActionRole.OTHER;
    try {
      return ActionRole.valueOf(value.trim().toUpperCase());
    } catch (IllegalArgumentException ex) {
      return ActionRole.OTHER;
    }
  }

  private static HeightSetpoint routeMechanismSetpoint(String value) {
    if (value == null || value.isBlank()) return HeightSetpoint.NET;
    try {
      return HeightSetpoint.valueOf(value.trim().toUpperCase());
    } catch (IllegalArgumentException ex) {
      return HeightSetpoint.NET;
    }
  }

  private static GamePiecePhysics loadProjectileGamePiece(
      FieldProfileConfig.ProjectileShotConfig cfg) {
    try {
      String gamePieceId =
          cfg.gamePieceId == null || cfg.gamePieceId.isBlank()
              ? GAME_PIECE_ID_FUEL
              : cfg.gamePieceId.trim();
      return DragShotPlanner.loadGamePieceFromDeployYaml(gamePieceId);
    } catch (Throwable ignored) {
      FieldProfileConfig.GamePiecePhysicsConfig fallback = cfg.fallbackGamePiece;
      return new GamePiecePhysics() {
        @Override
        public double massKg() {
          return fallback.massKg;
        }

        @Override
        public double crossSectionAreaM2() {
          return fallback.crossSectionAreaM2;
        }

        @Override
        public double dragCoefficient() {
          return fallback.dragCoefficient;
        }
      };
    }
  }

  /**
   * Returns the field obstacles value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public List<Obstacle> fieldObstacles() {
    // double maxRangeY = 1;
    // double maxRangeX = 1.2;
    FieldProfileConfig.RebuiltCorridorConfig corridor = profile.rebuiltCorridor;
    double fieldLength = geometry.lengthMeters();
    double fieldWidth = geometry.widthMeters();

    double rectWidth = positive(corridor.rectWidthMeters, 0.5929315);
    double rectHeight = positive(corridor.rectHeightMeters, 5.711800);
    double rectHalfY = rectHeight * 0.5;

    double rectOffset = positive(corridor.rectOffsetFromCenterMeters, 3.648981);
    double leftRectX = (fieldLength / 2) - rectOffset; // 2.5 meters 34 degrees 16 ms
    double rightRectX = (fieldLength / 2) + rectOffset;
    double rectCy = fieldWidth / 2;

    double gapHeight = Math.max(0.0, (fieldWidth * 0.5) - rectHalfY);
    double gapTopY = fieldWidth - (gapHeight * 0.5);
    double gapBottomY = gapHeight * 0.5;

    double rectStrength = positive(corridor.rectStrength, 4.2);
    double rectRangeX = positive(corridor.rectRangeXMeters, 1.55);
    double rectRangeY = positive(corridor.rectRangeYMeters, 2.0);

    double biasStrength = positive(corridor.biasStrength, 0.3);
    double biasRange = positive(corridor.biasRangeMeters, 1.2);

    double bypassStrengthScale = positive(corridor.bypassStrengthScale, 1.2);
    double bypassRange = positive(corridor.bypassRangeMeters, 1.4);

    double edgeOffset = positive(corridor.edgeOffsetMeters, 0.48);

    double rectHalfX = rectWidth * 0.5;
    Translation2d[] leftGate =
        new Translation2d[] {
          new Translation2d(leftRectX - rectHalfX, rectCy - rectHalfY),
          new Translation2d(leftRectX + rectHalfX, rectCy - rectHalfY),
          new Translation2d(leftRectX + rectHalfX, rectCy + rectHalfY),
          new Translation2d(leftRectX - rectHalfX, rectCy + rectHalfY)
        };
    Translation2d[] rightGate =
        new Translation2d[] {
          new Translation2d(rightRectX - rectHalfX, rectCy - rectHalfY),
          new Translation2d(rightRectX + rectHalfX, rectCy - rectHalfY),
          new Translation2d(rightRectX + rectHalfX, rectCy + rectHalfY),
          new Translation2d(rightRectX - rectHalfX, rectCy + rectHalfY)
        };

    double leftInsideX = leftRectX + rectHalfX + edgeOffset;
    double rightInsideX = rightRectX - rectHalfX - edgeOffset;

    double sidePullDx = positive(corridor.sidePullDxMeters, 1.0);

    double leftPullXOut = leftRectX - sidePullDx;
    double rightPullXOut = rightRectX + sidePullDx;

    double sideBiasStrength = biasStrength * positive(corridor.sideBiasStrengthScale, 0.35);
    double sideBiasRange = biasRange * positive(corridor.sideBiasRangeScale, 0.70);

    double sideBypassStrengthScale =
        bypassStrengthScale * positive(corridor.sideBypassStrengthScale, 0.75);
    double sideBypassRange = bypassRange * positive(corridor.sideBypassRangeScale, 0.75);

    double leftPullXIn = leftRectX + sidePullDx;

    double rightPullXIn = rightRectX - sidePullDx;

    double railXWindow = positive(corridor.railXWindowMeters, 1.4);
    double corridorHalfWidthGuess =
        Math.max(
            positive(corridor.railMinHalfWidthMeters, 0.24),
            gapHeight * positive(corridor.railHalfWidthGapScale, 0.42));
    double railStrength = positive(corridor.railStrength, 1.8);
    double railMaxForce = positive(corridor.railMaxForce, 2.6);
    double centerRailX = 0.5 * (leftRectX + rightRectX);
    double centerRailWindow =
        Math.max(
            positive(corridor.centerRailMinWindowMeters, 1.2),
            (rightRectX - leftRectX) * positive(corridor.centerRailWindowScale, 0.45));
    double outerRailXOffset = sidePullDx * positive(corridor.outerRailXOffsetScale, 0.9);
    double outerRailWindow = positive(corridor.outerRailWindowMeters, 1.0);

    return List.of(
        new RectangleObstacle(
            new Translation2d(leftRectX, rectCy),
            rectWidth,
            rectHeight,
            rectStrength,
            rectRangeX,
            rectRangeY),
        new RectangleObstacle(
            new Translation2d(rightRectX, rectCy),
            rectWidth,
            rectHeight,
            rectStrength,
            rectRangeX,
            rectRangeY),
        new GatedAttractorObstacle(
            new Translation2d(leftRectX, gapTopY),
            biasStrength,
            biasRange,
            leftGate,
            new Translation2d(leftInsideX, gapTopY),
            bypassStrengthScale,
            bypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(leftRectX, gapBottomY),
            biasStrength,
            biasRange,
            leftGate,
            new Translation2d(leftInsideX, gapBottomY),
            bypassStrengthScale,
            bypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(rightRectX, gapTopY),
            biasStrength,
            biasRange,
            rightGate,
            new Translation2d(rightInsideX, gapTopY),
            bypassStrengthScale,
            bypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(rightRectX, gapBottomY),
            biasStrength,
            biasRange,
            rightGate,
            new Translation2d(rightInsideX, gapBottomY),
            bypassStrengthScale,
            bypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(leftPullXIn, gapTopY),
            sideBiasStrength,
            sideBiasRange,
            leftGate,
            new Translation2d(leftInsideX, gapTopY),
            sideBypassStrengthScale,
            sideBypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(leftPullXOut, gapTopY),
            sideBiasStrength,
            sideBiasRange,
            leftGate,
            new Translation2d(leftInsideX, gapTopY),
            sideBypassStrengthScale,
            sideBypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(leftPullXIn, gapBottomY),
            sideBiasStrength,
            sideBiasRange,
            leftGate,
            new Translation2d(leftInsideX, gapBottomY),
            sideBypassStrengthScale,
            sideBypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(leftPullXOut, gapBottomY),
            sideBiasStrength,
            sideBiasRange,
            leftGate,
            new Translation2d(leftInsideX, gapBottomY),
            sideBypassStrengthScale,
            sideBypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(rightPullXIn, gapTopY),
            sideBiasStrength,
            sideBiasRange,
            rightGate,
            new Translation2d(rightInsideX, gapTopY),
            sideBypassStrengthScale,
            sideBypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(rightPullXOut, gapTopY),
            sideBiasStrength,
            sideBiasRange,
            rightGate,
            new Translation2d(rightInsideX, gapTopY),
            sideBypassStrengthScale,
            sideBypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(rightPullXIn, gapBottomY),
            sideBiasStrength,
            sideBiasRange,
            rightGate,
            new Translation2d(rightInsideX, gapBottomY),
            sideBypassStrengthScale,
            sideBypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(rightPullXOut, gapBottomY),
            sideBiasStrength,
            sideBiasRange,
            rightGate,
            new Translation2d(rightInsideX, gapBottomY),
            sideBypassStrengthScale,
            sideBypassRange,
            true),
        new CorridorCenterlineRail(
            leftRectX, railXWindow, gapTopY, corridorHalfWidthGuess, railStrength, railMaxForce),
        new CorridorCenterlineRail(
            leftRectX, railXWindow, gapBottomY, corridorHalfWidthGuess, railStrength, railMaxForce),
        new CorridorCenterlineRail(
            rightRectX, railXWindow, gapTopY, corridorHalfWidthGuess, railStrength, railMaxForce),
        new CorridorCenterlineRail(
            rightRectX,
            railXWindow,
            gapBottomY,
            corridorHalfWidthGuess,
            railStrength,
            railMaxForce),
        new CorridorCenterlineRail(
            centerRailX,
            centerRailWindow,
            gapTopY,
            corridorHalfWidthGuess,
            railStrength,
            railMaxForce),
        new CorridorCenterlineRail(
            centerRailX,
            centerRailWindow,
            gapBottomY,
            corridorHalfWidthGuess,
            railStrength,
            railMaxForce),
        new CorridorCenterlineRail(
            leftRectX - outerRailXOffset,
            outerRailWindow,
            gapTopY,
            corridorHalfWidthGuess,
            railStrength,
            railMaxForce),
        new CorridorCenterlineRail(
            leftRectX - outerRailXOffset,
            outerRailWindow,
            gapBottomY,
            corridorHalfWidthGuess,
            railStrength,
            railMaxForce),
        new CorridorCenterlineRail(
            rightRectX + outerRailXOffset,
            outerRailWindow,
            gapTopY,
            corridorHalfWidthGuess,
            railStrength,
            railMaxForce),
        new CorridorCenterlineRail(
            rightRectX + outerRailXOffset,
            outerRailWindow,
            gapBottomY,
            corridorHalfWidthGuess,
            railStrength,
            railMaxForce));
  }

  // @Override // TRENCH + BUMP
  // public List<Obstacle> fieldObstacles() {
  //   double maxRangeY = 0.9;
  //   double maxRangeX = 0.5;
  //   double strength = 1.4;
  //   return List.of(
  //       new SquareObstacle(new Translation2d(4.625594, Constants.FIELD_WIDTH / 2), 1.1938, 1,
  // 1.4),
  //       new SquareObstacle(
  //           new Translation2d(FIELD_LENGTH - 4.625594, Constants.FIELD_WIDTH / 2), 1.1938, 1,
  // 1.4),
  //       new RectangleObstacle(
  //           new Translation2d((FIELD_LENGTH / 2) - 3.63982, 1.4224),
  //           1.1938,
  //           0.255336,
  //           strength,
  //           maxRangeX,
  //           maxRangeY),
  //       new RectangleObstacle(
  //           new Translation2d((FIELD_LENGTH / 2) - 3.63982, Constants.FIELD_WIDTH - 1.4224),
  //           1.1938,
  //           0.255336,
  //           strength,
  //           maxRangeX,
  //           maxRangeY),
  //       new RectangleObstacle(
  //           new Translation2d((FIELD_LENGTH / 2) + 3.63982, 1.4224),
  //           1.1938,
  //           0.255336,
  //           strength,
  //           maxRangeX,
  //           maxRangeY),
  //       new RectangleObstacle(
  //           new Translation2d((FIELD_LENGTH / 2) + 3.63982, Constants.FIELD_WIDTH - 1.4224),
  //           1.1938,
  //           0.255336,
  //           strength,
  //           maxRangeX,
  //           maxRangeY));
  // }
  /**
   * Returns the walls value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public List<Obstacle> walls() {
    return List.of(
        new HorizontalObstacle(0.0, 1, true),
        new HorizontalObstacle(geometry.widthMeters(), 1, false),
        new VerticalObstacle(0.0, 3, true),
        new VerticalObstacle(geometry.lengthMeters(), 3, false));
  }

  /**
   * Returns the get heatmap value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public Heatmap getHeatmap() {
    /**
     * Configuration value for slow heat. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    final double SLOW_HEAT = 0.6;
    /**
     * Configuration value for fast heat. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    final double FAST_HEAT = 1.0;

    /**
     * Configuration value for trans m. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final double TRANS_M = 0.30;

    double cx = geometry.lengthMeters() * 0.5;
    double cy = geometry.widthMeters() * 0.5;

    double x0 = 0.0;
    double x1 = geometry.lengthMeters();
    double y0 = 0.0;
    double y1 = geometry.widthMeters();

    double half = 1.1938 * 0.5;

    double leftSqCx = 4.625594;
    double leftRectCx = (geometry.lengthMeters() * 0.5) - 3.63982;

    double rightSqCx = geometry.lengthMeters() - 4.625594;
    double rightRectCx = (geometry.lengthMeters() * 0.5) + 3.63982;

    double leftBandX0 = Math.min(leftSqCx, leftRectCx) - half;
    double leftBandX1 = Math.max(leftSqCx, leftRectCx) + half;

    double rightBandX0 = Math.min(rightSqCx, rightRectCx) - half;
    double rightBandX1 = Math.max(rightSqCx, rightRectCx) + half;

    leftBandX0 = MathUtil.clamp(leftBandX0, x0, x1);
    leftBandX1 = MathUtil.clamp(leftBandX1, x0, x1);
    rightBandX0 = MathUtil.clamp(rightBandX0, x0, x1);
    rightBandX1 = MathUtil.clamp(rightBandX1, x0, x1);

    if (leftBandX1 > rightBandX0) {
      double mid = 0.5 * (leftBandX1 + rightBandX0);
      leftBandX1 = mid;
      rightBandX0 = mid;
    }

    double fastL = Math.max(0.0, leftBandX0 - x0);
    double fastM = Math.max(0.0, rightBandX0 - leftBandX1);
    double fastR = Math.max(0.0, x1 - rightBandX1);

    double eL = MathUtil.clamp(TRANS_M, 0.0, fastL);
    double eML = MathUtil.clamp(TRANS_M, 0.0, fastM * 0.5);
    double eMR = MathUtil.clamp(TRANS_M, 0.0, fastM * 0.5);
    double eR = MathUtil.clamp(TRANS_M, 0.0, fastR);

    double fastLCoreW = Math.max(0.0, fastL - eL);
    double fastMCoreW = Math.max(0.0, fastM - eML - eMR);
    double fastRCoreW = Math.max(0.0, fastR - eR);

    var hb = Heatmap.builder();

    double curX = x0;

    if (fastLCoreW > 0.0) {
      hb.block("FAST_L_CORE", new Translation2d(curX, y0), fastLCoreW, y1 - y0, FAST_HEAT);
      curX += fastLCoreW;
    }
    if (eL > 0.0) {
      hb.block("FAST_L_EDGE", new Translation2d(curX, y0), eL, y1 - y0, FAST_HEAT);
      curX += eL;
    }

    hb.block(
        "SLOW_L", new Translation2d(leftBandX0, y0), leftBandX1 - leftBandX0, y1 - y0, SLOW_HEAT);
    curX = leftBandX1;

    if (eML > 0.0) {
      hb.block("FAST_M_L_EDGE", new Translation2d(curX, y0), eML, y1 - y0, FAST_HEAT);
      curX += eML;
    }
    if (fastMCoreW > 0.0) {
      hb.block("FAST_M_CORE", new Translation2d(curX, y0), fastMCoreW, y1 - y0, FAST_HEAT);
      curX += fastMCoreW;
    }
    if (eMR > 0.0) {
      hb.block("FAST_M_R_EDGE", new Translation2d(curX, y0), eMR, y1 - y0, FAST_HEAT);
      curX += eMR;
    }

    hb.block(
        "SLOW_R",
        new Translation2d(rightBandX0, y0),
        rightBandX1 - rightBandX0,
        y1 - y0,
        SLOW_HEAT);
    curX = rightBandX1;

    if (eR > 0.0) {
      hb.block("FAST_R_EDGE", new Translation2d(curX, y0), eR, y1 - y0, FAST_HEAT);
      curX += eR;
    }
    if (fastRCoreW > 0.0) {
      hb.block("FAST_R_CORE", new Translation2d(curX, y0), fastRCoreW, y1 - y0, FAST_HEAT);
      curX += fastRCoreW;
    }

    if (eL > 0.0) hb.transition("SLOW_L", "FAST_L_EDGE", 1.0);
    if (eML > 0.0) hb.transition("SLOW_L", "FAST_M_L_EDGE", 1.0);

    if (eMR > 0.0) hb.transition("SLOW_R", "FAST_M_R_EDGE", 1.0);
    if (eR > 0.0) hb.transition("SLOW_R", "FAST_R_EDGE", 1.0);

    return hb.build();
  }
}
