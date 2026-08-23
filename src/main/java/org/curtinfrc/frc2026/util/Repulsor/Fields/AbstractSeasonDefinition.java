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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldActionProfile.ActionRole;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldActionProfile.ProjectileShotAction;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceCollectionProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceRecoveryProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateLocalAccess;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.GameSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.StaticPoseSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.CollectPlannerTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;

/**
 * Generic base for season-specific {@link FieldDefinition} implementations. A new FRC game only
 * supplies constructor data (name, year, field dimensions, AprilTag layout, one collectable
 * resource model, optional score/station poses) plus an objective map built with {@link
 * FieldMapBuilder}; everything else has a safe game-neutral default that remains overridable.
 *
 * <p>This class deliberately uses neutral vocabulary only: a collectable object is a "resource",
 * goals are "score targets", and intake areas are "stations".
 */
public abstract class AbstractSeasonDefinition implements FieldDefinition {

  /** Neutral default launch constraints used by the generic projectile action. */
  protected static final Constraints DEFAULT_PROJECTILE_CONSTRAINTS =
      new Constraints(4.0, 30.0, 5.0, 45.0, Constraints.ShotStyle.ARC);

  private static final double DEFAULT_TARGET_HEIGHT_M = 0.35;
  private static final double DEFAULT_FIELD_MARGIN_M = 0.28;

  private final String gameName;
  private final int gameYear;
  private final AprilTagFieldLayout aprilTagLayout;
  private final FieldGeometry geometry;

  private final String resourceType;
  private final ResourceSpec resourceSpec;
  private final double allianceRecoveryZoneXMaxFraction;

  private final String projectileActionId;
  private final ActionRole projectileActionRole;
  private final Function<DriverStation.Alliance, Translation2d> projectileTargetForAlliance;
  private final double projectileTargetHeightMeters;
  private final String projectileRouteLevelId;
  private final HeightSetpoint projectileRouteMechanismLevel;

  private final GameSetpoint defaultScorePoint;
  private final GameSetpoint defaultCollectPoint;

  /**
   * Creates a season definition with every generic piece defaulted: a {@code TRANSFER_TO_SCORE}
   * projectile action aimed at a placeholder alliance-side target, and no default score/station
   * setpoints.
   *
   * @param gameName human-readable game name returned by {@link #gameName()}
   * @param gameYear game year returned by {@link #gameYear()}
   * @param lengthMeters field length in meters used by {@link #geometry()}
   * @param widthMeters field width in meters used by {@link #geometry()}
   * @param aprilTagLayout WPILib AprilTag layout for the game, or null for the default field
   * @param resourceType canonical collectable resource type string wired into the tracker
   * @param resourceRadiusMeters resource physical radius in meters
   * @param resourceUnitValue strategic value of one resource unit
   * @param resourceSigmaMeters observation uncertainty sigma in meters
   * @param allianceRecoveryZoneXMaxFraction fraction of field length kept as the blue-side recovery
   *     zone for transferred resources
   */
  protected AbstractSeasonDefinition(
      String gameName,
      int gameYear,
      double lengthMeters,
      double widthMeters,
      AprilTagFieldLayout aprilTagLayout,
      String resourceType,
      double resourceRadiusMeters,
      double resourceUnitValue,
      double resourceSigmaMeters,
      double allianceRecoveryZoneXMaxFraction) {
    this(
        gameName,
        gameYear,
        lengthMeters,
        widthMeters,
        aprilTagLayout,
        resourceType,
        resourceRadiusMeters,
        resourceUnitValue,
        resourceSigmaMeters,
        allianceRecoveryZoneXMaxFraction,
        "",
        ActionRole.TRANSFER_TO_SCORE,
        null,
        DEFAULT_TARGET_HEIGHT_M,
        HeightSetpoint.NONE,
        null,
        null);
  }

  /**
   * Creates a fully parameterized season definition.
   *
   * @param gameName human-readable game name returned by {@link #gameName()}
   * @param gameYear game year returned by {@link #gameYear()}
   * @param lengthMeters field length in meters used by {@link #geometry()}
   * @param widthMeters field width in meters used by {@link #geometry()}
   * @param aprilTagLayout WPILib AprilTag layout for the game, or null for the default field
   * @param resourceType canonical collectable resource type string wired into the tracker
   * @param resourceRadiusMeters resource physical radius in meters
   * @param resourceUnitValue strategic value of one resource unit
   * @param resourceSigmaMeters observation uncertainty sigma in meters
   * @param allianceRecoveryZoneXMaxFraction fraction of field length kept as the blue-side recovery
   *     zone for transferred resources
   * @param projectileActionId stable action ID for the generic projectile action; blank derives
   *     from the role name
   * @param projectileActionRole semantic role of the generic projectile action
   * @param projectileTargetForAlliance field-relative target supplier per alliance, or null for a
   *     quarter-field alliance-side placeholder derived from the geometry
   * @param projectileTargetHeightMeters target opening or impact-plane height in meters
   * @param projectileRouteMechanismLevel mechanism level used by the action route; its lowercase
   *     name doubles as the route level ID
   * @param defaultScorePoseBlue blue-side pose backing {@link #defaultScoreSetpoint()}, or null for
   *     an empty optional
   * @param defaultCollectPoseBlue blue-side pose backing {@link #defaultCollectSetpoint()}, or null
   *     for an empty optional
   */
  protected AbstractSeasonDefinition(
      String gameName,
      int gameYear,
      double lengthMeters,
      double widthMeters,
      AprilTagFieldLayout aprilTagLayout,
      String resourceType,
      double resourceRadiusMeters,
      double resourceUnitValue,
      double resourceSigmaMeters,
      double allianceRecoveryZoneXMaxFraction,
      String projectileActionId,
      ActionRole projectileActionRole,
      Function<DriverStation.Alliance, Translation2d> projectileTargetForAlliance,
      double projectileTargetHeightMeters,
      HeightSetpoint projectileRouteMechanismLevel,
      Pose2d defaultScorePoseBlue,
      Pose2d defaultCollectPoseBlue) {
    if (gameName == null || gameName.isBlank()) {
      throw new IllegalArgumentException("gameName cannot be null/blank");
    }
    if (resourceType == null || resourceType.isBlank()) {
      throw new IllegalArgumentException("resourceType cannot be null/blank");
    }
    this.gameName = gameName.trim();
    this.gameYear = gameYear;
    this.geometry = new FieldGeometry(lengthMeters, widthMeters);
    this.aprilTagLayout = aprilTagLayout;
    this.resourceType = resourceType.trim().toLowerCase();
    this.resourceSpec =
        new ResourceSpec(resourceRadiusMeters, resourceUnitValue, resourceSigmaMeters);
    this.allianceRecoveryZoneXMaxFraction = allianceRecoveryZoneXMaxFraction;

    this.projectileActionRole =
        projectileActionRole == null ? ActionRole.TRANSFER_TO_SCORE : projectileActionRole;
    this.projectileActionId =
        projectileActionId == null || projectileActionId.isBlank()
            ? this.projectileActionRole.name().toLowerCase()
            : projectileActionId.trim();
    this.projectileTargetForAlliance = projectileTargetForAlliance;
    this.projectileTargetHeightMeters =
        Double.isFinite(projectileTargetHeightMeters) && projectileTargetHeightMeters > 0.0
            ? projectileTargetHeightMeters
            : DEFAULT_TARGET_HEIGHT_M;
    this.projectileRouteMechanismLevel =
        projectileRouteMechanismLevel == null ? HeightSetpoint.NONE : projectileRouteMechanismLevel;
    this.projectileRouteLevelId = this.projectileRouteMechanismLevel.name().toLowerCase();

    this.defaultScorePoint =
        defaultScorePoseBlue == null
            ? null
            : new StaticPoseSetpoint("score.target", SetpointType.kScore, defaultScorePoseBlue);
    this.defaultCollectPoint =
        defaultCollectPoseBlue == null
            ? null
            : new StaticPoseSetpoint(
                "collect.station", SetpointType.kHumanPlayer, defaultCollectPoseBlue);
  }

  /** Returns the collectable resource type wired into the tracker by {@link #configureTracker}. */
  public String resourceType() {
    return resourceType;
  }

  /** Returns the evidence model for the collectable resource type. */
  public ResourceSpec resourceSpec() {
    return resourceSpec;
  }

  /**
   * Returns the fraction of field length treated as the blue-side recovery zone for transferred
   * resources.
   */
  public double allianceRecoveryZoneXMaxFraction() {
    return allianceRecoveryZoneXMaxFraction;
  }

  /**
   * Builds the collection profile installed through {@link
   * PredictiveFieldStateLocalAccess#setDefaultCollectionProfile}. Overridable to exclude regions or
   * change observation aging.
   */
  public ResourceCollectionProfile collectionProfile() {
    return new ResourceCollectionProfile(
        resourceType, resourceSpec, 0.95, 0.75, geometry(), List.of());
  }

  /**
   * Builds the recovery profile installed through {@link
   * PredictiveFieldStateLocalAccess#setDefaultRecoveryProfile}.
   */
  public ResourceRecoveryProfile recoveryProfile() {
    return new ResourceRecoveryProfile(
        resourceType(),
        resourceSpec(),
        geometry(),
        allianceRecoveryZoneXMaxFraction(),
        0.35,
        0.45,
        96);
  }

  @Override
  public String gameName() {
    return gameName;
  }

  @Override
  public int gameYear() {
    return gameYear;
  }

  @Override
  public AprilTagFieldLayout aprilTagLayout() {
    return aprilTagLayout != null
        ? aprilTagLayout
        : AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  @Override
  public FieldGeometry geometry() {
    return geometry;
  }

  @Override
  public double fieldLengthMeters() {
    return geometry.lengthMeters();
  }

  @Override
  public double fieldWidthMeters() {
    return geometry.widthMeters();
  }

  /**
   * Mutable obstacle list builders may populate before any planner consumes {@link
   * #fieldObstacles()}. The default returns a fresh empty list per call; store your own list and
   * return it to make additions persistent.
   */
  protected List<Obstacle> fieldObstaclesHook() {
    return new ArrayList<>();
  }

  /**
   * Mutable wall list builders may populate before any planner consumes {@link #walls()}. The
   * default returns a fresh empty list per call; store your own list and return it to make
   * additions persistent.
   */
  protected List<Obstacle> wallsHook() {
    return new ArrayList<>();
  }

  @Override
  public List<Obstacle> fieldObstacles() {
    List<Obstacle> obstacles = fieldObstaclesHook();
    return obstacles == null ? List.of() : List.copyOf(obstacles);
  }

  @Override
  public List<Obstacle> walls() {
    List<Obstacle> walls = wallsHook();
    return walls == null ? List.of() : List.copyOf(walls);
  }

  /**
   * Returns null by default. Consumers such as {@code DriveTuningHeat} treat a missing heatmap as
   * empty, which yields unrestricted speed everywhere; override to slow robots in high-heat zones.
   */
  @Override
  public Heatmap getHeatmap() {
    return null;
  }

  /**
   * Applies only game-neutral tracker wiring: the collectable resource type and evidence model,
   * collect planner tuning defaults, and the predictive collection/recovery profiles built from
   * constructor parameters. Scoring, semantic-region, and setpoint wiring stays with subclasses;
   * overrides should call {@code super.configureTracker(ft)} first.
   */
  @Override
  public void configureTracker(FieldTrackerCore ft) {
    if (ft == null) return;
    ft.setCollectResourceTypes(Set.of(resourceType()));
    ft.configureCollectResourceProfile(resourceType(), resourceSpec());
    ft.configureCollectPlanner(CollectPlannerTuning.defaults());
    PredictiveFieldStateLocalAccess.setDefaultCollectionProfile(collectionProfile());
    PredictiveFieldStateLocalAccess.setDefaultRecoveryProfile(recoveryProfile());
  }

  /** Returns the stable ID of the generic projectile action. */
  public String projectileActionId() {
    return projectileActionId;
  }

  /** Returns the semantic role of the generic projectile action. */
  public ActionRole projectileActionRole() {
    return projectileActionRole;
  }

  /**
   * Returns the per-alliance target supplier. When none was supplied at construction, a placeholder
   * aiming at the quarter-field alliance-side point is derived from this season's own geometry
   * (mirrored about its mid-length for red).
   */
  public Function<DriverStation.Alliance, Translation2d> projectileTargetForAlliance() {
    if (projectileTargetForAlliance != null) return projectileTargetForAlliance;
    return alliance -> {
      Translation2d blueTarget =
          new Translation2d(geometry.lengthMeters() * 0.25, geometry.widthMeters() * 0.5);
      return alliance == DriverStation.Alliance.Red
          ? new Translation2d(geometry.lengthMeters() - blueTarget.getX(), blueTarget.getY())
          : blueTarget;
    };
  }

  /** Returns the projectile physics used by the generic action; overridable. */
  protected GamePiecePhysics projectilePhysics() {
    return new GamePiecePhysics() {
      @Override
      public double massKg() {
        return 0.25;
      }

      @Override
      public double crossSectionAreaM2() {
        return 0.012;
      }

      @Override
      public double dragCoefficient() {
        return 1.0;
      }
    };
  }

  /** Returns the launch constraints used by the generic action; overridable. */
  protected Constraints projectileConstraints() {
    return DEFAULT_PROJECTILE_CONSTRAINTS;
  }

  /** Builds the single generic projectile action; overridable to tune stand-off and offsets. */
  protected ProjectileShotAction projectileAction() {
    return new ProjectileShotAction(
        projectileActionId(),
        projectileActionRole(),
        projectileTargetForAlliance(),
        projectilePhysics(),
        projectileTargetHeightMeters,
        projectileConstraints(),
        projectileRouteLevelId,
        projectileRouteMechanismLevel,
        0.0,
        new double[] {0.0},
        DEFAULT_FIELD_MARGIN_M,
        false,
        null);
  }

  @Override
  public FieldActionProfile actionProfile() {
    Map<String, ProjectileShotAction> actions = new LinkedHashMap<>();
    actions.put(projectileActionId(), projectileAction());
    return new FieldActionProfile(actions);
  }

  @Override
  public Optional<RepulsorSetpoint> defaultScoreSetpoint() {
    if (defaultScorePoint == null) return Optional.empty();
    return Optional.of(
        new RepulsorSetpoint(defaultScorePoint, "score.target", HeightSetpoint.NONE));
  }

  @Override
  public Optional<RepulsorSetpoint> defaultCollectSetpoint() {
    if (defaultCollectPoint == null) return Optional.empty();
    return Optional.of(
        new RepulsorSetpoint(defaultCollectPoint, "collect.station", HeightSetpoint.NONE));
  }
}
