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

package org.curtinfrc.frc2026.util.Repulsor;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.AutoPathRuntimeConfig;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.Behaviour;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourContext;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourFlag;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourManager;
import org.curtinfrc.frc2026.util.Repulsor.Commands.Triggers;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.NtRepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.RepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.Fallback.PlannerFallback;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldDefinition;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Flags.FlagManager;
import org.curtinfrc.frc2026.util.Repulsor.Reasoning.Reasoner;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.GameSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.State.SimMatchDriver;
import org.curtinfrc.frc2026.util.Repulsor.State.StateManager;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.RepulsorStrategyPreset;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.StrategyDirective;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Vision.FieldVision;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DriveTuningHeat;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision;
import org.littletonrobotics.junction.Logger;

/**
 * Provides repulsor functionality for the Repulsor core Repulsor coordination layer. Use this type
 * from robot code, field profiles, or tests when integrating the corresponding Repulsor subsystem.
 * Coordinates are field-relative unless a method documents robot-relative motion.
 */
public class Repulsor {

  /**
   * Defines the usage type values used by the Repulsor core Repulsor coordination layer. Use this
   * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public enum UsageType {
    kFullAuto,
    kAutoDrive
  }

  private double robot_x;
  private double robot_y;

  private Supplier<Double> shooterReleaseHeightMeters = () -> 0.0;

  private FieldPlanner m_planner;
  private VisionPlanner m_visionPlanner = new VisionPlanner();
  private DriveRepulsor m_drive;
  private UsageType m_usageType = UsageType.kAutoDrive;
  private FieldDefinition m_fieldDefinition = Constants.FIELD;

  /**
   * Returns the is same drive value maintained by this Repulsor component.
   *
   * @param other value used by this operation.
   * @return value produced by this operation.
   */
  public boolean isSameDrive(DriveRepulsor other) {
    return this.m_drive == other;
  }

  private RepulsorSetpoint m_currentGoal;

  private RepulsorSetpoint m_nextScore;
  private final AtomicReference<RepulsorStrategyPreset> m_strategyPresetSnapshot =
      new AtomicReference<>();
  private volatile String m_currentStrategyPreset = "";
  private volatile StrategyDirective m_strategyDirective = StrategyDirective.none();

  private final List<FieldVision> m_fieldVisions = new ArrayList<>();

  private Optional<Trigger> m_gateInScoring = Optional.empty();
  private Optional<Trigger> m_gateInCollecting = Optional.empty();

  private Supplier<Boolean> m_hasPiece = () -> false;

  private BehaviourManager m_behaviourManager;
  private FlagManager<BehaviourFlag> m_behaviourFlags;

  /**
   * Returns the at setpoint value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean atSetpoint() {
    Optional<Distance> err = m_planner.getErr();
    if (err.isEmpty()) return false;
    // Logger.recordOutput("Repulsor/err", err.get());
    return err.isPresent() && err.get().lt(Meters.of(0.1));
  }

  /**
   * Returns the with has piece supplier value maintained by this Repulsor component.
   *
   * @param hasPiece value used by this operation.
   * @return value produced by this operation.
   */
  public Repulsor withHasPieceSupplier(Supplier<Boolean> hasPiece) {
    this.m_hasPiece = hasPiece;
    return this;
  }

  /**
   * Returns the has piece value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean hasPiece() {
    return Boolean.TRUE.equals(m_hasPiece.get());
  }

  /**
   * Returns the with shooter release height meters supplier value maintained by this Repulsor
   * component.
   *
   * @param supplier value used by this operation.
   * @return value produced by this operation.
   */
  public Repulsor withShooterReleaseHeightMetersSupplier(Supplier<Double> supplier) {
    this.shooterReleaseHeightMeters = supplier == null ? () -> 0.0 : supplier;
    return this;
  }

  /**
   * Returns the add behaviour value maintained by this Repulsor component.
   *
   * @param behaviour value used by this operation.
   * @return value produced by this operation.
   */
  public Repulsor addBehaviour(Behaviour behaviour) {
    m_behaviourManager.add(Objects.requireNonNull(behaviour, "behaviour"));
    return this;
  }

  /**
   * Returns the add behaviours value maintained by this Repulsor component.
   *
   * @param behaviours value used by this operation.
   * @return value produced by this operation.
   */
  public Repulsor addBehaviours(Behaviour... behaviours) {
    if (behaviours == null) return this;
    for (Behaviour behaviour : behaviours) {
      addBehaviour(behaviour);
    }
    return this;
  }

  /**
   * Updates clear behaviours state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @return value produced by this operation.
   */
  public Repulsor clearBehaviours() {
    m_behaviourManager.clear();
    return this;
  }

  /**
   * Updates set reasoner state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param reasoner value used by this operation.
   * @return value produced by this operation.
   */
  public Repulsor setReasoner(Reasoner<BehaviourFlag, BehaviourContext> reasoner) {
    m_behaviourManager.setReasoner(reasoner);
    return this;
  }

  /**
   * Updates set strategy directive state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param directive value used by this operation.
   */
  public void setStrategyDirective(StrategyDirective directive) {
    m_strategyDirective = directive == null ? StrategyDirective.none() : directive;
  }

  /**
   * Returns the get strategy directive value maintained by this Repulsor component.
   *
   * @return strategy directive result for get strategy directive.
   */
  public StrategyDirective getStrategyDirective() {
    return m_strategyDirective;
  }

  /**
   * Returns the is in scoring gate value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isInScoringGate() {
    return m_gateInScoring.map(Trigger::getAsBoolean).orElse(true);
  }

  /**
   * Returns the is in collecting gate value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isInCollectingGate() {
    return m_gateInCollecting.map(Trigger::getAsBoolean).orElse(false);
  }

  /**
   * Returns the get next score value maintained by this Repulsor component.
   *
   * @return repulsor setpoint result for get next score.
   */
  public RepulsorSetpoint getNextScore() {
    return m_nextScore;
  }

  /**
   * Returns the follow gate value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param collecting value used by this operation.
   * @param scoring value used by this operation.
   * @return value produced by this operation.
   */
  public <T> Repulsor followGate(Triggers.PhaseGate<T> gate, T collecting, T scoring) {
    Trigger inScoring = gate.when(scoring);
    Trigger inCollecting = gate.when(collecting);

    m_gateInScoring = Optional.of(inScoring);
    m_gateInCollecting = Optional.of(inCollecting);
    return this;
  }

  /**
   * Returns the follow gate value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param collectingTags value used by this operation.
   * @param scoringTags value used by this operation.
   * @return value produced by this operation.
   */
  public <E extends Enum<E>> Repulsor followGate(
      Triggers.ParallelGate<E> gate, EnumSet<E> collectingTags, EnumSet<E> scoringTags) {

    if (collectingTags.isEmpty() || scoringTags.isEmpty()) {
      throw new IllegalArgumentException("collectingTags and scoringTags must be non-empty");
    }

    Supplier<Boolean> scoringAllOn = () -> scoringTags.stream().allMatch(gate::isOn);
    Supplier<Boolean> collectingAllOn = () -> collectingTags.stream().allMatch(gate::isOn);

    Trigger inScoring = new Trigger(scoringAllOn::get);
    Trigger inCollecting = new Trigger(collectingAllOn::get);

    m_gateInScoring = Optional.of(inScoring);
    m_gateInCollecting = Optional.of(inCollecting);
    return this;
  }

  /**
   * Returns the repulsor value maintained by this Repulsor component.
   *
   * @param drive value used by this operation.
   * @param usageType value used by this operation.
   * @param robot_x distance or field-coordinate value in meters.
   * @param robot_y distance or field-coordinate value in meters.
   * @param hasPiece value used by this operation.
   */
  public Repulsor(
      DriveRepulsor drive,
      UsageType usageType,
      double robot_x,
      double robot_y,
      Supplier<Boolean> hasPiece) {
    this(drive, usageType, robot_x, robot_y, hasPiece, Constants.FIELD);
  }

  /**
   * Returns the repulsor value maintained by this Repulsor component.
   *
   * @param drive value used by this operation.
   * @param usageType value used by this operation.
   * @param robot_x distance or field-coordinate value in meters.
   * @param robot_y distance or field-coordinate value in meters.
   * @param hasPiece value used by this operation.
   * @param fieldDefinition value used by this operation.
   */
  public Repulsor(
      DriveRepulsor drive,
      UsageType usageType,
      double robot_x,
      double robot_y,
      Supplier<Boolean> hasPiece,
      FieldDefinition fieldDefinition) {
    this.m_drive = drive;
    this.m_usageType = usageType;
    this.robot_x = robot_x;
    this.robot_y = robot_y;
    this.m_hasPiece = hasPiece;

    FieldDefinition field = fieldDefinition == null ? Constants.FIELD : fieldDefinition;
    m_fieldDefinition = field;
    m_currentGoal = field.defaultCollectSetpoint().orElse(null);
    m_nextScore = field.defaultScoreSetpoint().orElse(m_currentGoal);

    m_planner = new FieldPlanner(field, new DriveTuningHeat(() -> m_drive.getPose(), field));
    m_behaviourManager = new BehaviourManager();

    FieldTrackerCore.setDefaultProvider(field);
    FieldTrackerCore ft = FieldTrackerCore.getInstance();
    ft.rebuild(field);
    FieldVision front = ft.createFieldVision("main");
    m_fieldVisions.add(front);

    String defaultPreset = field.defaultStrategyPreset();
    if (defaultPreset != null && !defaultPreset.isBlank()) {
      applyStrategyPreset(defaultPreset);
    }

    SimMatchDriver.simInit(false);
  }

  /**
   * Returns the repulsor value maintained by this Repulsor component.
   *
   * @param drive value used by this operation.
   * @param robot_x distance or field-coordinate value in meters.
   * @param robot_y distance or field-coordinate value in meters.
   * @param hasPiece value used by this operation.
   */
  public Repulsor(DriveRepulsor drive, double robot_x, double robot_y, Supplier<Boolean> hasPiece) {
    this(drive, UsageType.kFullAuto, robot_x, robot_y, hasPiece);
  }

  /**
   * Returns the repulsor value maintained by this Repulsor component.
   *
   * @param drive value used by this operation.
   * @param robot_x distance or field-coordinate value in meters.
   * @param robot_y distance or field-coordinate value in meters.
   * @param hasPiece value used by this operation.
   * @param fieldDefinition value used by this operation.
   */
  public Repulsor(
      DriveRepulsor drive,
      double robot_x,
      double robot_y,
      Supplier<Boolean> hasPiece,
      FieldDefinition fieldDefinition) {
    this(drive, UsageType.kFullAuto, robot_x, robot_y, hasPiece, fieldDefinition);
  }

  /**
   * Returns the with initial next value maintained by this Repulsor component.
   *
   * @param setpoint value used by this operation.
   * @return value produced by this operation.
   */
  public Repulsor withInitialNext(RepulsorSetpoint setpoint) {
    if (setpoint != null && setpoint.point().type() == SetpointType.kHumanPlayer) {
      throw new IllegalArgumentException("Next score setpoint cannot be a human-player one");
    }
    m_nextScore = setpoint;
    return this;
  }

  /**
   * Returns the with initial hp value maintained by this Repulsor component.
   *
   * @param setpoint value used by this operation.
   * @return value produced by this operation.
   */
  public Repulsor withInitialHP(RepulsorSetpoint setpoint) {
    if (setpoint != null && setpoint.point().type() != SetpointType.kHumanPlayer) {
      throw new IllegalArgumentException("Next collect setpoint must be a human-player/collect one");
    }
    m_currentGoal = setpoint;
    return this;
  }

  /**
   * Updates set next score state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param next value used by this operation.
   */
  public void setNextScore(RepulsorSetpoint next) {
    if (next != null && next.point().type() == SetpointType.kHumanPlayer) {
      throw new IllegalArgumentException("Next score setpoint cannot be a human-player one");
    }
    m_nextScore = next;
  }

  /**
   * Returns the with fallback value maintained by this Repulsor component.
   *
   * @param fallback value used by this operation.
   * @return value produced by this operation.
   */
  public Repulsor withFallback(PlannerFallback fallback) {
    m_planner = m_planner.withFallback(fallback);
    return this;
  }

  /**
   * Returns the with vision value maintained by this Repulsor component.
   *
   * @param vision value used by this operation.
   * @return value produced by this operation.
   */
  public Repulsor withVision(RepulsorVision vision) {
    m_visionPlanner.addVision(vision);
    return this;
  }

  /**
   * Returns the with field vision value maintained by this Repulsor component.
   *
   * @param vision value used by this operation.
   * @return value produced by this operation.
   */
  public Repulsor withFieldVision(FieldVision vision) {
    m_fieldVisions.add(vision);
    return this;
  }

  /**
   * Runs add field vision in the Repulsor runtime.
   *
   * @param vision value used by this operation.
   */
  public void addFieldVision(FieldVision vision) {
    m_fieldVisions.add(vision);
  }

  /**
   * Returns the get field planner value maintained by this Repulsor component.
   *
   * @return field planner result for get field planner.
   */
  public FieldPlanner getFieldPlanner() {
    return m_planner;
  }

  /**
   * Returns the get field definition value maintained by this Repulsor component.
   *
   * @return field definition result for get field definition.
   */
  public FieldDefinition getFieldDefinition() {
    return m_fieldDefinition;
  }

  public List<String> availableStrategyPresets() {
    Map<String, RepulsorStrategyPreset> presets =
        m_fieldDefinition == null ? Map.of() : m_fieldDefinition.strategyPresets();
    ArrayList<String> names = new ArrayList<>(presets.keySet());
    Collections.sort(names);
    return List.copyOf(names);
  }

  public Optional<RepulsorStrategyPreset> strategyPreset(String name) {
    if (m_fieldDefinition == null || name == null || name.isBlank()) return Optional.empty();
    return m_fieldDefinition.strategyPreset(name);
  }

  public String currentStrategyPreset() {
    return m_currentStrategyPreset;
  }

  public Optional<RepulsorStrategyPreset> currentStrategyPresetSnapshot() {
    return Optional.ofNullable(m_strategyPresetSnapshot.get());
  }

  public AutoPathRuntimeConfig autoPathRuntimeConfig() {
    RepulsorStrategyPreset preset = m_strategyPresetSnapshot.get();
    if (preset != null) return preset.autoPath();
    return m_fieldDefinition == null
        ? AutoPathRuntimeConfig.defaults()
        : m_fieldDefinition.autoPathRuntimeConfig();
  }

  public boolean applyStrategyPreset(String name) {
    if (m_fieldDefinition == null) return false;
    String requested = name == null ? "" : name.trim();
    Optional<RepulsorStrategyPreset> requestedPreset = m_fieldDefinition.strategyPreset(requested);
    boolean fallbackUsed = false;
    RepulsorStrategyPreset preset = requestedPreset.orElse(null);
    if (preset == null) {
      String fallback = m_fieldDefinition.defaultStrategyPreset();
      if (fallback != null && !fallback.isBlank() && !fallback.equals(requested)) {
        preset = m_fieldDefinition.strategyPreset(fallback).orElse(null);
        fallbackUsed = preset != null;
      }
    }
    if (preset == null) {
      Logger.recordOutput("Repulsor/StrategyPreset/Requested", requested);
      Logger.recordOutput("Repulsor/StrategyPreset/Applied", false);
      Logger.recordOutput("Repulsor/StrategyPreset/FallbackUsed", false);
      return false;
    }

    applyStrategyPresetSnapshot(preset, requested, fallbackUsed);
    return requestedPreset.isPresent();
  }

  private void applyStrategyPresetSnapshot(
      RepulsorStrategyPreset preset, String requested, boolean fallbackUsed) {
    if (preset == null) return;

    m_strategyPresetSnapshot.set(preset);
    m_currentStrategyPreset = preset.name();

    if (m_planner != null) {
      m_planner.setRuntimeConfig(preset.plannerRuntime());
      m_planner.setWaypointPolicyProfile(preset.waypointPolicy());
    }

    FieldTrackerCore tracker = FieldTrackerCore.getInstance();
    tracker.configurePredictiveRanking(preset.predictiveRanking());
    tracker.configureObjectiveSelection(preset.objectiveSelection());
    tracker.configureCollectPlanner(preset.collectPlanner());

    Logger.recordOutput("Repulsor/StrategyPreset/Requested", requested == null ? "" : requested);
    Logger.recordOutput("Repulsor/StrategyPreset/Current", preset.name());
    Logger.recordOutput("Repulsor/StrategyPreset/Applied", true);
    Logger.recordOutput("Repulsor/StrategyPreset/FallbackUsed", fallbackUsed);
  }

  /**
   * Returns the get vision planner value maintained by this Repulsor component.
   *
   * @return vision planner result for get vision planner.
   */
  public VisionPlanner getVisionPlanner() {
    return m_visionPlanner;
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  public void update() {
    DeltaTime.update();

    StateManager.update(DeltaTime.get());

    SimMatchDriver.simPeriodic(DeltaTime.get());

    for (var vision : m_fieldVisions) {
      vision.update(m_drive.getPose());
    }

    refreshNextScoreFromFieldTracker();

    m_visionPlanner.tick();

    boolean enabled = false;
    if (RepulsorDriverStation.isInitialized()) {
      RepulsorDriverStation dsBase = RepulsorDriverStation.getInstance();
      if (dsBase instanceof NtRepulsorDriverStation ds) {
        try {
          enabled = ds.getConfigBool("force_controller_override");
        } catch (RuntimeException ignored) {
          enabled = false;
        }
      }
    }

    if (!m_usageType.equals(UsageType.kFullAuto)) {
      return;
    }

    if (enabled) {
      m_behaviourManager.stop();
      return;
    }

    m_behaviourManager.update(
        new BehaviourContext(
            this, m_planner, m_visionPlanner, m_drive, robot_x, robot_y, m_drive::getPose));
  }

  /** Runs disable behaviours in the Repulsor runtime. */
  public void disableBehaviours() {
    m_usageType = UsageType.kAutoDrive;
  }

  /**
   * Returns the get drive value maintained by this Repulsor component.
   *
   * @return drive repulsor result for get drive.
   */
  public DriveRepulsor getDrive() {
    return m_drive;
  }

  /**
   * Updates setup state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  public void setup() {
    if (m_usageType != UsageType.kFullAuto) return;
  }

  private SetpointContext ctxFor(Pose2d robotPose, List<? extends Obstacle> dyn) {
    double len = Math.max(0.0, robot_x);
    double wid = Math.max(0.0, robot_y);
    double release =
        shooterReleaseHeightMeters == null ? 0.0 : Math.max(0.0, shooterReleaseHeightMeters.get());
    return new SetpointContext(Optional.ofNullable(robotPose), len, wid, release, dyn);
  }

  private Command alignCore(
      Supplier<RepulsorSetpoint> supplier,
      Optional<Trigger> untilOpt,
      CategorySpec cat,
      boolean suppressFallback) {
    final AtomicReference<RepulsorSetpoint> activeRef = new AtomicReference<>();
    final AtomicBoolean initialized = new AtomicBoolean(false);

    Command cmd =
        Commands.run(
                () -> {
                  if (!initialized.get()) {
                    activeRef.set(supplier.get());
                    initialized.set(true);
                  }

                  m_planner.pollChosenSetpoint().ifPresent(activeRef::set);

                  RepulsorSetpoint effective = activeRef.get();
                  if (effective == null) return;

                  m_currentGoal = effective;

                  Pose2d robotPose = m_drive.getPose();
                  List<? extends Obstacle> dyn = m_visionPlanner.getObstacles();
                  Pose2d goalPose = effective.get(ctxFor(robotPose, dyn));
                  m_planner.setRequestedGoal(goalPose);

                  RepulsorSample sample =
                      m_planner.calculate(
                          robotPose,
                          dyn,
                          robot_x,
                          robot_y,
                          cat,
                          suppressFallback,
                          shooterReleaseHeightMeters == null
                              ? 0.0
                              : Math.max(0.0, shooterReleaseHeightMeters.get()));

                  m_planner
                      .pollChosenSetpoint()
                      .ifPresent(
                          sp -> {
                            activeRef.set(sp);
                            m_currentGoal = sp;
                            Pose2d g = sp.get(ctxFor(robotPose, dyn));
                            m_planner.setRequestedGoal(g);
                          });

                  m_drive.runVelocity(
                      sample.asChassisSpeeds(m_drive.getOmegaPID(), robotPose.getRotation()));
                },
                m_drive)
            .finallyDo(interrupted -> m_drive.runVelocity(new ChassisSpeeds()));

    if (untilOpt.isPresent()) {
      cmd = cmd.until(untilOpt.get());
    }
    return cmd;
  }

  /**
   * Returns the align to value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param until value used by this operation.
   * @param cat value used by this operation.
   * @return value produced by this operation.
   */
  public Command alignTo(Supplier<RepulsorSetpoint> point, Trigger until, CategorySpec cat) {
    return alignCore(point, Optional.of(until), cat, false);
  }

  /**
   * Returns the align to value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param until value used by this operation.
   * @param cat value used by this operation.
   * @return value produced by this operation.
   */
  public Command alignTo(RepulsorSetpoint point, Trigger until, CategorySpec cat) {
    return alignCore(() -> point, Optional.of(until), cat, false);
  }

  /**
   * Returns the align to value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param cat value used by this operation.
   * @return value produced by this operation.
   */
  public Command alignTo(RepulsorSetpoint point, CategorySpec cat) {
    return alignCore(() -> point, Optional.empty(), cat, false);
  }

  /**
   * Returns the align to value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param cat value used by this operation.
   * @return value produced by this operation.
   */
  public Command alignTo(Supplier<RepulsorSetpoint> point, CategorySpec cat) {
    return alignCore(point, Optional.empty(), cat, false);
  }

  /**
   * Returns the align to value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param until value used by this operation.
   * @param cat value used by this operation.
   * @param suppressFallback value used by this operation.
   * @return value produced by this operation.
   */
  public Command alignTo(
      Supplier<RepulsorSetpoint> point, Trigger until, CategorySpec cat, boolean suppressFallback) {
    return alignCore(point, Optional.of(until), cat, suppressFallback);
  }

  /**
   * Returns the align to value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param until value used by this operation.
   * @param cat value used by this operation.
   * @param suppressFallback value used by this operation.
   * @return value produced by this operation.
   */
  public Command alignTo(
      RepulsorSetpoint point, Trigger until, CategorySpec cat, boolean suppressFallback) {
    return alignCore(() -> point, Optional.of(until), cat, suppressFallback);
  }

  /**
   * Returns the align to value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param cat value used by this operation.
   * @param suppressFallback value used by this operation.
   * @return value produced by this operation.
   */
  public Command alignTo(RepulsorSetpoint point, CategorySpec cat, boolean suppressFallback) {
    return alignCore(() -> point, Optional.empty(), cat, suppressFallback);
  }

  /**
   * Returns the align to value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param cat value used by this operation.
   * @param suppressFallback value used by this operation.
   * @return value produced by this operation.
   */
  public Command alignTo(
      Supplier<RepulsorSetpoint> point, CategorySpec cat, boolean suppressFallback) {
    return alignCore(point, Optional.empty(), cat, suppressFallback);
  }

  /**
   * Updates set current goal state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param sp value used by this operation.
   */
  public void setCurrentGoal(RepulsorSetpoint sp) {
    m_currentGoal = sp;
  }

  public RepulsorSetpoint getCurrentGoal() {
    return m_currentGoal;
  }

  /**
   * Returns the get target height value maintained by this Repulsor component.
   *
   * @return height setpoint result for get target height.
   */
  public HeightSetpoint getTargetHeight() {
    return m_currentGoal == null ? HeightSetpoint.NONE : m_currentGoal.height();
  }

  private Trigger withinCore(
      Distance d, Optional<SetpointType> typeOpt, Optional<GameSetpoint> pointOpt) {
    return new Trigger(
        () -> {
          Optional<Distance> err = m_planner.getErr();
          if (err.isEmpty()) {
            return false;
          }
          boolean within = err.get().lt(d);
          if (typeOpt.isPresent()) {
            within =
                within && m_currentGoal != null && m_currentGoal.point().type() == typeOpt.get();
          }
          if (pointOpt.isPresent()) {
            within = within && m_currentGoal != null && m_currentGoal.point() == pointOpt.get();
          }
          return within;
        });
  }

  /**
   * Returns the within value maintained by this Repulsor component.
   *
   * @param d value used by this operation.
   * @return value produced by this operation.
   */
  public Trigger within(Distance d) {
    return withinCore(d, Optional.empty(), Optional.empty());
  }

  /**
   * Returns the within value maintained by this Repulsor component.
   *
   * @param d value used by this operation.
   * @param t value used by this operation.
   * @return value produced by this operation.
   */
  public Trigger within(Distance d, SetpointType t) {
    return withinCore(d, Optional.of(t), Optional.empty());
  }

  /**
   * Returns the within value maintained by this Repulsor component.
   *
   * @param d value used by this operation.
   * @param p value used by this operation.
   * @return value produced by this operation.
   */
  public Trigger within(Distance d, GameSetpoint p) {
    return withinCore(d, Optional.empty(), Optional.of(p));
  }

  private Optional<RepulsorSetpoint> chooseNextScoreFromFieldTracker() {
    FieldTrackerCore ft = FieldTrackerCore.getInstance();
    GameElement[] elements = ft.getFieldMap();
    if (elements == null || elements.length == 0) return Optional.empty();

    Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) return Optional.empty();
    Alliance preferred =
        allianceOpt.get() == DriverStation.Alliance.Blue ? Alliance.kBlue : Alliance.kRed;

    for (GameElement e : elements) {
      if (e.getAlliance() == preferred && !e.isAtCapacity()) {
        Optional<RepulsorSetpoint> sp = e.getRelatedPoint();
        if (sp.isPresent()) return sp;
      }
    }

    return Optional.empty();
  }

  private void refreshNextScoreFromFieldTracker() {
    Optional<RepulsorSetpoint> ftNext = chooseNextScoreFromFieldTracker();
    ftNext.ifPresent(this::setNextScore);
  }
}
