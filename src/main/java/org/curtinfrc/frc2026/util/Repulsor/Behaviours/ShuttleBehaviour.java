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

package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.Runtime.ProjectileCycleRuntime;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.Runtime.ProjectileCycleRuntime.Aim;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldActionProfile.ProjectileShotAction;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.MutablePoseSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Intent;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.StrategyDirective;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.littletonrobotics.junction.Logger;

/**
 * Provides shuttle behaviour functionality for the Repulsor command-behaviour layer that converts
 * strategy and state into WPILib commands. Use this type from robot code, field profiles, or tests
 * when integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
 * method documents robot-relative motion.
 */
public class ShuttleBehaviour extends Behaviour {
  private static final double SHOOT_POS_TOL_METERS = 0.34;
  private static final double SHOOT_YAW_TOL_DEG = 13.0;

  private static final double OMEGA_COMFORT_RADPS = 3.0;
  private static final double PRETURN_LEAD_SEC = 0.20;

  private static final long MAGAZINE_CAPACITY = 16L;
  private static final long MAG_SAFETY_MARGIN = 1L;

  private final int prio;
  private final Supplier<Boolean> hasPiece;
  private final Supplier<Double> ourSpeedCap;
  private final Supplier<Boolean> mechanismReady;

  private final NetworkTablesValue<Double> shotAngle =
      NetworkTablesValue.ofDouble(
          NetworkTableInstance.getDefault(), NetworkTablesValue.toAdvantageKit("/ShotAngle"), 0.0);

  private final NetworkTablesValue<Double> shotSpeed =
      NetworkTablesValue.ofDouble(
          NetworkTableInstance.getDefault(), NetworkTablesValue.toAdvantageKit("/ShotSpeed"), 0.0);

  private final NetworkTablesValue<Boolean> shooterPassthrough =
      NetworkTablesValue.ofBoolean(
          NetworkTableInstance.getDefault(),
          NetworkTablesValue.toAdvantageKit("/ShooterPassthrough"),
          false);

  private final NetworkTablesValue<Long> pieceCount =
      NetworkTablesValue.ofInteger(NetworkTableInstance.getDefault(), "/PieceCount", 0L);

  private Pose2d lastCollectBluePose;

  /**
   * Returns the shuttle behaviour value maintained by this Repulsor component.
   *
   * @param priority distance or field-coordinate value in meters.
   * @param hasPiece value used by this operation.
   * @param ourSpeedCap value used by this operation.
   */
  public ShuttleBehaviour(int priority, Supplier<Boolean> hasPiece, Supplier<Double> ourSpeedCap) {
    this(priority, hasPiece, ourSpeedCap, () -> true);
  }

  /**
   * Returns the shuttle behaviour value maintained by this Repulsor component.
   *
   * @param priority distance or field-coordinate value in meters.
   * @param hasPiece value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param mechanismReady distance or field-coordinate value in meters.
   */
  public ShuttleBehaviour(
      int priority,
      Supplier<Boolean> hasPiece,
      Supplier<Double> ourSpeedCap,
      Supplier<Boolean> mechanismReady) {
    this.prio = priority;
    this.hasPiece = hasPiece;
    this.ourSpeedCap = ourSpeedCap;
    this.mechanismReady = mechanismReady == null ? () -> true : mechanismReady;
  }

  /**
   * Returns the name value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public String name() {
    return "Shuttle";
  }

  /**
   * Returns the priority value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public int priority() {
    return prio;
  }

  /**
   * Returns the should run value maintained by this Repulsor component.
   *
   * @param flags value used by this operation.
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  @Override
  public boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
    return flags.contains(BehaviourFlag.SHUTTLE_MODE)
        && ctx.repulsor.getFieldDefinition().actionProfile().transferProjectileShot().isPresent();
  }

  /**
   * Builds the WPILib command sequence for the current behaviour context.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  @Override
  public Command build(BehaviourContext ctx) {
    Optional<ProjectileShotAction> shotProfileOpt = selectedTransferAction(ctx);
    FieldGeometry geometry = ctx.repulsor.getFieldDefinition().geometry();
    List<Obstacle> staticShotObstacles = new ArrayList<>();
    staticShotObstacles.addAll(ctx.repulsor.getFieldDefinition().walls());
    staticShotObstacles.addAll(ctx.repulsor.getFieldDefinition().fieldObstacles());

    AtomicReference<Pose2d> collectBluePoseRef = new AtomicReference<>(Pose2d.kZero);
    RepulsorSetpoint collectRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint(
                "SHUTTLE_COLLECT_ROUTE", SetpointType.kOther, collectBluePoseRef),
            "none",
            HeightSetpoint.NONE);

    AtomicReference<Pose2d> shuttleBluePoseRef = new AtomicReference<>(Pose2d.kZero);
    RepulsorSetpoint shuttleShootRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint("SHUTTLE_SHOOT_ROUTE", SetpointType.kScore, shuttleBluePoseRef),
            shotProfileOpt.map(ProjectileShotAction::routeLevel).orElse("none"),
            shotProfileOpt
                .map(ProjectileShotAction::routeMechanismSetpoint)
                .orElse(HeightSetpoint.NONE));

    AtomicReference<Pose2d> lastRobotPose = new AtomicReference<>(null);
    AtomicLong lastRobotPoseNs = new AtomicLong(0L);
    AtomicReference<Double> lastTimeToPlaneSec =
        new AtomicReference<>(ProjectileCycleRuntime.DEFAULT_TIME_TO_PLANE_SEC);
    AtomicReference<ShotSolution> lastValidShot = new AtomicReference<>(null);

    final int collectGoalUnits = 2;

    return Commands.run(
            () -> {
              Pose2d robotPose = ctx.robotPose.get();
              long nowNs = System.nanoTime();
              var spCtx = ProjectileCycleRuntime.makeCtx(ctx, robotPose);

              var fieldVelocity =
                  ProjectileCycleRuntime.estimateFieldVelocity(
                      lastRobotPose.get(), lastRobotPoseNs.get(), robotPose, nowNs);
              lastRobotPose.set(robotPose);
              lastRobotPoseNs.set(nowNs);

              long currentPieceCount = ProjectileCycleRuntime.safePieceCount(pieceCount);
              boolean piecePresent = Boolean.TRUE.equals(hasPiece.get()) || currentPieceCount > 0L;

              double cap = ourSpeedCap != null ? Math.max(0.25, ourSpeedCap.get()) : 3.5;

              Pose2d collectGoalBlue =
                  directiveCollectGoal(ctx, robotPose, Intent.TRANSFER_FOR_LATER_SCORE)
                      .orElseGet(
                          () ->
                              FieldTrackerCore.getInstance()
                                  .nextCollectionGoalBlue(robotPose, cap, collectGoalUnits));
              if (collectGoalBlue == null) {
                collectGoalBlue = new Pose2d(geometry.center(), robotPose.getRotation());
              }
              collectGoalBlue =
                  new Pose2d(collectGoalBlue.getTranslation(), collectGoalBlue.getRotation());

              Aim aim =
                  shotProfileOpt
                      .map(
                          profile ->
                              ProjectileCycleRuntime.computeAim(
                                  profile,
                                  geometry,
                                  robotPose,
                                  spCtx,
                                  staticShotObstacles,
                                  ctx.vision.getObstacles(),
                                  fieldVelocity,
                                  lastTimeToPlaneSec))
                      .orElse(new Aim(robotPose, Optional.empty()));

              ProjectileCycleRuntime.publishShotTelemetry(aim, lastValidShot, shotSpeed, shotAngle);

              Choice choice =
                  chooseOpportunistic(
                      robotPose, cap, currentPieceCount, piecePresent, collectGoalBlue, aim);
              if (choice == Choice.SHOOT && shotProfileOpt.isEmpty()) {
                choice = Choice.COLLECT;
              }

              CategorySpec category;
              RepulsorSetpoint activeGoal;

              if (choice == Choice.SHOOT) {
                category = CategorySpec.kScore;
                Pose2d shootFieldPose = aim.shootPose();
                shuttleBluePoseRef.set(shootFieldPose);
                activeGoal = shuttleShootRoute;
              } else if (choice == Choice.COLLECT) {
                category = CategorySpec.kCollect;
                lastCollectBluePose = collectGoalBlue;
                collectBluePoseRef.set(collectGoalBlue);
                activeGoal = collectRoute;
              } else {
                shooterPassthrough.set(false);
                ctx.drive.runVelocity(new ChassisSpeeds());
                return;
              }

              ctx.repulsor.setCurrentGoal(activeGoal);
              Pose2d goalPose = activeGoal.get(spCtx);
              ctx.planner.setRequestedGoal(goalPose);

              boolean readyToShoot =
                  currentPieceCount > 0L
                      && ProjectileCycleRuntime.canRelease(
                          aim, robotPose, goalPose, SHOOT_POS_TOL_METERS, SHOOT_YAW_TOL_DEG);
              boolean preturn = choice == Choice.SHOOT && shouldPreTurn(robotPose, goalPose, cap);
              boolean mechanismAtSetpoint = Boolean.TRUE.equals(mechanismReady.get());
              boolean allowPassthrough = readyToShoot && mechanismAtSetpoint;
              Logger.recordOutput("Repulsor/Shuttle/ReadyToRelease", readyToShoot);
              Logger.recordOutput("Repulsor/Shuttle/MechanismReady", mechanismAtSetpoint);
              Logger.recordOutput("Repulsor/Shuttle/PassthroughAllowed", allowPassthrough);
              shooterPassthrough.set(allowPassthrough);

              if (choice == Choice.COLLECT && currentPieceCount >= MAGAZINE_CAPACITY) {
                shooterPassthrough.set(false);
              }

              RepulsorSample sample =
                  ctx.planner.calculate(
                      robotPose,
                      ctx.vision.getObstacles(),
                      ctx.robot_x,
                      ctx.robot_y,
                      category,
                      false,
                      0.0);

              ChassisSpeeds speeds =
                  sample.asChassisSpeeds(
                      ctx.repulsor.getDrive().getOmegaPID(), robotPose.getRotation());

              if (preturn && !readyToShoot) {
                double yawErr =
                    ProjectileCycleRuntime.shortestAngleRad(
                        robotPose.getRotation().getRadians(), goalPose.getRotation().getRadians());
                double omega =
                    MathUtil.clamp(
                        yawErr * 3.2, -OMEGA_COMFORT_RADPS * 1.25, OMEGA_COMFORT_RADPS * 1.25);
                speeds =
                    new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);
              }

              ctx.drive.runVelocity(speeds);
            },
            ctx.drive)
        .finallyDo(
            interrupted -> {
              shooterPassthrough.set(false);
              ctx.drive.runVelocity(new ChassisSpeeds());
            });
  }

  private static Optional<Pose2d> directiveCollectGoal(
      BehaviourContext ctx, Pose2d robotPose, Intent expectedIntent) {
    StrategyDirective directive = ctx.repulsor.getStrategyDirective();
    if (directive.intent() != expectedIntent || !directive.hasResourceTarget()) {
      return Optional.empty();
    }
    return Optional.of(directive.targetPose(robotPose.getRotation()));
  }

  private static Optional<ProjectileShotAction> selectedTransferAction(BehaviourContext ctx) {
    StrategyDirective directive = ctx.repulsor.getStrategyDirective();
    if (directive.intent() == Intent.TRANSFER_FOR_LATER_SCORE
        && !"none".equals(directive.actionId())) {
      Optional<ProjectileShotAction> action =
          ctx.repulsor.getFieldDefinition().actionProfile().projectileShot(directive.actionId());
      if (action.isPresent()) {
        return action;
      }
    }
    return ctx.repulsor.getFieldDefinition().actionProfile().transferProjectileShot();
  }

  private static boolean shouldPreTurn(Pose2d robotPose, Pose2d goalPose, double capMps) {
    double dist = robotPose.getTranslation().getDistance(goalPose.getTranslation());
    double v = Math.max(0.5, capMps);
    double eta = dist / v;
    double yawErr =
        Math.abs(
            ProjectileCycleRuntime.shortestAngleRad(
                robotPose.getRotation().getRadians(), goalPose.getRotation().getRadians()));
    double turnTime = yawErr / Math.max(1e-6, OMEGA_COMFORT_RADPS);
    return turnTime >= Math.max(0.0, eta - PRETURN_LEAD_SEC);
  }

  private static Choice chooseOpportunistic(
      Pose2d robotPose,
      double capMps,
      long pieceCount,
      boolean piecePresent,
      Pose2d collectGoalBlue,
      Aim aim) {
    if (!piecePresent && pieceCount <= 0L) {
      return Choice.COLLECT;
    }

    if (pieceCount >= MAGAZINE_CAPACITY - MAG_SAFETY_MARGIN) {
      return aim.activeShotSolution().isPresent() ? Choice.SHOOT : Choice.COLLECT;
    }

    Pose2d shootPose = aim.activeShootPose();
    boolean shotOk = aim.activeShotSolution().isPresent();

    double v = Math.max(0.6, capMps);

    double dCollect = robotPose.getTranslation().getDistance(collectGoalBlue.getTranslation());
    double dShoot = robotPose.getTranslation().getDistance(shootPose.getTranslation());

    double etaCollect = dCollect / v;
    double etaShoot = dShoot / v;

    double yawErrShoot =
        Math.abs(
            ProjectileCycleRuntime.shortestAngleRad(
                robotPose.getRotation().getRadians(), shootPose.getRotation().getRadians()));
    double alignTime = yawErrShoot / Math.max(1e-6, OMEGA_COMFORT_RADPS);

    double fullness = MathUtil.clamp(pieceCount / (double) MAGAZINE_CAPACITY, 0.0, 1.0);

    double shootPenalty = shotOk ? 0.0 : 2.5 + 2.0 * fullness;
    double collectPenalty = 0.0;

    double shootBenefit = 0.55 * fullness + 0.05 * Math.min(16.0, pieceCount);
    double collectBenefit = 0.12 * (1.0 - fullness);

    double shootCost = etaShoot + 0.65 * alignTime + shootPenalty - shootBenefit;
    double collectCost = etaCollect + collectPenalty - collectBenefit;

    if (pieceCount > 0L
        && ProjectileCycleRuntime.isReadyToShoot(
            robotPose, shootPose, SHOOT_POS_TOL_METERS, SHOOT_YAW_TOL_DEG)
        && shotOk) {
      return Choice.SHOOT;
    }

    if (pieceCount <= 0L) {
      return Choice.COLLECT;
    }

    return shootCost <= collectCost ? Choice.SHOOT : Choice.COLLECT;
  }

  private enum Choice {
    COLLECT,
    SHOOT,
    STOP
  }
}
