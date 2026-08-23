package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointUtil;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Intent;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.StrategyDirective;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.littletonrobotics.junction.Logger;

/**
 * Provides shuttle recovery behaviour functionality for the Repulsor command-behaviour layer that
 * converts strategy and state into WPILib commands. Use this type from robot code, field profiles,
 * or tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public final class ShuttleRecoveryBehaviour extends Behaviour {
  private static final double SHOOT_POS_TOL_METERS = 0.34;
  private static final double SHOOT_YAW_TOL_DEG = 13.0;

  private static final long MAGAZINE_CAPACITY = 16L;
  private static final int RECOVERY_GOAL_UNITS = 2;

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

  /**
   * Returns the shuttle recovery behaviour value maintained by this Repulsor component.
   *
   * @param priority distance or field-coordinate value in meters.
   * @param hasPiece value used by this operation.
   * @param ourSpeedCap value used by this operation.
   */
  public ShuttleRecoveryBehaviour(
      int priority, Supplier<Boolean> hasPiece, Supplier<Double> ourSpeedCap) {
    this(priority, hasPiece, ourSpeedCap, () -> true);
  }

  /**
   * Returns the shuttle recovery behaviour value maintained by this Repulsor component.
   *
   * @param priority distance or field-coordinate value in meters.
   * @param hasPiece value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param mechanismReady distance or field-coordinate value in meters.
   */
  public ShuttleRecoveryBehaviour(
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
    return "ShuttleRecovery";
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
    return flags.contains(BehaviourFlag.SHUTTLE_RECOVERY_MODE)
        && ctx.repulsor.getFieldDefinition().actionProfile().scoreProjectileShot().isPresent();
  }

  /**
   * Builds the WPILib command sequence for the current behaviour context.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  @Override
  public Command build(BehaviourContext ctx) {
    Optional<ProjectileShotAction> shotProfileOpt = selectedScoreAction(ctx);
    FieldGeometry geometry = ctx.repulsor.getFieldDefinition().geometry();
    List<Obstacle> staticShotObstacles = new ArrayList<>();
    staticShotObstacles.addAll(ctx.repulsor.getFieldDefinition().walls());
    staticShotObstacles.addAll(ctx.repulsor.getFieldDefinition().fieldObstacles());

    AtomicReference<Pose2d> collectBluePoseRef = new AtomicReference<>(Pose2d.kZero);
    RepulsorSetpoint collectRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint(
                "SHUTTLE_RECOVERY_COLLECT_ROUTE", SetpointType.kOther, collectBluePoseRef),
            "none",
            HeightSetpoint.NONE);

    AtomicReference<Pose2d> shuttleBluePoseRef = new AtomicReference<>(Pose2d.kZero);
    RepulsorSetpoint shuttleShootRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint(
                "SHUTTLE_RECOVERY_SHOOT_ROUTE", SetpointType.kScore, shuttleBluePoseRef),
            shotProfileOpt.map(ProjectileShotAction::routeLevel).orElse("none"),
            shotProfileOpt
                .map(ProjectileShotAction::routeMechanismSetpoint)
                .orElse(HeightSetpoint.NONE));

    AtomicReference<Pose2d> lastRobotPose = new AtomicReference<>(null);
    AtomicLong lastRobotPoseNs = new AtomicLong(0L);
    AtomicReference<Double> lastTimeToPlaneSec =
        new AtomicReference<>(ProjectileCycleRuntime.DEFAULT_TIME_TO_PLANE_SEC);
    AtomicReference<ShotSolution> lastValidShot = new AtomicReference<>(null);

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
              boolean piece = Boolean.TRUE.equals(hasPiece.get()) || currentPieceCount > 0L;
              double cap = ourSpeedCap != null ? Math.max(0.25, ourSpeedCap.get()) : 3.5;

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

              CategorySpec category;
              RepulsorSetpoint activeGoal;

              if (piece && shotProfileOpt.isPresent()) {
                category = CategorySpec.kScore;
                Pose2d shootFieldPose = aim.shootPose();

                DriverStation.Alliance alliance =
                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
                Pose2d bluePose =
                    alliance == DriverStation.Alliance.Red
                        ? SetpointUtil.flipToBlue(shootFieldPose)
                        : shootFieldPose;
                shuttleBluePoseRef.set(bluePose);
                activeGoal = shuttleShootRoute;
              } else {
                category = CategorySpec.kCollect;
                Pose2d collectGoal =
                    directiveCollectGoal(ctx, robotPose, Intent.SCORE_AVAILABLE_RESOURCES)
                        .orElseGet(() -> chooseRecoveryCollectGoalBlue(robotPose, cap, geometry));
                collectBluePoseRef.set(collectGoal);
                activeGoal = collectRoute;
              }

              ctx.repulsor.setCurrentGoal(activeGoal);
              Pose2d goalPose = activeGoal.get(spCtx);
              ctx.planner.setRequestedGoal(goalPose);

              boolean readyToShoot =
                  piece
                      && ProjectileCycleRuntime.canRelease(
                          aim, robotPose, goalPose, SHOOT_POS_TOL_METERS, SHOOT_YAW_TOL_DEG);
              boolean mechanismAtSetpoint = Boolean.TRUE.equals(mechanismReady.get());
              boolean allowPassthrough =
                  readyToShoot && currentPieceCount > 0L && mechanismAtSetpoint;
              Logger.recordOutput("Repulsor/ShuttleRecovery/ReadyToRelease", readyToShoot);
              Logger.recordOutput("Repulsor/ShuttleRecovery/MechanismReady", mechanismAtSetpoint);
              Logger.recordOutput("Repulsor/ShuttleRecovery/PassthroughAllowed", allowPassthrough);
              shooterPassthrough.set(allowPassthrough);

              if (!piece && currentPieceCount >= MAGAZINE_CAPACITY) {
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

              ctx.drive.runVelocity(
                  sample.asChassisSpeeds(
                      ctx.repulsor.getDrive().getOmegaPID(), robotPose.getRotation()));
            },
            ctx.drive)
        .finallyDo(
            interrupted -> {
              shooterPassthrough.set(false);
              ctx.drive.runVelocity(new ChassisSpeeds());
            });
  }

  private Pose2d chooseRecoveryCollectGoalBlue(
      Pose2d robotPose, double cap, FieldGeometry geometry) {
    Pose2d nextBlue =
        FieldTrackerCore.getInstance()
            .nextAllianceShuttleRecoveryGoalBlue(robotPose, cap, RECOVERY_GOAL_UNITS);
    if (nextBlue == null) {
      return new Pose2d(
          geometry.lengthMeters() * 0.25, geometry.widthMeters() * 0.5, robotPose.getRotation());
    }
    return new Pose2d(nextBlue.getTranslation(), nextBlue.getRotation());
  }

  private static Optional<Pose2d> directiveCollectGoal(
      BehaviourContext ctx, Pose2d robotPose, Intent expectedIntent) {
    StrategyDirective directive = ctx.repulsor.getStrategyDirective();
    if (directive.intent() != expectedIntent || !directive.hasResourceTarget()) {
      return Optional.empty();
    }
    return Optional.of(directive.targetPose(robotPose.getRotation()));
  }

  private static Optional<ProjectileShotAction> selectedScoreAction(BehaviourContext ctx) {
    StrategyDirective directive = ctx.repulsor.getStrategyDirective();
    if (directive.intent() == Intent.SCORE_AVAILABLE_RESOURCES
        && !"none".equals(directive.actionId())) {
      Optional<ProjectileShotAction> action =
          ctx.repulsor.getFieldDefinition().actionProfile().projectileShot(directive.actionId());
      if (action.isPresent()) {
        return action;
      }
    }
    return ctx.repulsor.getFieldDefinition().actionProfile().scoreProjectileShot();
  }
}
