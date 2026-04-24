package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldActionProfile.ShuttleShotProfile;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.MutablePoseSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointUtil;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;

public final class ShuttleRecoveryBehaviour extends Behaviour {
  private static final double SHOOT_POS_TOL_METERS = 0.34;
  private static final double SHOOT_YAW_TOL_DEG = 13.0;

  private static final double MOTION_COMP_LATENCY_SEC = 0.08;
  private static final double MOTION_COMP_MIN_LEAD_SEC = 0.10;
  private static final double MOTION_COMP_MAX_LEAD_SEC = 0.45;
  private static final double MOTION_COMP_MAX_SPEED_MPS = 4.5;
  private static final double DEFAULT_TIME_TO_PLANE_SEC = 0.18;
  private static final long MAGAZINE_CAPACITY = 16L;
  private static final int RECOVERY_GOAL_UNITS = 2;

  private final int prio;
  private final Supplier<Boolean> hasPiece;
  private final Supplier<Double> ourSpeedCap;

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

  public ShuttleRecoveryBehaviour(
      int priority, Supplier<Boolean> hasPiece, Supplier<Double> ourSpeedCap) {
    this.prio = priority;
    this.hasPiece = hasPiece;
    this.ourSpeedCap = ourSpeedCap;
  }

  @Override
  public String name() {
    return "ShuttleRecovery";
  }

  @Override
  public int priority() {
    return prio;
  }

  @Override
  public boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
    return flags.contains(BehaviourFlag.SHUTTLE_RECOVERY_MODE);
  }

  @Override
  public Command build(BehaviourContext ctx) {
    Optional<ShuttleShotProfile> shotProfileOpt =
        ctx.repulsor.getFieldDefinition().actionProfile().shuttleShot();
    FieldGeometry geometry = ctx.repulsor.getFieldDefinition().geometry();
    List<Obstacle> staticShotObstacles = new ArrayList<>();
    staticShotObstacles.addAll(ctx.repulsor.getFieldDefinition().walls());
    staticShotObstacles.addAll(ctx.repulsor.getFieldDefinition().fieldObstacles());

    AtomicReference<Pose2d> collectBluePoseRef = new AtomicReference<>(Pose2d.kZero);
    RepulsorSetpoint collectRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint(
                "SHUTTLE_RECOVERY_COLLECT_ROUTE", SetpointType.kOther, collectBluePoseRef),
            HeightSetpoint.NONE);

    AtomicReference<Pose2d> shuttleBluePoseRef = new AtomicReference<>(Pose2d.kZero);
    RepulsorSetpoint shuttleShootRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint(
                "SHUTTLE_RECOVERY_SHOOT_ROUTE", SetpointType.kScore, shuttleBluePoseRef),
            shotProfileOpt.map(ShuttleShotProfile::routeHeight).orElse(HeightSetpoint.NONE));

    AtomicReference<Pose2d> lastRobotPose = new AtomicReference<>(null);
    AtomicLong lastRobotPoseNs = new AtomicLong(0L);
    AtomicReference<Double> lastTimeToPlaneSec = new AtomicReference<>(DEFAULT_TIME_TO_PLANE_SEC);
    AtomicReference<ShotSolution> lastValidShot = new AtomicReference<>(null);

    return Commands.run(
            () -> {
              Pose2d robotPose = ctx.robotPose.get();
              long nowNs = System.nanoTime();
              SetpointContext spCtx = makeCtx(ctx, robotPose);
              Translation2d fieldVelocity =
                  estimateFieldVelocity(
                      lastRobotPose.get(), lastRobotPoseNs.get(), robotPose, nowNs);
              lastRobotPose.set(robotPose);
              lastRobotPoseNs.set(nowNs);

              long currentPieceCount = safePieceCount(pieceCount);
              boolean piece = Boolean.TRUE.equals(hasPiece.get()) || currentPieceCount > 0L;
              double cap = ourSpeedCap != null ? Math.max(0.25, ourSpeedCap.get()) : 3.5;

              ShuttleAim aim =
                  shotProfileOpt
                      .map(
                          profile ->
                              computeShuttleAim(
                                  profile,
                                  geometry,
                                  robotPose,
                                  spCtx,
                                  staticShotObstacles,
                                  ctx.vision.getObstacles(),
                                  fieldVelocity,
                                  lastTimeToPlaneSec))
                      .orElse(new ShuttleAim(robotPose, Optional.empty()));

              if (aim.shotSolution().isPresent()) {
                ShotSolution solution = aim.shotSolution().get();
                lastValidShot.set(solution);
                shotSpeed.set(solution.launchSpeedMetersPerSecond());
                shotAngle.set(solution.launchAngle().getDegrees());
              } else {
                ShotSolution prior = lastValidShot.get();
                if (prior != null) {
                  shotSpeed.set(prior.launchSpeedMetersPerSecond());
                  shotAngle.set(prior.launchAngle().getDegrees());
                } else {
                  shotSpeed.set(0.0);
                  shotAngle.set(0.0);
                }
              }

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
                Pose2d collectGoal = chooseRecoveryCollectGoalBlue(robotPose, cap, geometry);
                collectBluePoseRef.set(collectGoal);
                activeGoal = collectRoute;
              }

              ctx.repulsor.setCurrentGoal(activeGoal);
              Pose2d goalPose = activeGoal.get(spCtx);
              ctx.planner.setRequestedGoal(goalPose);

              boolean readyToShoot =
                  piece && isReadyToShoot(robotPose, goalPose) && aim.shotSolution().isPresent();
              boolean allowPassthrough = readyToShoot && currentPieceCount > 0L;
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
    DriverStation.Alliance wpAlliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    Pose2d robotPoseBlue =
        wpAlliance == DriverStation.Alliance.Red ? SetpointUtil.flipToRed(robotPose) : robotPose;

    Pose2d nextBlue =
        FieldTrackerCore.getInstance()
            .nextAllianceShuttleRecoveryGoalBlue(robotPoseBlue, cap, RECOVERY_GOAL_UNITS);
    if (nextBlue == null) {
      return new Pose2d(
          geometry.lengthMeters() * 0.25, geometry.widthMeters() * 0.5, robotPoseBlue.getRotation());
    }
    return new Pose2d(nextBlue.getTranslation(), nextBlue.getRotation());
  }

  private ShuttleAim computeShuttleAim(
      ShuttleShotProfile profile,
      FieldGeometry geometry,
      Pose2d robotPose,
      SetpointContext spCtx,
      List<Obstacle> staticObstacles,
      List<? extends Obstacle> obstacles,
      Translation2d fieldVelocity,
      AtomicReference<Double> lastTimeToPlaneSec) {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    Translation2d hubTarget = profile.target(alliance);

    double releaseH = Math.max(0.0, spCtx.shooterReleaseHeightMeters());
    double halfL = Math.max(0.0, spCtx.robotLengthMeters()) / 2.0;
    double halfW = Math.max(0.0, spCtx.robotWidthMeters()) / 2.0;

    double prevFlight =
        lastTimeToPlaneSec.get() == null ? DEFAULT_TIME_TO_PLANE_SEC : lastTimeToPlaneSec.get();
    double leadSec =
        MathUtil.clamp(
            prevFlight + MOTION_COMP_LATENCY_SEC,
            MOTION_COMP_MIN_LEAD_SEC,
            MOTION_COMP_MAX_LEAD_SEC);

    Translation2d compensatedTarget =
        hubTarget.minus(
            new Translation2d(fieldVelocity.getX() * leadSec, fieldVelocity.getY() * leadSec));
    compensatedTarget = geometry.clamp(compensatedTarget, profile.fieldMarginMeters());

    Optional<ShotSolution> solved =
        solveShuttleShot(
            robotPose.getTranslation(),
            hubTarget,
            compensatedTarget,
            profile,
            geometry,
            staticObstacles,
            releaseH,
            halfL,
            halfW,
            obstacles,
            alliance);
    if (solved.isPresent()) {
      ShotSolution solution = solved.get();
      lastTimeToPlaneSec.set(
          MathUtil.clamp(
              solution.timeToPlaneSeconds(), MOTION_COMP_MIN_LEAD_SEC, MOTION_COMP_MAX_LEAD_SEC));
      return new ShuttleAim(new Pose2d(solution.shooterPosition(), solution.shooterYaw()), solved);
    }

    return new ShuttleAim(fallbackShuttlePose(hubTarget, alliance, profile, geometry), Optional.empty());
  }

  private Optional<ShotSolution> solveShuttleShot(
      Translation2d robotPos,
      Translation2d hubTarget,
      Translation2d compensatedTarget,
      ShuttleShotProfile profile,
      FieldGeometry geometry,
      List<Obstacle> staticObstacles,
      double shooterReleaseHeightMeters,
      double halfL,
      double halfW,
      List<? extends Obstacle> obstacles,
      DriverStation.Alliance alliance) {
    Translation2d behind = behindDirection(alliance);
    Translation2d lateral = new Translation2d(-behind.getY(), behind.getX());
    Translation2d base =
        geometry.clamp(
            hubTarget.plus(
                new Translation2d(
                    behind.getX() * profile.behindTargetMeters(),
                    behind.getY() * profile.behindTargetMeters())),
            profile.fieldMarginMeters());

    ShotSolution best = null;
    double bestScore = Double.POSITIVE_INFINITY;

    for (double lateralOffset : profile.lateralOffsetsMeters()) {
      Translation2d shooterPos =
          geometry.clamp(
              base.plus(
                  new Translation2d(
                      lateral.getX() * lateralOffset, lateral.getY() * lateralOffset)),
              profile.fieldMarginMeters());

      if (!isShooterPoseValid(
          shooterPos, hubTarget, halfL, halfW, staticObstacles, obstacles, geometry)) {
        continue;
      }

      Optional<ShotSolution> solved =
          DragShotPlanner.calculateStaticShotAngleAndSpeed(
              profile.gamePiecePhysics(),
              shooterPos,
              compensatedTarget,
              profile.targetHeightMeters(),
              shooterReleaseHeightMeters,
              profile.constraints());
      if (solved.isEmpty()) {
        continue;
      }

      ShotSolution candidate = solved.get();
      double score =
          Math.abs(candidate.verticalErrorMeters())
              + 0.02 * robotPos.getDistance(candidate.shooterPosition());
      if (score < bestScore) {
        bestScore = score;
        best = candidate;
      }
    }

    return Optional.ofNullable(best);
  }

  private static SetpointContext makeCtx(BehaviourContext ctx, Pose2d robotPose) {
    double release;
    try {
      var ht = ctx.repulsor.getTargetHeight();
      var d = ht != null ? ht.getHeight() : null;
      release = d != null ? Math.max(0.0, d.in(Meters)) : 0.0;
    } catch (Exception ignored) {
      release = 0.0;
    }
    return new SetpointContext(
        Optional.of(robotPose),
        Math.max(0.0, ctx.robot_x) * 2.0,
        Math.max(0.0, ctx.robot_y) * 2.0,
        release,
        ctx.vision.getObstacles());
  }

  private static double shortestAngleRad(double from, double to) {
    return MathUtil.angleModulus(to - from);
  }

  private static boolean nearPose(Pose2d a, Pose2d b, double posTol, double degTol) {
    if (a.getTranslation().getDistance(b.getTranslation()) > posTol) {
      return false;
    }
    double e =
        Math.abs(shortestAngleRad(a.getRotation().getRadians(), b.getRotation().getRadians()));
    return e <= Math.toRadians(degTol);
  }

  private static boolean isReadyToShoot(Pose2d robotPose, Pose2d goalPose) {
    return nearPose(robotPose, goalPose, SHOOT_POS_TOL_METERS, SHOOT_YAW_TOL_DEG);
  }

  private static Translation2d estimateFieldVelocity(
      Pose2d prevPose, long prevNs, Pose2d nowPose, long nowNs) {
    if (prevPose == null || prevNs == 0L || nowNs <= prevNs) {
      return new Translation2d();
    }
    double dt = (nowNs - prevNs) * 1e-9;
    if (dt < 1e-4) {
      return new Translation2d();
    }
    double vx = (nowPose.getX() - prevPose.getX()) / dt;
    double vy = (nowPose.getY() - prevPose.getY()) / dt;

    double speed = Math.hypot(vx, vy);
    if (speed > MOTION_COMP_MAX_SPEED_MPS && speed > 1e-6) {
      double s = MOTION_COMP_MAX_SPEED_MPS / speed;
      vx *= s;
      vy *= s;
    }
    return new Translation2d(vx, vy);
  }

  private static Pose2d fallbackShuttlePose(
      Translation2d hubTarget,
      DriverStation.Alliance alliance,
      ShuttleShotProfile profile,
      FieldGeometry geometry) {
    Translation2d behind = behindDirection(alliance);
    Translation2d shooterPos =
        geometry.clamp(
            hubTarget.plus(
                new Translation2d(
                    behind.getX() * profile.behindTargetMeters(),
                    behind.getY() * profile.behindTargetMeters())),
            profile.fieldMarginMeters());
    Rotation2d yaw = hubTarget.minus(shooterPos).getAngle();
    return new Pose2d(shooterPos, yaw);
  }

  private static Translation2d behindDirection(DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Red) {
      return new Translation2d(1.0, 0.0);
    }
    return new Translation2d(-1.0, 0.0);
  }

  private static boolean isShooterPoseValid(
      Translation2d shooterPos,
      Translation2d targetFieldPosition,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<Obstacle> staticObstacles,
      List<? extends Obstacle> dynamicObstacles,
      FieldGeometry geometry) {
    if (!geometry.contains(shooterPos)) return false;

    Translation2d delta = targetFieldPosition.minus(shooterPos);
    Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(delta.getY(), delta.getX()));
    Translation2d[] rect =
        FieldPlanner.robotRect(shooterPos, yaw, robotHalfLengthMeters, robotHalfWidthMeters);

    for (Obstacle obstacle : staticObstacles) {
      if (obstacle.intersectsRectangle(rect)) return false;
    }
    if (dynamicObstacles != null) {
      for (Obstacle obstacle : dynamicObstacles) {
        if (obstacle.intersectsRectangle(rect)) return false;
      }
    }
    return true;
  }

  private static long safePieceCount(NetworkTablesValue<Long> countValue) {
    Long value = countValue.get();
    if (value == null) {
      return 0L;
    }
    return Math.max(0L, value);
  }

  private record ShuttleAim(Pose2d shootPose, Optional<ShotSolution> shotSolution) {}
}
