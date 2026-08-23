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

package org.curtinfrc.frc2026.util.Repulsor.Behaviours.Runtime;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourContext;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldActionProfile.ProjectileShotAction;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.RepulsorDiagnostics;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.MovingShotSolver;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;
import org.littletonrobotics.junction.Logger;

/**
 * Shared runtime utilities for behaviours that collect, transfer, or score projectile resources.
 * The class centralizes field-relative aim computation, shot telemetry publication, release gates,
 * and moving-shot compensation so individual behaviours only express their specific sequence.
 */
public final class ProjectileCycleRuntime {
  /** Default measured command-to-release latency in seconds; tune per robot mechanism. */
  public static final double MOTION_COMP_LATENCY_SEC = 0.08;

  /** Minimum moving-shot lead time in seconds used to prevent overreacting to noisy estimates. */
  public static final double MOTION_COMP_MIN_LEAD_SEC = 0.10;

  /** Maximum moving-shot lead time in seconds used to bound prediction error. */
  public static final double MOTION_COMP_MAX_LEAD_SEC = 0.45;

  /** Maximum field-relative speed, in meters per second, considered for motion compensation. */
  public static final double MOTION_COMP_MAX_SPEED_MPS = 4.5;

  /** Default projectile time-to-target-plane estimate in seconds before a solution is available. */
  public static final double DEFAULT_TIME_TO_PLANE_SEC = 0.18;

  /** Default moving-shot tuning used when an action profile does not provide custom values. */
  public static final MovingShotSolver.Config DEFAULT_MOVING_SHOT_CONFIG =
      MovingShotSolver.Config.defaults();

  private ProjectileCycleRuntime() {}

  /**
   * Aim result consumed by behaviours and shooter IO. Static and moving-shot solutions are both
   * carried so the behaviour can fall back to a stop-and-shoot pose when moving release gates are
   * not satisfied.
   *
   * @param shootPose field-relative static shot pose selected by the profile search
   * @param shotSolution static shot solution for the selected pose
   * @param movingShot optional moving-shot result using the current field-relative velocity
   */
  public record Aim(
      Pose2d shootPose,
      Optional<ShotSolution> shotSolution,
      Optional<MovingShotSolver.Result> movingShot) {
    /**
     * Creates an aim result without moving-shot compensation.
     *
     * @param shootPose field-relative pose the robot should drive toward
     * @param shotSolution static projectile solution at that pose
     */
    public Aim(Pose2d shootPose, Optional<ShotSolution> shotSolution) {
      this(shootPose, shotSolution, Optional.empty());
    }

    /**
     * Selects the solution that should currently be published to the shooter mechanism.
     *
     * @return moving-shot solution when present, otherwise the static solution
     */
    public Optional<ShotSolution> activeShotSolution() {
      if (movingShot.isPresent()) {
        return Optional.of(movingShot.get().solution());
      }
      return shotSolution;
    }

    /**
     * Selects the pose that should currently be used for heading and release checks.
     *
     * @return predicted moving release pose when present, otherwise the static shot pose
     */
    public Pose2d activeShootPose() {
      return movingShot.map(MovingShotSolver.Result::predictedReleasePose).orElse(shootPose);
    }

    /**
     * Reports whether moving-shot gates allow release without stopping at the static pose.
     *
     * @return true when a moving-shot result exists and all release gates are satisfied
     */
    public boolean readyToReleaseMoving() {
      return movingShot.map(MovingShotSolver.Result::readyToRelease).orElse(false);
    }
  }

  /**
   * Builds a setpoint context from the current behaviour state.
   *
   * @param ctx behaviour runtime context containing robot dimensions, vision obstacles, and
   *     mechanism target height
   * @param robotPose current robot {@link Pose2d} in field-relative coordinates
   * @return setpoint context for route resolution and shot planning
   */
  public static SetpointContext makeCtx(BehaviourContext ctx, Pose2d robotPose) {
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

  /**
   * Builds a setpoint context from the current behaviour state using an explicit release-height
   * source instead of the mechanism target height.
   *
   * @param ctx behaviour runtime context containing robot dimensions and vision obstacles
   * @param robotPose current robot {@link Pose2d} in field-relative coordinates
   * @param releaseHeightMeters supplier of the shooter release height in meters
   * @return setpoint context for route resolution and shot planning
   */
  public static SetpointContext makeCtx(
      BehaviourContext ctx, Pose2d robotPose, DoubleSupplier releaseHeightMeters) {
    return new SetpointContext(
        Optional.ofNullable(robotPose),
        Math.max(0.0, ctx.robot_x) * 2.0,
        Math.max(0.0, ctx.robot_y) * 2.0,
        Math.max(0.0, releaseHeightMeters.getAsDouble()),
        ctx.vision.getObstacles());
  }

  /**
   * Estimates field-relative chassis translation velocity from two pose samples.
   *
   * @param prevPose previous field-relative robot pose
   * @param prevNs previous timestamp in nanoseconds
   * @param nowPose current field-relative robot pose
   * @param nowNs current timestamp in nanoseconds
   * @return clamped field-relative velocity in meters per second, or zero when samples are invalid
   */
  public static Translation2d estimateFieldVelocity(
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

  /**
   * Computes aim using the moving-shot tuning stored on the projectile action profile.
   *
   * @param profile configured projectile action for the current field profile
   * @param geometry field geometry used to clamp candidate shot poses
   * @param robotPose current robot pose in field-relative coordinates
   * @param spCtx setpoint context containing robot dimensions, release height, and obstacles
   * @param staticObstacles static field obstacles used to reject unsafe shot poses
   * @param obstacles dynamic obstacles used to reject unsafe shot poses
   * @param fieldVelocity estimated field-relative robot velocity in meters per second
   * @param lastTimeToPlaneSec mutable cache of the previous projectile flight-time estimate
   * @return static and optional moving-shot aim result for the behaviour to execute
   */
  public static Aim computeAim(
      ProjectileShotAction profile,
      FieldGeometry geometry,
      Pose2d robotPose,
      SetpointContext spCtx,
      List<Obstacle> staticObstacles,
      List<? extends Obstacle> obstacles,
      Translation2d fieldVelocity,
      AtomicReference<Double> lastTimeToPlaneSec) {
    return computeAim(
        profile,
        geometry,
        robotPose,
        spCtx,
        staticObstacles,
        obstacles,
        fieldVelocity,
        lastTimeToPlaneSec,
        profile.movingShotConfig());
  }

  /**
   * Computes a projectile aim solution for the current robot pose and field state.
   *
   * <p>The static solution searches legal stand-off poses behind the target and rejects poses whose
   * robot footprint intersects static or dynamic obstacles. When the action enables moving shots,
   * the method also solves a velocity-compensated release using {@link MovingShotSolver}.
   *
   * @param profile configured projectile action for the current field profile
   * @param geometry field geometry used to clamp candidate shot poses
   * @param robotPose current robot pose in field-relative coordinates
   * @param spCtx setpoint context containing robot dimensions, release height, and obstacles
   * @param staticObstacles static field obstacles used to reject unsafe shot poses
   * @param obstacles dynamic obstacles used to reject unsafe shot poses
   * @param fieldVelocity estimated field-relative robot velocity in meters per second
   * @param lastTimeToPlaneSec mutable cache updated with the latest valid flight-time estimate
   * @param movingShotConfig tuning used by the velocity-compensated shot solver
   * @return static and optional moving-shot aim result for the behaviour to execute
   */
  public static Aim computeAim(
      ProjectileShotAction profile,
      FieldGeometry geometry,
      Pose2d robotPose,
      SetpointContext spCtx,
      List<Obstacle> staticObstacles,
      List<? extends Obstacle> obstacles,
      Translation2d fieldVelocity,
      AtomicReference<Double> lastTimeToPlaneSec,
      MovingShotSolver.Config movingShotConfig) {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    Translation2d target = profile.target(alliance);

    double releaseH = actionReleaseHeightMeters(profile, spCtx);
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
        target.minus(
            new Translation2d(fieldVelocity.getX() * leadSec, fieldVelocity.getY() * leadSec));
    compensatedTarget = geometry.clamp(compensatedTarget, profile.fieldMarginMeters());

    Optional<ShotSolution> solved =
        solveShot(
            robotPose.getTranslation(),
            target,
            compensatedTarget,
            profile,
            geometry,
            staticObstacles,
            releaseH,
            halfL,
            halfW,
            obstacles,
            alliance);
    Optional<MovingShotSolver.Result> movingShot =
        profile.movingShotEnabled()
            ? MovingShotSolver.solve(
                new MovingShotSolver.Request(
                    profile.gamePiecePhysics(),
                    target,
                    profile.targetHeightMeters(),
                    robotPose.getTranslation(),
                    robotPose.getRotation(),
                    fieldVelocity,
                    releaseH,
                    profile.constraints(),
                    movingShotConfig,
                    prevFlight))
            : Optional.empty();
    movingShot.ifPresent(
        result ->
            lastTimeToPlaneSec.set(
                MathUtil.clamp(
                    result.solution().timeToPlaneSeconds(),
                    MOTION_COMP_MIN_LEAD_SEC,
                    MOTION_COMP_MAX_LEAD_SEC)));

    if (solved.isPresent()) {
      ShotSolution solution = solved.get();
      if (movingShot.isEmpty()) {
        lastTimeToPlaneSec.set(
            MathUtil.clamp(
                solution.timeToPlaneSeconds(), MOTION_COMP_MIN_LEAD_SEC, MOTION_COMP_MAX_LEAD_SEC));
      }
      return new Aim(
          new Pose2d(solution.shooterPosition(), solution.shooterYaw()), solved, movingShot);
    }

    return new Aim(
        fallbackShootPose(target, alliance, profile, geometry), Optional.empty(), movingShot);
  }

  /**
   * Publishes shot speed and angle outputs for the active aim solution.
   *
   * <p>If a current solution is unavailable, the last valid solution is reused to avoid abrupt
   * mechanism commands. If no solution has ever been valid, both outputs are set to zero.
   *
   * @param aim aim result computed for the current cycle
   * @param lastValidShot mutable cache of the previous valid shot solution
   * @param shotSpeed NetworkTables-backed output for launch speed in meters per second
   * @param shotAngle NetworkTables-backed output for launch angle in degrees
   */
  public static void publishShotTelemetry(
      Aim aim,
      AtomicReference<ShotSolution> lastValidShot,
      NetworkTablesValue<Double> shotSpeed,
      NetworkTablesValue<Double> shotAngle) {
    publishMovingShotTelemetry(aim);
    Optional<ShotSolution> active = aim.activeShotSolution();
    if (active.isPresent()) {
      ShotSolution solution = active.get();
      lastValidShot.set(solution);
      shotSpeed.set(solution.launchSpeedMetersPerSecond());
      shotAngle.set(solution.launchAngle().getDegrees());
      return;
    }

    ShotSolution prior = lastValidShot.get();
    if (prior != null) {
      shotSpeed.set(prior.launchSpeedMetersPerSecond());
      shotAngle.set(prior.launchAngle().getDegrees());
    } else {
      shotSpeed.set(0.0);
      shotAngle.set(0.0);
    }
  }

  /**
   * Checks whether the robot is close enough to a static shot pose to release.
   *
   * @param robotPose current robot pose in field-relative coordinates
   * @param goalPose desired shot pose in field-relative coordinates
   * @param posTolMeters allowed translational error in meters
   * @param yawTolDeg allowed heading error in degrees
   * @return true when both translation and yaw are inside tolerance
   */
  public static boolean isReadyToShoot(
      Pose2d robotPose, Pose2d goalPose, double posTolMeters, double yawTolDeg) {
    if (robotPose.getTranslation().getDistance(goalPose.getTranslation()) > posTolMeters) {
      return false;
    }
    double e =
        Math.abs(
            shortestAngleRad(
                robotPose.getRotation().getRadians(), goalPose.getRotation().getRadians()));
    return e <= Math.toRadians(yawTolDeg);
  }

  /**
   * Checks whether a behaviour may release a projectile this cycle.
   *
   * <p>Moving-shot readiness bypasses the static pose tolerance because {@link MovingShotSolver}
   * already evaluates yaw, speed, and vertical-error gates for the predicted release pose.
   *
   * @param aim current aim solution
   * @param robotPose current robot pose in field-relative coordinates
   * @param goalPose static shot pose in field-relative coordinates
   * @param posTolMeters allowed static translational error in meters
   * @param yawTolDeg allowed static heading error in degrees
   * @return true when moving-shot gates pass or the static shot pose is reached
   */
  public static boolean canRelease(
      Aim aim, Pose2d robotPose, Pose2d goalPose, double posTolMeters, double yawTolDeg) {
    if (aim.readyToReleaseMoving()) {
      return true;
    }
    return aim.shotSolution().isPresent()
        && isReadyToShoot(robotPose, goalPose, posTolMeters, yawTolDeg);
  }

  private static void publishMovingShotTelemetry(Aim aim) {
    Optional<MovingShotSolver.Result> moving = aim.movingShot();
    Logger.recordOutput("Repulsor/MovingShot/Solved", moving.isPresent());
    Logger.recordOutput("Repulsor/MovingShot/Ready", aim.readyToReleaseMoving());
    if (moving.isEmpty()) {
      Logger.recordOutput("Repulsor/MovingShot/YawErrorDeg", 0.0);
      Logger.recordOutput("Repulsor/MovingShot/VerticalErrorMeters", 0.0);
      Logger.recordOutput("Repulsor/MovingShot/ReleaseSpeedAllowed", false);
      Logger.recordOutput("Repulsor/MovingShot/YawAligned", false);
      Logger.recordOutput("Repulsor/MovingShot/VerticalErrorAllowed", false);
      Logger.recordOutput("Repulsor/MovingShot/FlightPredictionSeconds", 0.0);
      Logger.recordOutput("Repulsor/MovingShot/CompensatedVelocityMps", 0.0);
      return;
    }

    MovingShotSolver.Result result = moving.get();
    Logger.recordOutput("Repulsor/MovingShot/PredictedReleasePose", result.predictedReleasePose());
    Logger.recordOutput(
        "Repulsor/MovingShot/CompensatedTarget",
        new Pose2d(result.compensatedTarget(), Rotation2d.kZero));
    Logger.recordOutput(
        "Repulsor/MovingShot/YawErrorDeg", Math.toDegrees(result.yawErrorRadians()));
    Logger.recordOutput(
        "Repulsor/MovingShot/VerticalErrorMeters", result.solution().verticalErrorMeters());
    Logger.recordOutput("Repulsor/MovingShot/ReleaseSpeedAllowed", result.releaseSpeedAllowed());
    Logger.recordOutput("Repulsor/MovingShot/YawAligned", result.yawAligned());
    Logger.recordOutput("Repulsor/MovingShot/VerticalErrorAllowed", result.verticalErrorAllowed());
    Logger.recordOutput(
        "Repulsor/MovingShot/FlightPredictionSeconds", result.flightPredictionSeconds());
    Logger.recordOutput(
        "Repulsor/MovingShot/CompensatedVelocityMps",
        result.compensatedShooterVelocity().getNorm());
  }

  /**
   * Returns the shortest angle rad value maintained by this Repulsor component.
   *
   * @param from value used by this operation.
   * @param to value used by this operation.
   * @return value produced by this operation.
   */
  public static double shortestAngleRad(double from, double to) {
    return MathUtil.angleModulus(to - from);
  }

  /**
   * Returns the safe piece count value maintained by this Repulsor component.
   *
   * @param countValue value used by this operation.
   * @return value produced by this operation.
   */
  public static long safePieceCount(NetworkTablesValue<Long> countValue) {
    Long value = countValue.get();
    if (value == null) {
      return 0L;
    }
    return Math.max(0L, value);
  }

  private static Optional<ShotSolution> solveShot(
      Translation2d robotPos,
      Translation2d target,
      Translation2d compensatedTarget,
      ProjectileShotAction profile,
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
            target.plus(
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
          shooterPos, target, halfL, halfW, staticObstacles, obstacles, geometry)) {
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

  private static double actionReleaseHeightMeters(
      ProjectileShotAction profile, SetpointContext spCtx) {
    try {
      var height = profile.routeMechanismSetpoint();
      var distance = height != null ? height.getHeight() : null;
      if (distance != null) {
        return Math.max(0.0, distance.in(Meters));
      }
    } catch (RuntimeException ex) {
      RepulsorDiagnostics.warnThrottled(
          "ProjectileCycle/releaseHeight",
          "Failed to read projectile release height from mechanism setpoint, using context fallback: "
              + ex.getMessage(),
          5.0);
    }
    return Math.max(0.0, spCtx.shooterReleaseHeightMeters());
  }

  private static Pose2d fallbackShootPose(
      Translation2d target,
      DriverStation.Alliance alliance,
      ProjectileShotAction profile,
      FieldGeometry geometry) {
    Translation2d behind = behindDirection(alliance);
    Translation2d shooterPos =
        geometry.clamp(
            target.plus(
                new Translation2d(
                    behind.getX() * profile.behindTargetMeters(),
                    behind.getY() * profile.behindTargetMeters())),
            profile.fieldMarginMeters());
    Rotation2d yaw = target.minus(shooterPos).getAngle();
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
}
