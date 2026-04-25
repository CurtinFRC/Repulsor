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
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourContext;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldActionProfile.ProjectileShotAction;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;

public final class ProjectileCycleRuntime {
  public static final double MOTION_COMP_LATENCY_SEC = 0.08;
  public static final double MOTION_COMP_MIN_LEAD_SEC = 0.10;
  public static final double MOTION_COMP_MAX_LEAD_SEC = 0.45;
  public static final double MOTION_COMP_MAX_SPEED_MPS = 4.5;
  public static final double DEFAULT_TIME_TO_PLANE_SEC = 0.18;

  private ProjectileCycleRuntime() {}

  public record Aim(Pose2d shootPose, Optional<ShotSolution> shotSolution) {}

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

  public static Aim computeAim(
      ProjectileShotAction profile,
      FieldGeometry geometry,
      Pose2d robotPose,
      SetpointContext spCtx,
      List<Obstacle> staticObstacles,
      List<? extends Obstacle> obstacles,
      Translation2d fieldVelocity,
      AtomicReference<Double> lastTimeToPlaneSec) {
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

    if (solved.isPresent()) {
      ShotSolution solution = solved.get();
      lastTimeToPlaneSec.set(
          MathUtil.clamp(
              solution.timeToPlaneSeconds(), MOTION_COMP_MIN_LEAD_SEC, MOTION_COMP_MAX_LEAD_SEC));
      return new Aim(new Pose2d(solution.shooterPosition(), solution.shooterYaw()), solved);
    }

    return new Aim(fallbackShootPose(target, alliance, profile, geometry), Optional.empty());
  }

  public static void publishShotTelemetry(
      Aim aim,
      AtomicReference<ShotSolution> lastValidShot,
      NetworkTablesValue<Double> shotSpeed,
      NetworkTablesValue<Double> shotAngle) {
    if (aim.shotSolution().isPresent()) {
      ShotSolution solution = aim.shotSolution().get();
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

  public static double shortestAngleRad(double from, double to) {
    return MathUtil.angleModulus(to - from);
  }

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
    } catch (RuntimeException ignored) {
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
