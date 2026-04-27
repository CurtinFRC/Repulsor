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

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

/**
 * Solves a projectile shot for a moving shooter by predicting the release pose and compensating the
 * target for inherited robot velocity. This intentionally depends only on generic projectile data,
 * not on a specific field, game piece name, or behaviour.
 *
 * <p>Inputs use WPILib field-relative {@link Translation2d} and {@link Pose2d} coordinates. The
 * solver is suitable for scoring, passing, shuttling, or any other action where a projectile leaves
 * a moving robot and should intersect a stationary field target.
 */
public final class MovingShotSolver {
  private MovingShotSolver() {}

  /**
   * Tuning values for release prediction and safety gates. These values are profile-tunable because
   * mechanism latency, acceptable release speed, and aim tolerance vary by robot and projectile.
   *
   * @param releaseLatencySeconds expected delay, in seconds, between command and projectile release
   * @param minFlightPredictionSeconds lower bound for flight-time lead compensation
   * @param maxFlightPredictionSeconds upper bound for flight-time lead compensation
   * @param defaultFlightPredictionSeconds initial lead-time estimate before a shot solution exists
   * @param maxCompensatedSpeedMetersPerSecond maximum field-relative chassis speed considered for
   *     target compensation
   * @param maxReleaseSpeedMetersPerSecond maximum field-relative chassis speed allowed for release
   * @param yawToleranceDegrees maximum yaw error, in degrees, for {@code readyToRelease}
   * @param maxVerticalErrorMeters maximum absolute target-plane vertical error, in meters
   * @param iterations number of compensation/shot-solve refinement passes
   */
  public record Config(
      double releaseLatencySeconds,
      double minFlightPredictionSeconds,
      double maxFlightPredictionSeconds,
      double defaultFlightPredictionSeconds,
      double maxCompensatedSpeedMetersPerSecond,
      double maxReleaseSpeedMetersPerSecond,
      double yawToleranceDegrees,
      double maxVerticalErrorMeters,
      int iterations) {
    public Config {
      releaseLatencySeconds = Math.max(0.0, releaseLatencySeconds);
      minFlightPredictionSeconds = Math.max(0.0, minFlightPredictionSeconds);
      maxFlightPredictionSeconds = Math.max(minFlightPredictionSeconds, maxFlightPredictionSeconds);
      defaultFlightPredictionSeconds =
          MathUtil.clamp(
              defaultFlightPredictionSeconds,
              minFlightPredictionSeconds,
              maxFlightPredictionSeconds);
      maxCompensatedSpeedMetersPerSecond = Math.max(0.0, maxCompensatedSpeedMetersPerSecond);
      maxReleaseSpeedMetersPerSecond = Math.max(0.0, maxReleaseSpeedMetersPerSecond);
      yawToleranceDegrees = Math.max(0.0, yawToleranceDegrees);
      maxVerticalErrorMeters = Math.max(0.0, maxVerticalErrorMeters);
      iterations = Math.max(1, iterations);
    }

    /**
     * Creates conservative default tuning for a fast FRC projectile mechanism.
     *
     * @return default moving-shot configuration
     */
    public static Config defaults() {
      return new Config(0.08, 0.10, 0.45, 0.18, 4.5, 4.5, 13.0, 0.20, 3);
    }
  }

  /**
   * Complete input snapshot for one moving-shot solve. Callers should build this from the current
   * robot pose, estimated field-relative velocity, active projectile physics, and selected target.
   *
   * @param gamePiecePhysics projectile drag/mass model used by the static shot solver
   * @param targetFieldPosition field-relative target position in meters
   * @param targetHeightMeters vertical height of the target plane in meters
   * @param shooterFieldPosition current field-relative shooter position in meters
   * @param shooterYaw current field-relative shooter yaw
   * @param shooterFieldVelocity estimated field-relative shooter velocity in meters per second
   * @param shooterReleaseHeightMeters projectile release height above carpet in meters
   * @param constraints allowed launch speed, angle, and yaw domain for the projectile
   * @param config moving-shot tuning; defaults are used when {@code null}
   * @param previousFlightTimeSeconds previous solved flight time used as the next lead-time seed
   */
  public record Request(
      GamePiecePhysics gamePiecePhysics,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      Translation2d shooterFieldPosition,
      Rotation2d shooterYaw,
      Translation2d shooterFieldVelocity,
      double shooterReleaseHeightMeters,
      Constraints constraints,
      Config config,
      Double previousFlightTimeSeconds) {
    public Request {
      if (gamePiecePhysics == null) {
        throw new IllegalArgumentException("gamePiecePhysics cannot be null");
      }
      if (constraints == null) {
        throw new IllegalArgumentException("constraints cannot be null");
      }
      targetFieldPosition = targetFieldPosition == null ? new Translation2d() : targetFieldPosition;
      shooterFieldPosition =
          shooterFieldPosition == null ? new Translation2d() : shooterFieldPosition;
      shooterYaw = shooterYaw == null ? Rotation2d.kZero : shooterYaw;
      shooterFieldVelocity =
          shooterFieldVelocity == null ? new Translation2d() : shooterFieldVelocity;
      shooterReleaseHeightMeters = Math.max(0.0, shooterReleaseHeightMeters);
      config = config == null ? Config.defaults() : config;
    }
  }

  /**
   * Output from one moving-shot solve, including the compensated shot and release gates used by
   * behaviours. The contained {@link ShotSolution} is the active solution to publish to shooter
   * mechanisms.
   *
   * @param solution compensated projectile solution at the predicted release pose
   * @param predictedReleasePose field-relative pose where the projectile is expected to leave the
   *     robot
   * @param compensatedTarget field-relative target adjusted opposite robot velocity
   * @param releaseLatencySeconds latency used to predict the release pose
   * @param flightPredictionSeconds final flight-time estimate used for compensation
   * @param compensatedShooterVelocity field-relative velocity after speed clamping
   * @param yawErrorRadians absolute yaw error between current robot heading and solved shot yaw
   * @param yawAligned whether yaw error is inside the configured tolerance
   * @param releaseSpeedAllowed whether chassis speed is inside the configured release limit
   * @param verticalErrorAllowed whether solved vertical error is inside the configured tolerance
   * @param readyToRelease whether all moving-shot release gates are satisfied
   */
  public record Result(
      ShotSolution solution,
      Pose2d predictedReleasePose,
      Translation2d compensatedTarget,
      double releaseLatencySeconds,
      double flightPredictionSeconds,
      Translation2d compensatedShooterVelocity,
      double yawErrorRadians,
      boolean yawAligned,
      boolean releaseSpeedAllowed,
      boolean verticalErrorAllowed,
      boolean readyToRelease) {}

  /**
   * Solves a field-relative moving projectile shot for the provided request.
   *
   * <p>The method iteratively predicts release pose, compensates the target by inherited shooter
   * velocity, and delegates the static projectile solve to {@link DragShotPlanner}. It does not
   * mutate global state; callers normally retain {@link Result#flightPredictionSeconds()} to seed
   * the next periodic solve.
   *
   * @param request complete shot request; required physics and constraints must be non-null
   * @return solved moving-shot result, or {@link Optional#empty()} when no valid static shot exists
   *     for the compensated target
   */
  public static Optional<Result> solve(Request request) {
    Config config = request.config();
    Translation2d velocity =
        clampVelocity(request.shooterFieldVelocity(), config.maxCompensatedSpeedMetersPerSecond());

    double speed = velocity.getNorm();
    boolean releaseSpeedAllowed = speed <= config.maxReleaseSpeedMetersPerSecond() + 1e-9;
    double flightPrediction =
        request.previousFlightTimeSeconds() == null
            ? config.defaultFlightPredictionSeconds()
            : request.previousFlightTimeSeconds();

    ShotSolution solution = null;
    Translation2d predictedReleasePosition = request.shooterFieldPosition();
    Translation2d compensatedTarget = request.targetFieldPosition();

    for (int i = 0; i < config.iterations(); i++) {
      double clampedFlight =
          MathUtil.clamp(
              flightPrediction,
              config.minFlightPredictionSeconds(),
              config.maxFlightPredictionSeconds());
      predictedReleasePosition =
          request
              .shooterFieldPosition()
              .plus(
                  new Translation2d(velocity.getX(), velocity.getY())
                      .times(config.releaseLatencySeconds()));
      compensatedTarget =
          request
              .targetFieldPosition()
              .minus(new Translation2d(velocity.getX(), velocity.getY()).times(clampedFlight));

      Optional<ShotSolution> solved =
          DragShotPlanner.calculateStaticShotAngleAndSpeed(
              request.gamePiecePhysics(),
              predictedReleasePosition,
              compensatedTarget,
              request.targetHeightMeters(),
              request.shooterReleaseHeightMeters(),
              request.constraints());
      if (solved.isEmpty()) {
        return Optional.empty();
      }

      solution = solved.get();
      flightPrediction = solution.timeToPlaneSeconds();
    }

    double yawError =
        Math.abs(
            shortestAngleRad(
                request.shooterYaw().getRadians(), solution.shooterYaw().getRadians()));
    boolean yawAligned = yawError <= Math.toRadians(config.yawToleranceDegrees());
    boolean verticalErrorAllowed =
        Math.abs(solution.verticalErrorMeters()) <= config.maxVerticalErrorMeters();
    boolean ready = yawAligned && releaseSpeedAllowed && verticalErrorAllowed;

    return Optional.of(
        new Result(
            solution,
            new Pose2d(predictedReleasePosition, solution.shooterYaw()),
            compensatedTarget,
            config.releaseLatencySeconds(),
            flightPrediction,
            velocity,
            yawError,
            yawAligned,
            releaseSpeedAllowed,
            verticalErrorAllowed,
            ready));
  }

  private static Translation2d clampVelocity(Translation2d velocity, double maxSpeed) {
    double speed = velocity.getNorm();
    if (speed <= maxSpeed || speed < 1e-9) {
      return velocity;
    }
    return velocity.times(maxSpeed / speed);
  }

  private static double shortestAngleRad(double from, double to) {
    return MathUtil.angleModulus(to - from);
  }
}
