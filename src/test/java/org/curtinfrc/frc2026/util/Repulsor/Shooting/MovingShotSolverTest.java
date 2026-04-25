package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class MovingShotSolverTest {
  private static final GamePiecePhysics PIECE =
      new GamePiecePhysics() {
        @Override
        public double massKg() {
          return 0.27;
        }

        @Override
        public double crossSectionAreaM2() {
          return 0.014;
        }

        @Override
        public double dragCoefficient() {
          return 0.95;
        }
      };

  private static final Constraints CONSTRAINTS = new Constraints(4.0, 30.0, 5.0, 85.0);

  @Test
  void stationaryMovingShotMatchesStaticShot() {
    Translation2d shooter = new Translation2d(1.0, 1.0);
    Translation2d target = new Translation2d(4.0, 1.0);

    ShotSolution stationary =
        DragShotPlanner.calculateStaticShotAngleAndSpeedLocal(
                PIECE, shooter, target, 1.4, 0.35, CONSTRAINTS)
            .orElseThrow();

    MovingShotSolver.Result moving =
        MovingShotSolver.solve(
                new MovingShotSolver.Request(
                    PIECE,
                    target,
                    1.4,
                    shooter,
                    Rotation2d.kZero,
                    new Translation2d(),
                    0.35,
                    CONSTRAINTS,
                    config(0.0, 0.18, 0.18, 0.18, 4.5, 4.5, 13.0, 0.20, 1),
                    0.18))
            .orElseThrow();

    assertEquals(
        stationary.launchSpeedMetersPerSecond(),
        moving.solution().launchSpeedMetersPerSecond(),
        1e-6);
    assertEquals(
        stationary.launchAngle().getDegrees(), moving.solution().launchAngle().getDegrees(), 1e-6);
    assertTrue(moving.readyToRelease());
  }

  @Test
  void compensatesTargetOppositeShooterVelocity() {
    MovingShotSolver.Result result =
        MovingShotSolver.solve(
                new MovingShotSolver.Request(
                    PIECE,
                    new Translation2d(4.0, 1.0),
                    1.4,
                    new Translation2d(1.0, 1.0),
                    Rotation2d.kZero,
                    new Translation2d(1.0, 0.0),
                    0.35,
                    CONSTRAINTS,
                    config(0.0, 0.20, 0.20, 0.20, 4.5, 4.5, 13.0, 0.20, 1),
                    0.20))
            .orElseThrow();

    assertEquals(3.8, result.compensatedTarget().getX(), 1e-9);
    assertEquals(1.0, result.compensatedTarget().getY(), 1e-9);
  }

  @Test
  void blocksReleaseWhenYawIsNotAligned() {
    MovingShotSolver.Result result =
        MovingShotSolver.solve(
                new MovingShotSolver.Request(
                    PIECE,
                    new Translation2d(4.0, 1.0),
                    1.4,
                    new Translation2d(1.0, 1.0),
                    Rotation2d.kCCW_90deg,
                    new Translation2d(),
                    0.35,
                    CONSTRAINTS,
                    config(0.0, 0.18, 0.18, 0.18, 4.5, 4.5, 5.0, 0.20, 1),
                    0.18))
            .orElseThrow();

    assertFalse(result.yawAligned());
    assertFalse(result.readyToRelease());
  }

  @Test
  void blocksReleaseWhenVelocityExceedsReleaseLimit() {
    MovingShotSolver.Result result =
        MovingShotSolver.solve(
                new MovingShotSolver.Request(
                    PIECE,
                    new Translation2d(4.0, 1.0),
                    1.4,
                    new Translation2d(1.0, 1.0),
                    Rotation2d.kZero,
                    new Translation2d(1.0, 0.0),
                    0.35,
                    CONSTRAINTS,
                    config(0.0, 0.18, 0.18, 0.18, 4.5, 0.5, 13.0, 0.20, 1),
                    0.18))
            .orElseThrow();

    assertFalse(result.releaseSpeedAllowed());
    assertFalse(result.readyToRelease());
  }

  @Test
  void velocitySweepProducesFiniteBoundedSolutions() {
    Translation2d target = new Translation2d(5.0, 1.0);
    Translation2d shooter = new Translation2d(1.0, 1.0);

    for (int i = 0; i <= 8; i++) {
      double vx = i * 0.25;
      MovingShotSolver.Result result =
          MovingShotSolver.solve(
                  new MovingShotSolver.Request(
                      PIECE,
                      target,
                      1.4,
                      shooter.plus(new Translation2d(i * 0.05, 0.0)),
                      Rotation2d.kZero,
                      new Translation2d(vx, 0.0),
                      0.35,
                      CONSTRAINTS,
                      config(0.08, 0.10, 0.45, 0.18, 4.5, 4.5, 20.0, 0.20, 3),
                      0.18))
              .orElseThrow();

      double speed = result.solution().launchSpeedMetersPerSecond();
      double angle = result.solution().launchAngle().getDegrees();
      assertTrue(Double.isFinite(speed));
      assertTrue(Double.isFinite(angle));
      assertTrue(speed >= CONSTRAINTS.minLaunchSpeedMetersPerSecond());
      assertTrue(speed <= CONSTRAINTS.maxLaunchSpeedMetersPerSecond());
      assertTrue(angle >= CONSTRAINTS.minLaunchAngleDeg());
      assertTrue(angle <= CONSTRAINTS.maxLaunchAngleDeg());
      assertTrue(result.solution().timeToPlaneSeconds() > 0.0);
    }
  }

  private static MovingShotSolver.Config config(
      double releaseLatencySeconds,
      double minFlightPredictionSeconds,
      double maxFlightPredictionSeconds,
      double defaultFlightPredictionSeconds,
      double maxCompensatedSpeedMetersPerSecond,
      double maxReleaseSpeedMetersPerSecond,
      double yawToleranceDegrees,
      double maxVerticalErrorMeters,
      int iterations) {
    return new MovingShotSolver.Config(
        releaseLatencySeconds,
        minFlightPredictionSeconds,
        maxFlightPredictionSeconds,
        defaultFlightPredictionSeconds,
        maxCompensatedSpeedMetersPerSecond,
        maxReleaseSpeedMetersPerSecond,
        yawToleranceDegrees,
        maxVerticalErrorMeters,
        iterations);
  }
}
