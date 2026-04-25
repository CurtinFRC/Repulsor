package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class FieldProfileYamlLoaderTest {
  @TempDir Path tempDir;

  @Test
  void loadsYamlProfileOverridesWithoutTouchingConstants() throws Exception {
    Path profile = tempDir.resolve("custom.yaml");
    Files.writeString(
        profile,
        """
        id: custom
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources:
          cube:
            radiusMeters: 0.2
            unitValue: 2.0
            sigmaMeters: 0.7
        projectileShots: {}
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      FieldProfileConfig cfg =
          FieldProfileYamlLoader.loadOrDefault("custom", new FieldProfileConfig());

      assertEquals("CUSTOM", cfg.gameName);
      assertEquals(2099, cfg.gameYear);
      assertEquals(12.5, cfg.geometry.lengthMeters, 1e-9);
      assertEquals(6.25, cfg.geometry.widthMeters, 1e-9);
      assertTrue(cfg.resources.containsKey("cube"));
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void rejectsInvalidYamlProfileValues() throws Exception {
    Path profile = tempDir.resolve("invalid.yaml");
    Files.writeString(
        profile,
        """
        id: invalid
        gameName: INVALID
        gameYear: 2099
        geometry:
          lengthMeters: -1.0
          widthMeters: 6.25
        resources: {}
        projectileShots: {}
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      assertThrows(
          IllegalArgumentException.class,
          () -> FieldProfileYamlLoader.loadOrDefault("invalid", new FieldProfileConfig()));
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void loadsMovingShotTuningFromYaml() throws Exception {
    Path profile = tempDir.resolve("moving-shot.yaml");
    Files.writeString(
        profile,
        """
        id: moving-shot
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources: {}
        projectileShots:
          pass:
            enabled: true
            role: TRANSFER_TO_SCORE
            gamePieceId: ball
            target:
              kind: alliance_side
              blueXMeters: 3.0
              blueYMeters: 2.0
            targetHeightMeters: 1.0
            constraints:
              minLaunchSpeedMetersPerSecond: 3.0
              maxLaunchSpeedMetersPerSecond: 22.0
              minLaunchAngleDegrees: 10.0
              maxLaunchAngleDegrees: 55.0
              shotStyle: DIRECT
            routeLevel: pass
            routeMechanismSetpoint: NET
            behindTargetMeters: 1.0
            lateralOffsetsMeters: [0.0]
            fieldMarginMeters: 0.25
            movingShot:
              enabled: true
              releaseLatencySeconds: 0.12
              minFlightPredictionSeconds: 0.20
              maxFlightPredictionSeconds: 0.70
              defaultFlightPredictionSeconds: 0.30
              maxCompensatedSpeedMetersPerSecond: 5.0
              maxReleaseSpeedMetersPerSecond: 4.0
              yawToleranceDegrees: 9.0
              maxVerticalErrorMeters: 0.15
              iterations: 4
            fallbackGamePiece:
              massKg: 0.27
              crossSectionAreaM2: 0.014
              dragCoefficient: 0.95
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      FieldProfileConfig cfg =
          FieldProfileYamlLoader.loadOrDefault("moving-shot", new FieldProfileConfig());

      FieldProfileConfig.MovingShotConfig moving = cfg.projectileShots.get("pass").movingShot;
      assertTrue(moving.enabled);
      assertEquals(0.12, moving.releaseLatencySeconds, 1e-9);
      assertEquals(0.70, moving.maxFlightPredictionSeconds, 1e-9);
      assertEquals(4, moving.iterations);
      assertEquals(9.0, moving.solverConfig().yawToleranceDegrees(), 1e-9);
      FieldProfileConfig.ShotConstraintsConfig constraints =
          cfg.projectileShots.get("pass").constraints;
      assertEquals(3.0, constraints.minLaunchSpeedMetersPerSecond, 1e-9);
      assertEquals(22.0, constraints.maxLaunchSpeedMetersPerSecond, 1e-9);
      assertEquals(10.0, constraints.minLaunchAngleDegrees, 1e-9);
      assertEquals(55.0, constraints.maxLaunchAngleDegrees, 1e-9);
      assertEquals("DIRECT", constraints.shotStyle);
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }
}
