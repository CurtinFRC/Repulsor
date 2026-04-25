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
}
