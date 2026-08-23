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

package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap;
import org.curtinfrc.frc2026.util.Repulsor.RepulsorSeason;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class DefaultFieldOverrideTest {
  @AfterEach
  void resetProvider() {
    RepulsorSeason.clearDefaultFieldProvider();
  }

  @Test
  void providerRegistrationRejectsNull() {
    assertThrows(
        IllegalArgumentException.class, () -> RepulsorSeason.setDefaultFieldProvider(null));
    RepulsorSeason.clearDefaultFieldProvider();
  }

  @Test
  void resolutionPrefersRegisteredProvider() {
    try {
      RepulsorSeason.setDefaultFieldProvider(TestField::new);
      FieldDefinition resolved = Constants.resolveDefaultField();
      assertInstanceOf(TestField.class, resolved);
      assertEquals(new FieldGeometry(12.5, 6.5), resolved.geometry());
      assertNotNull(resolved.geometry());
    } finally {
      RepulsorSeason.clearDefaultFieldProvider();
    }
  }

  @Test
  void nullProvidedValueFallsThroughToBuiltin() {
    try {
      RepulsorSeason.setDefaultFieldProvider(() -> null);
      FieldDefinition resolved = Constants.resolveDefaultField();
      assertInstanceOf(expectedBuiltinForProperty(), resolved);
    } finally {
      RepulsorSeason.clearDefaultFieldProvider();
    }
  }

  @Test
  void clearedProviderFallsThroughToBuiltin() {
    RepulsorSeason.setDefaultFieldProvider(TestField::new);
    RepulsorSeason.clearDefaultFieldProvider();
    assertInstanceOf(expectedBuiltinForProperty(), Constants.resolveDefaultField());
  }

  @Test
  void repulsorFieldPropertyStillSelectsSeasons() {
    String originalProperty = System.getProperty("repulsor.field");
    try {
      System.setProperty("repulsor.field", "reefscape2025");
      assertInstanceOf(Reefscape2025.class, Constants.resolveDefaultField());
      System.setProperty("repulsor.field", "rebuilt2026");
      assertInstanceOf(Rebuilt2026.class, Constants.resolveDefaultField());
      System.setProperty("repulsor.field", "unknown-season");
      assertInstanceOf(Rebuilt2026.class, Constants.resolveDefaultField());
    } finally {
      if (originalProperty == null) {
        System.clearProperty("repulsor.field");
      } else {
        System.setProperty("repulsor.field", originalProperty);
      }
    }
  }

  private static Class<?> expectedBuiltinForProperty() {
    String field = System.getProperty("repulsor.field", "rebuilt2026").trim().toLowerCase();
    return switch (field) {
      case "reefscape", "reefscape2025", "2025" -> Reefscape2025.class;
      default -> Rebuilt2026.class;
    };
  }

  private static final class TestField implements FieldDefinition {
    private final FieldGeometry geometry = new FieldGeometry(12.5, 6.5);

    @Override
    public GameElement[] build(FieldTrackerCore ft) {
      return new GameElement[0];
    }

    @Override
    public String gameName() {
      return "test";
    }

    @Override
    public int gameYear() {
      return 1970;
    }

    @Override
    public FieldGeometry geometry() {
      return geometry;
    }

    @Override
    public List<Obstacle> fieldObstacles() {
      return List.of();
    }

    @Override
    public List<Obstacle> walls() {
      return List.of();
    }

    @Override
    public Heatmap getHeatmap() {
      return null;
    }
  }
}
