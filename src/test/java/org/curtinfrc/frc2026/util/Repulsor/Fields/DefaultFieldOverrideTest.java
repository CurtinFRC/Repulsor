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

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.junit.jupiter.api.Test;

class DefaultFieldOverrideTest {
  @Test
  void providerRejectedAfterConstantsResolution() {
    assertNotNull(Constants.FIELD);
    assertThrows(
        IllegalStateException.class, () -> Constants.setDefaultFieldProvider(TestField::new));
  }

  @Test
  void loadDefaultFieldPrefersRegisteredProvider() throws Exception {
    Object originalProvider = staticValue("defaultFieldProvider");
    boolean originalResolved = (boolean) staticValue("fieldResolved");
    try {
      setStatic("defaultFieldProvider", (Supplier<FieldDefinition>) TestField::new);
      setStatic("fieldResolved", false);
      FieldDefinition resolved = invokeLoadDefaultField();
      assertInstanceOf(TestField.class, resolved);
      assertEquals(new FieldGeometry(12.5, 6.5), resolved.geometry());
      assertEquals(
          new FieldGeometry(12.5, 6.5),
          new FieldModel(resolved.geometry(), resolved.aprilTagLayout()).geometry());
    } finally {
      setStatic("defaultFieldProvider", originalProvider);
      setStatic("fieldResolved", originalResolved);
    }
  }

  @Test
  void nullProviderFallsThroughToBuiltin() throws Exception {
    Object originalProvider = staticValue("defaultFieldProvider");
    boolean originalResolved = (boolean) staticValue("fieldResolved");
    try {
      setStatic("defaultFieldProvider", (Supplier<FieldDefinition>) () -> null);
      setStatic("fieldResolved", false);
      FieldDefinition resolved = invokeLoadDefaultField();
      assertInstanceOf(expectedBuiltinForProperty(), resolved);
    } finally {
      setStatic("defaultFieldProvider", originalProvider);
      setStatic("fieldResolved", originalResolved);
    }
  }

  @Test
  void repulsorFieldPropertyStillSelectsSeasons() throws Exception {
    String originalProperty = System.getProperty("repulsor.field");
    Object originalProvider = staticValue("defaultFieldProvider");
    boolean originalResolved = (boolean) staticValue("fieldResolved");
    try {
      setStatic("defaultFieldProvider", null);
      setStatic("fieldResolved", false);
      System.setProperty("repulsor.field", "reefscape2025");
      assertInstanceOf(Reefscape2025.class, invokeLoadDefaultField());
      System.setProperty("repulsor.field", "rebuilt2026");
      assertInstanceOf(Rebuilt2026.class, invokeLoadDefaultField());
      System.setProperty("repulsor.field", "unknown-season");
      assertInstanceOf(Rebuilt2026.class, invokeLoadDefaultField());
    } finally {
      if (originalProperty == null) {
        System.clearProperty("repulsor.field");
      } else {
        System.setProperty("repulsor.field", originalProperty);
      }
      setStatic("defaultFieldProvider", originalProvider);
      setStatic("fieldResolved", originalResolved);
    }
  }

  private static Class<?> expectedBuiltinForProperty() {
    String field = System.getProperty("repulsor.field", "rebuilt2026").trim().toLowerCase();
    return switch (field) {
      case "reefscape", "reefscape2025", "2025" -> Reefscape2025.class;
      default -> Rebuilt2026.class;
    };
  }

  private static FieldDefinition invokeLoadDefaultField() throws Exception {
    Method method = Constants.class.getDeclaredMethod("loadDefaultField");
    method.setAccessible(true);
    return (FieldDefinition) method.invoke(null);
  }

  private static Object staticValue(String name) throws Exception {
    Field field = Constants.class.getDeclaredField(name);
    field.setAccessible(true);
    return field.get(null);
  }

  private static void setStatic(String name, Object value) throws Exception {
    Field field = Constants.class.getDeclaredField(name);
    field.setAccessible(true);
    field.set(null, value);
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
