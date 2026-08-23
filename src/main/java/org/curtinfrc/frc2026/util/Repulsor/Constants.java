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

package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldDefinition;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldModel;

/**
 * Provides constants functionality for the Repulsor core Repulsor coordination layer. Use this type
 * from robot code, field profiles, or tests when integrating the corresponding Repulsor subsystem.
 * Coordinates are field-relative unless a method documents robot-relative motion.
 */
public final class Constants {
  private static volatile Supplier<FieldDefinition> defaultFieldProvider;
  private static volatile boolean fieldResolved;

  public static final FieldDefinition FIELD = loadDefaultField();
  public static final FieldModel FIELD_MODEL = FIELD.fieldModel();
  public static final AprilTagFieldLayout aprilTagLayout = FIELD_MODEL.aprilTagLayout();
  public static final FieldGeometry FIELD_GEOMETRY = FIELD_MODEL.geometry();
  public static final double FIELD_LENGTH = FIELD_GEOMETRY.lengthMeters();
  public static final double FIELD_WIDTH = FIELD_GEOMETRY.widthMeters();

  static {
    fieldResolved = true;
  }

  public static void setDefaultFieldProvider(Supplier<FieldDefinition> provider) {
    if (provider == null) {
      throw new IllegalArgumentException("provider cannot be null");
    }
    if (fieldResolved) {
      throw new IllegalStateException(
          "Default field already resolved; register a provider before first Constants access");
    }
    defaultFieldProvider = provider;
  }

  private static FieldDefinition loadDefaultField() {
    Supplier<FieldDefinition> provider = defaultFieldProvider;
    if (provider != null) {
      FieldDefinition provided = provider.get();
      if (provided != null) {
        return provided;
      }
    }
    String field = System.getProperty("repulsor.field", "rebuilt2026").trim().toLowerCase();
    return switch (field) {
      case "reefscape", "reefscape2025", "2025" ->
          builtinField("org.curtinfrc.frc2026.util.Repulsor.Fields.Reefscape2025");
      case "rebuilt", "rebuilt2026", "2026" ->
          builtinField("org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026");
      default -> builtinField("org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026");
    };
  }

  private static FieldDefinition builtinField(String className) {
    try {
      return (FieldDefinition) Class.forName(className).getDeclaredConstructor().newInstance();
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("Unable to instantiate built-in field " + className, e);
    }
  }

  private Constants() {}
}
