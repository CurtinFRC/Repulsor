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
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldDefinition;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldModel;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Reefscape2025;

public final class Constants {
  public static final FieldDefinition FIELD = loadDefaultField();
  public static final FieldModel FIELD_MODEL = FIELD.fieldModel();
  public static final AprilTagFieldLayout aprilTagLayout = FIELD_MODEL.aprilTagLayout();
  public static final FieldGeometry FIELD_GEOMETRY = FIELD_MODEL.geometry();
  public static final double FIELD_LENGTH = FIELD_GEOMETRY.lengthMeters();
  public static final double FIELD_WIDTH = FIELD_GEOMETRY.widthMeters();

  private static FieldDefinition loadDefaultField() {
    String field = System.getProperty("repulsor.field", "rebuilt2026").trim().toLowerCase();
    return switch (field) {
      case "reefscape", "reefscape2025", "2025" -> new Reefscape2025();
      case "rebuilt", "rebuilt2026", "2026" -> new Rebuilt2026();
      default -> new Rebuilt2026();
    };
  }

  private Constants() {}
}
