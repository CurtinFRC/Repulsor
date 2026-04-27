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

import edu.wpi.first.apriltag.AprilTagFieldLayout;

/**
 * Immutable data record for field model values passed through the Repulsor field/profile definition
 * layer used to tune Repulsor for a specific game. Use this type from robot code, field profiles,
 * or tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 *
 * @param geometry component of the field model model
 * @param aprilTagLayout record component for the field model snapshot
 */
public record FieldModel(FieldGeometry geometry, AprilTagFieldLayout aprilTagLayout) {
  public FieldModel {
    if (geometry == null) {
      throw new IllegalArgumentException("geometry cannot be null");
    }
  }
}
