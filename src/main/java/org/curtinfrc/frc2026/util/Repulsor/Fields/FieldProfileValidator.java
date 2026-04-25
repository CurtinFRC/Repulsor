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

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;

public final class FieldProfileValidator {
  private FieldProfileValidator() {}

  public static void requireValid(FieldProfileConfig cfg, String source) {
    List<String> errors = validate(cfg);
    if (!errors.isEmpty()) {
      throw new IllegalArgumentException(
          "Invalid Repulsor profile " + source + ": " + String.join("; ", errors));
    }
  }

  public static List<String> validate(FieldProfileConfig cfg) {
    List<String> errors = new ArrayList<>();
    if (cfg == null) {
      errors.add("config is null");
      return errors;
    }

    if (cfg.id == null || cfg.id.isBlank()) errors.add("id must be non-empty");
    if (cfg.gameName == null || cfg.gameName.isBlank()) errors.add("gameName must be non-empty");
    if (cfg.gameYear == null || cfg.gameYear <= 0) errors.add("gameYear must be positive");

    if (cfg.geometry == null) {
      errors.add("geometry is required");
    } else {
      requirePositive(cfg.geometry.lengthMeters, "geometry.lengthMeters", errors);
      requirePositive(cfg.geometry.widthMeters, "geometry.widthMeters", errors);
    }

    if (cfg.resources != null) {
      for (Map.Entry<String, FieldProfileConfig.ResourceConfig> entry : cfg.resources.entrySet()) {
        String prefix = "resources." + entry.getKey();
        FieldProfileConfig.ResourceConfig resource = entry.getValue();
        if (entry.getKey() == null || entry.getKey().isBlank()) {
          errors.add("resource key must be non-empty");
        }
        if (resource == null) {
          errors.add(prefix + " is null");
          continue;
        }
        requirePositive(resource.radiusMeters, prefix + ".radiusMeters", errors);
        requirePositive(resource.unitValue, prefix + ".unitValue", errors);
        requirePositive(resource.sigmaMeters, prefix + ".sigmaMeters", errors);
      }
    }

    if (cfg.projectileShots != null) {
      for (Map.Entry<String, FieldProfileConfig.ProjectileShotConfig> entry :
          cfg.projectileShots.entrySet()) {
        validateProjectileShot(entry.getKey(), entry.getValue(), errors);
      }
    }

    if (cfg.shuttleShot != null && Boolean.TRUE.equals(cfg.shuttleShot.enabled)) {
      validateProjectileShot("legacyShuttleShot", cfg.shuttleShot, errors);
    }

    return errors;
  }

  public static List<String> validate(FieldDefinition field) {
    List<String> errors = new ArrayList<>();
    if (field == null) {
      errors.add("field is null");
      return errors;
    }

    FieldGeometry geometry = field.geometry();
    for (Obstacle obstacle : field.fieldObstacles()) {
      if (obstacle == null) errors.add(field.gameName() + " has null field obstacle");
    }
    for (Obstacle wall : field.walls()) {
      if (wall == null) errors.add(field.gameName() + " has null wall obstacle");
    }
    if (field.defaultCollectSetpoint().isPresent()
        && !geometry.contains(
            field.defaultCollectSetpoint().get().getBlue(SetpointContext.EMPTY).getTranslation())) {
      errors.add("default collect setpoint is outside field geometry");
    }
    if (field.defaultScoreSetpoint().isPresent()
        && !geometry.contains(
            field.defaultScoreSetpoint().get().getBlue(SetpointContext.EMPTY).getTranslation())) {
      errors.add("default score setpoint is outside field geometry");
    }
    return errors;
  }

  private static void requirePositive(Double value, String name, List<String> errors) {
    if (value == null || !Double.isFinite(value) || value <= 0.0) {
      errors.add(name + " must be finite and > 0");
    }
  }

  private static void requireNonNegative(Double value, String name, List<String> errors) {
    if (value == null || !Double.isFinite(value) || value < 0.0) {
      errors.add(name + " must be finite and >= 0");
    }
  }

  private static void validateProjectileShot(
      String id, FieldProfileConfig.ProjectileShotConfig shot, List<String> errors) {
    String prefix = "projectileShots." + id;
    if (id == null || id.isBlank()) errors.add("projectile shot id must be non-empty");
    if (shot == null) {
      errors.add(prefix + " is null");
      return;
    }
    if (!Boolean.TRUE.equals(shot.enabled)) return;
    requirePositive(shot.targetHeightMeters, prefix + ".targetHeightMeters", errors);
    requireNonNegative(shot.behindTargetMeters, prefix + ".behindTargetMeters", errors);
    requireNonNegative(shot.fieldMarginMeters, prefix + ".fieldMarginMeters", errors);
    if (shot.lateralOffsetsMeters == null || shot.lateralOffsetsMeters.length == 0) {
      errors.add(prefix + ".lateralOffsetsMeters must have at least one value");
    }
    if (shot.fallbackGamePiece == null) {
      errors.add(prefix + ".fallbackGamePiece is required");
    } else {
      requirePositive(shot.fallbackGamePiece.massKg, prefix + ".fallbackGamePiece.massKg", errors);
      requirePositive(
          shot.fallbackGamePiece.crossSectionAreaM2,
          prefix + ".fallbackGamePiece.crossSectionAreaM2",
          errors);
      requirePositive(
          shot.fallbackGamePiece.dragCoefficient,
          prefix + ".fallbackGamePiece.dragCoefficient",
          errors);
    }
  }
}
