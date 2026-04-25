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

import java.util.LinkedHashMap;
import java.util.Map;

public class FieldProfileConfig {
  public String id;
  public String gameName;
  public Integer gameYear;
  public GeometryConfig geometry = new GeometryConfig();
  public Map<String, ResourceConfig> resources = new LinkedHashMap<>();
  public Map<String, ProjectileShotConfig> projectileShots = new LinkedHashMap<>();

  /** Compatibility input for older 2026-specific YAML. Prefer projectileShots. */
  @Deprecated(forRemoval = false)
  public ShuttleShotConfig shuttleShot = new ShuttleShotConfig();

  public RebuiltCorridorConfig rebuiltCorridor = new RebuiltCorridorConfig();

  public FieldGeometry fieldGeometry(double fallbackLengthMeters, double fallbackWidthMeters) {
    return new FieldGeometry(
        finitePositive(geometry.lengthMeters, fallbackLengthMeters),
        finitePositive(geometry.widthMeters, fallbackWidthMeters));
  }

  public static FieldProfileConfig merge(FieldProfileConfig base, FieldProfileConfig overlay) {
    if (base == null) return overlay;
    if (overlay == null) return base;

    if (overlay.id != null) base.id = overlay.id;
    if (overlay.gameName != null) base.gameName = overlay.gameName;
    if (overlay.gameYear != null) base.gameYear = overlay.gameYear;

    if (overlay.geometry != null) {
      if (overlay.geometry.lengthMeters != null)
        base.geometry.lengthMeters = overlay.geometry.lengthMeters;
      if (overlay.geometry.widthMeters != null)
        base.geometry.widthMeters = overlay.geometry.widthMeters;
    }

    if (overlay.resources != null && !overlay.resources.isEmpty()) {
      overlay.resources.forEach(
          (type, resource) -> {
            ResourceConfig target =
                base.resources.computeIfAbsent(type, ignored -> new ResourceConfig());
            if (resource.radiusMeters != null) target.radiusMeters = resource.radiusMeters;
            if (resource.unitValue != null) target.unitValue = resource.unitValue;
            if (resource.sigmaMeters != null) target.sigmaMeters = resource.sigmaMeters;
          });
    }

    if (overlay.projectileShots != null && !overlay.projectileShots.isEmpty()) {
      overlay.projectileShots.forEach(
          (id, shot) -> {
            ProjectileShotConfig target =
                base.projectileShots.computeIfAbsent(id, ignored -> new ProjectileShotConfig());
            mergeProjectileShot(target, shot);
          });
    }

    mergeProjectileShot(base.shuttleShot, overlay.shuttleShot);
    mergeCorridor(base.rebuiltCorridor, overlay.rebuiltCorridor);
    return base;
  }

  private static void mergeProjectileShot(ProjectileShotConfig base, ProjectileShotConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.enabled != null) base.enabled = overlay.enabled;
    if (overlay.role != null) base.role = overlay.role;
    if (overlay.gamePieceId != null) base.gamePieceId = overlay.gamePieceId;
    if (overlay.targetHeightMeters != null) base.targetHeightMeters = overlay.targetHeightMeters;
    if (overlay.routeLevel != null) base.routeLevel = overlay.routeLevel;
    if (overlay.routeMechanismSetpoint != null)
      base.routeMechanismSetpoint = overlay.routeMechanismSetpoint;
    if (overlay.behindTargetMeters != null) base.behindTargetMeters = overlay.behindTargetMeters;
    if (overlay.lateralOffsetsMeters != null)
      base.lateralOffsetsMeters = overlay.lateralOffsetsMeters;
    if (overlay.fieldMarginMeters != null) base.fieldMarginMeters = overlay.fieldMarginMeters;
    if (overlay.fallbackGamePiece != null) {
      if (overlay.fallbackGamePiece.massKg != null) {
        base.fallbackGamePiece.massKg = overlay.fallbackGamePiece.massKg;
      }
      if (overlay.fallbackGamePiece.crossSectionAreaM2 != null) {
        base.fallbackGamePiece.crossSectionAreaM2 = overlay.fallbackGamePiece.crossSectionAreaM2;
      }
      if (overlay.fallbackGamePiece.dragCoefficient != null) {
        base.fallbackGamePiece.dragCoefficient = overlay.fallbackGamePiece.dragCoefficient;
      }
    }
  }

  private static void mergeCorridor(RebuiltCorridorConfig base, RebuiltCorridorConfig overlay) {
    if (base == null || overlay == null) return;
    if (overlay.rectWidthMeters != null) base.rectWidthMeters = overlay.rectWidthMeters;
    if (overlay.rectHeightMeters != null) base.rectHeightMeters = overlay.rectHeightMeters;
    if (overlay.rectOffsetFromCenterMeters != null) {
      base.rectOffsetFromCenterMeters = overlay.rectOffsetFromCenterMeters;
    }
    if (overlay.edgeOffsetMeters != null) base.edgeOffsetMeters = overlay.edgeOffsetMeters;
    if (overlay.rectStrength != null) base.rectStrength = overlay.rectStrength;
    if (overlay.rectRangeXMeters != null) base.rectRangeXMeters = overlay.rectRangeXMeters;
    if (overlay.rectRangeYMeters != null) base.rectRangeYMeters = overlay.rectRangeYMeters;
    if (overlay.biasStrength != null) base.biasStrength = overlay.biasStrength;
    if (overlay.biasRangeMeters != null) base.biasRangeMeters = overlay.biasRangeMeters;
    if (overlay.bypassStrengthScale != null) base.bypassStrengthScale = overlay.bypassStrengthScale;
    if (overlay.bypassRangeMeters != null) base.bypassRangeMeters = overlay.bypassRangeMeters;
    if (overlay.sidePullDxMeters != null) base.sidePullDxMeters = overlay.sidePullDxMeters;
    if (overlay.sideBiasStrengthScale != null) {
      base.sideBiasStrengthScale = overlay.sideBiasStrengthScale;
    }
    if (overlay.sideBiasRangeScale != null) base.sideBiasRangeScale = overlay.sideBiasRangeScale;
    if (overlay.sideBypassStrengthScale != null) {
      base.sideBypassStrengthScale = overlay.sideBypassStrengthScale;
    }
    if (overlay.sideBypassRangeScale != null) {
      base.sideBypassRangeScale = overlay.sideBypassRangeScale;
    }
    if (overlay.railXWindowMeters != null) base.railXWindowMeters = overlay.railXWindowMeters;
    if (overlay.railMinHalfWidthMeters != null) {
      base.railMinHalfWidthMeters = overlay.railMinHalfWidthMeters;
    }
    if (overlay.railHalfWidthGapScale != null) {
      base.railHalfWidthGapScale = overlay.railHalfWidthGapScale;
    }
    if (overlay.railStrength != null) base.railStrength = overlay.railStrength;
    if (overlay.railMaxForce != null) base.railMaxForce = overlay.railMaxForce;
    if (overlay.centerRailMinWindowMeters != null) {
      base.centerRailMinWindowMeters = overlay.centerRailMinWindowMeters;
    }
    if (overlay.centerRailWindowScale != null) {
      base.centerRailWindowScale = overlay.centerRailWindowScale;
    }
    if (overlay.outerRailXOffsetScale != null) {
      base.outerRailXOffsetScale = overlay.outerRailXOffsetScale;
    }
    if (overlay.outerRailWindowMeters != null) {
      base.outerRailWindowMeters = overlay.outerRailWindowMeters;
    }
  }

  static double finitePositive(Double value, double fallback) {
    return value != null && Double.isFinite(value) && value > 0.0 ? value : fallback;
  }

  static int positive(Integer value, int fallback) {
    return value != null && value > 0 ? value : fallback;
  }

  public static class GeometryConfig {
    public Double lengthMeters;
    public Double widthMeters;
  }

  public static class ResourceConfig {
    public Double radiusMeters;
    public Double unitValue;
    public Double sigmaMeters;
  }

  public static class ProjectileShotConfig {
    public Boolean enabled;
    public String role;
    public String gamePieceId;
    public Double targetHeightMeters;
    public String routeLevel;
    public String routeMechanismSetpoint;
    public Double behindTargetMeters;
    public double[] lateralOffsetsMeters;
    public Double fieldMarginMeters;
    public GamePiecePhysicsConfig fallbackGamePiece = new GamePiecePhysicsConfig();
  }

  /** Compatibility config for older profile YAML. Prefer projectileShots. */
  public static class ShuttleShotConfig extends ProjectileShotConfig {}

  public static class GamePiecePhysicsConfig {
    public Double massKg;
    public Double crossSectionAreaM2;
    public Double dragCoefficient;
  }

  public static class RebuiltCorridorConfig {
    public Double rectWidthMeters;
    public Double rectHeightMeters;
    public Double rectOffsetFromCenterMeters;
    public Double edgeOffsetMeters;
    public Double rectStrength;
    public Double rectRangeXMeters;
    public Double rectRangeYMeters;
    public Double biasStrength;
    public Double biasRangeMeters;
    public Double bypassStrengthScale;
    public Double bypassRangeMeters;
    public Double sidePullDxMeters;
    public Double sideBiasStrengthScale;
    public Double sideBiasRangeScale;
    public Double sideBypassStrengthScale;
    public Double sideBypassRangeScale;
    public Double railXWindowMeters;
    public Double railMinHalfWidthMeters;
    public Double railHalfWidthGapScale;
    public Double railStrength;
    public Double railMaxForce;
    public Double centerRailMinWindowMeters;
    public Double centerRailWindowScale;
    public Double outerRailXOffsetScale;
    public Double outerRailWindowMeters;
  }
}
