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

package org.curtinfrc.frc2026.util.Repulsor.Predictive.Model;

import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;

/**
 * Profile for selecting a recovery point for resources that have been transferred back toward an
 * alliance side. The same model covers 2026 fuel shuttling and future games where a resource is
 * passed, staged, dropped, or otherwise moved for later scoring.
 *
 * @param resourceType dynamic-object type to recover
 * @param resourceSpec evidence model for the resource type
 * @param fieldGeometry field dimensions used for alliance flipping and zone generation
 * @param allianceZoneXMaxFraction fraction of field length included in the blue-side recovery zone
 * @param zoneEdgeMarginMeters margin from walls in meters for generated candidate points
 * @param gridStepMeters spacing between generated recovery candidates in meters
 * @param collectLimit maximum ranked candidates evaluated by the predictive collector
 */
public record ResourceRecoveryProfile(
    String resourceType,
    ResourceSpec resourceSpec,
    FieldGeometry fieldGeometry,
    double allianceZoneXMaxFraction,
    double zoneEdgeMarginMeters,
    double gridStepMeters,
    int collectLimit) {
  public ResourceRecoveryProfile {
    resourceType =
        resourceType == null || resourceType.isBlank()
            ? "resource"
            : resourceType.trim().toLowerCase();
    resourceSpec = resourceSpec == null ? new ResourceSpec(0.075, 1.0, 0.95) : resourceSpec;
    if (fieldGeometry == null) {
      throw new IllegalArgumentException("fieldGeometry cannot be null");
    }
    allianceZoneXMaxFraction = clamp(allianceZoneXMaxFraction, 0.01, 1.0);
    zoneEdgeMarginMeters = finiteNonNegative(zoneEdgeMarginMeters, 0.35);
    gridStepMeters = finitePositive(gridStepMeters, 0.45);
    collectLimit = Math.max(1, collectLimit);
  }

  /**
   * Creates a profile compatible with the original 2026 fuel shuttle recovery API.
   *
   * @param geometry field dimensions for the active game
   * @return recovery profile using fuel as the transferred resource
   */
  public static ResourceRecoveryProfile fuel2026(FieldGeometry geometry) {
    return new ResourceRecoveryProfile(
        "fuel", new ResourceSpec(0.075, 1.0, 0.95), geometry, 0.42, 0.35, 0.45, 96);
  }

  private static double finitePositive(double value, double fallback) {
    return Double.isFinite(value) && value > 0.0 ? value : fallback;
  }

  private static double finiteNonNegative(double value, double fallback) {
    return Double.isFinite(value) && value >= 0.0 ? value : fallback;
  }

  private static double clamp(double value, double min, double max) {
    if (!Double.isFinite(value)) return min;
    return Math.max(min, Math.min(max, value));
  }
}
