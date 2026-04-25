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

package org.curtinfrc.frc2026.util.Repulsor.Strategy;

import edu.wpi.first.math.geometry.Translation2d;

public record ResourceRegionSummary(
    String id,
    double resourceUnits,
    double resourceValue,
    Translation2d nearestResource,
    double nearestDistanceMeters,
    double trafficRisk,
    double obstacleRisk) {
  public ResourceRegionSummary {
    id = id == null || id.isBlank() ? "region" : id.trim();
    resourceUnits = finiteNonNegative(resourceUnits);
    resourceValue = finiteNonNegative(resourceValue);
    nearestDistanceMeters =
        nearestResource == null
            ? Double.POSITIVE_INFINITY
            : finiteNonNegative(nearestDistanceMeters);
    trafficRisk = finiteNonNegative(trafficRisk);
    obstacleRisk = finiteNonNegative(obstacleRisk);
  }

  public static ResourceRegionSummary empty(String id) {
    return new ResourceRegionSummary(id, 0.0, 0.0, null, Double.POSITIVE_INFINITY, 0.0, 0.0);
  }

  public boolean hasResources() {
    return resourceUnits > 1e-9 && resourceValue > 1e-9 && nearestResource != null;
  }

  private static double finiteNonNegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
