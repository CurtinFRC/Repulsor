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

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;

/**
 * Tunable collection model used by the predictive field state. A field profile can install one of
 * these profiles to describe how long resource observations stay valid, how evidence decays, and
 * which field regions should be ignored by generic collection logic.
 *
 * <p>The profile deliberately uses neutral resource language. A 2026 profile may configure the
 * resource type as fuel, while another game can use pipes, notes, cones, or any future object type
 * without changing the prediction/runtime algorithms.
 *
 * @param defaultResourceType fallback type to collect when callers do not provide a type set
 * @param defaultResourceSpec default scoring/evidence model for the fallback resource type
 * @param observationHardMaxAgeSeconds maximum accepted resource observation age in seconds
 * @param observationAgeDecay exponential age-decay coefficient applied to resource evidence
 * @param fieldGeometry field dimensions used by filters, wall costs, and alliance-zone helpers
 * @param excludedRegions field-relative regions that should not be treated as collectable
 */
public record ResourceCollectionProfile(
    String defaultResourceType,
    ResourceSpec defaultResourceSpec,
    double observationHardMaxAgeSeconds,
    double observationAgeDecay,
    FieldGeometry fieldGeometry,
    List<Predicate<Translation2d>> excludedRegions) {
  public ResourceCollectionProfile {
    defaultResourceType =
        defaultResourceType == null || defaultResourceType.isBlank()
            ? "resource"
            : defaultResourceType.trim().toLowerCase();
    defaultResourceSpec =
        defaultResourceSpec == null ? new ResourceSpec(0.10, 1.0, 0.06) : defaultResourceSpec;
    observationHardMaxAgeSeconds = finitePositive(observationHardMaxAgeSeconds, 0.95);
    observationAgeDecay = finiteNonNegative(observationAgeDecay, 0.75);
    if (fieldGeometry == null) {
      throw new IllegalArgumentException("fieldGeometry cannot be null");
    }
    excludedRegions = excludedRegions == null ? List.of() : List.copyOf(excludedRegions);
  }

  /**
   * Creates a profile compatible with the existing 2026 fuel predictor defaults.
   *
   * @param geometry field geometry used for bounds checks
   * @return collection profile using fuel as the default resource type
   */
  public static ResourceCollectionProfile fuel2026(FieldGeometry geometry) {
    return new ResourceCollectionProfile(
        "fuel", new ResourceSpec(0.10, 1.0, 0.06), 0.95, 0.75, geometry, List.of());
  }

  /**
   * Returns whether a field point can be used as a collection-resource observation.
   *
   * @param point field-relative point in meters
   * @return true when the point is inside the field and outside excluded profile regions
   */
  public boolean accepts(Translation2d point) {
    if (!fieldGeometry.contains(point)) return false;
    for (Predicate<Translation2d> region : excludedRegions) {
      if (region != null && region.test(point)) return false;
    }
    return true;
  }

  private static double finitePositive(double value, double fallback) {
    return Double.isFinite(value) && value > 0.0 ? value : fallback;
  }

  private static double finiteNonNegative(double value, double fallback) {
    return Double.isFinite(value) && value >= 0.0 ? value : fallback;
  }
}
