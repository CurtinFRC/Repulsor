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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides resource regions functionality for the Repulsor package-internal data structures used by
 * the surrounding Repulsor subsystem. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class ResourceRegions {
  /**
   * Configuration value for centers. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Translation2d[] centers;

  /**
   * Configuration value for mass. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double[] mass;

  /**
   * Returns the resource regions value maintained by this Repulsor component.
   *
   * @param centers value used by this operation.
   * @param mass value used by this operation.
   */
  public ResourceRegions(Translation2d[] centers, double[] mass) {
    this.centers = centers != null ? centers : new Translation2d[0];
    this.mass = mass != null ? mass : new double[this.centers.length];
  }
}
