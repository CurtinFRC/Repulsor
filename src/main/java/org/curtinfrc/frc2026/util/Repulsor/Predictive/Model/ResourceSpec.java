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

/**
 * Provides resource spec functionality for the Repulsor typed model layer for field objects and
 * prediction inputs. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class ResourceSpec {
  /**
   * Configuration value for radius m. Distances use meters in WPILib field coordinates and should
   * be treated as tunable when sourced from profiles.
   */
  public final double radiusM;

  /**
   * Configuration value for unit value. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double unitValue;

  /**
   * Configuration value for sigma m. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double sigmaM;

  /**
   * Returns the resource spec value maintained by this Repulsor component.
   *
   * @param radiusM value used by this operation.
   * @param unitValue value used by this operation.
   * @param sigmaM value used by this operation.
   */
  public ResourceSpec(double radiusM, double unitValue, double sigmaM) {
    this.radiusM = Math.max(0.01, radiusM);
    this.unitValue = unitValue;
    this.sigmaM = Math.max(0.02, sigmaM);
  }
}
