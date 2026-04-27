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
 * Provides collect probe functionality for the Repulsor typed model layer for field objects and
 * prediction inputs. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class CollectProbe {
  /**
   * Configuration value for count. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final int count;

  /**
   * Configuration value for units. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double units;

  /**
   * Returns the collect probe value maintained by this Repulsor component.
   *
   * @param count value used by this operation.
   * @param units value used by this operation.
   */
  public CollectProbe(int count, double units) {
    this.count = Math.max(0, count);
    this.units = Math.max(0.0, units);
  }
}
