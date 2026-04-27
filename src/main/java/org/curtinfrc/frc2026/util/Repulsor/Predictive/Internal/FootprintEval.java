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

/**
 * Provides footprint eval functionality for the Repulsor package-internal data structures used by
 * the surrounding Repulsor subsystem. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class FootprintEval {
  /**
   * Configuration value for max count. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public int maxCount;

  /**
   * Configuration value for sum units. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double sumUnits;

  /**
   * Configuration value for avg evidence. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double avgEvidence;

  /**
   * Configuration value for has evidence. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public boolean hasEvidence;
}
