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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides heading pick functionality for the Repulsor package-internal data structures used by the
 * surrounding Repulsor subsystem. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class HeadingPick {
  /**
   * Configuration value for center. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Translation2d center;

  /**
   * Configuration value for heading. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Rotation2d heading;

  /**
   * Configuration value for eval. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final FootprintEval eval;

  /**
   * Returns the heading pick value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param heading value used by this operation.
   * @param eval value used by this operation.
   */
  public HeadingPick(Translation2d center, Rotation2d heading, FootprintEval eval) {
    this.center = center;
    this.heading = heading;
    this.eval = eval;
  }
}
