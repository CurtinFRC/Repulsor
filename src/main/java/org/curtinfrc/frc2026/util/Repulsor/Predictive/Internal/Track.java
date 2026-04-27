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
 * Provides track functionality for the Repulsor package-internal data structures used by the
 * surrounding Repulsor subsystem. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class Track {
  /**
   * Configuration value for pos. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public Translation2d pos;

  /**
   * Configuration value for vel. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public Translation2d vel;

  /**
   * Configuration value for speed cap. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double speedCap;

  /**
   * Configuration value for last ts. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double lastTs;

  /**
   * Returns the track value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param v value used by this operation.
   * @param cap value used by this operation.
   * @param t value used by this operation.
   */
  public Track(Translation2d p, Translation2d v, double cap, double t) {
    pos = p != null ? p : new Translation2d();
    vel = v != null ? v : new Translation2d();
    speedCap = cap;
    lastTs = t;
  }
}
