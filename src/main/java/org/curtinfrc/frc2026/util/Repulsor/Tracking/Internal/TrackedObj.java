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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Internal;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Provides tracked obj functionality for the Repulsor package-internal data structures used by the
 * surrounding Repulsor subsystem. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class TrackedObj {
  /**
   * Configuration value for id. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final String id;

  /**
   * Configuration value for type. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public volatile String type;

  /**
   * Configuration value for pos. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public volatile Pose3d pos;

  /**
   * Configuration value for prev. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public volatile Pose3d prev;

  /**
   * Configuration value for t ns. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public volatile long tNs;

  /**
   * Configuration value for vx. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public volatile double vx;

  /**
   * Configuration value for vy. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public volatile double vy;

  /**
   * Configuration value for vz. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public volatile double vz;

  /**
   * Returns the tracked obj value maintained by this Repulsor component.
   *
   * @param id value used by this operation.
   */
  public TrackedObj(String id) {
    this.id = id;
    this.type = "unknown";
    this.pos = null;
    this.prev = null;
    this.tNs = 0L;
    this.vx = 0.0;
    this.vy = 0.0;
    this.vz = 0.0;
  }
}
