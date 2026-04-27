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

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides shot library entry functionality for the Repulsor projectile and shot-planning layer.
 * Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public final class ShotLibraryEntry {
  private final Translation2d shooterPosition;
  private final double shooterYawRad;
  private final double launchSpeedMetersPerSecond;
  private final double launchAngleRad;
  private final double timeToPlaneSeconds;
  private final double verticalErrorMeters;

  /**
   * Returns the shot library entry value maintained by this Repulsor component.
   *
   * @param shooterPosition value used by this operation.
   * @param shooterYawRad value used by this operation.
   * @param launchSpeedMetersPerSecond distance or field-coordinate value in meters.
   * @param launchAngleRad value used by this operation.
   * @param timeToPlaneSeconds time value in seconds.
   * @param verticalErrorMeters distance or field-coordinate value in meters.
   */
  public ShotLibraryEntry(
      Translation2d shooterPosition,
      double shooterYawRad,
      double launchSpeedMetersPerSecond,
      double launchAngleRad,
      double timeToPlaneSeconds,
      double verticalErrorMeters) {
    this.shooterPosition = shooterPosition;
    this.shooterYawRad = shooterYawRad;
    this.launchSpeedMetersPerSecond = launchSpeedMetersPerSecond;
    this.launchAngleRad = launchAngleRad;
    this.timeToPlaneSeconds = timeToPlaneSeconds;
    this.verticalErrorMeters = verticalErrorMeters;
  }

  /**
   * Returns the shooter position value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Translation2d shooterPosition() {
    return shooterPosition;
  }

  /**
   * Returns the shooter yaw rad value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double shooterYawRad() {
    return shooterYawRad;
  }

  /**
   * Returns the launch speed meters per second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double launchSpeedMetersPerSecond() {
    return launchSpeedMetersPerSecond;
  }

  /**
   * Returns the launch angle rad value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double launchAngleRad() {
    return launchAngleRad;
  }

  /**
   * Returns the time to plane seconds value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double timeToPlaneSeconds() {
    return timeToPlaneSeconds;
  }

  /**
   * Returns the vertical error meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double verticalErrorMeters() {
    return verticalErrorMeters;
  }
}
