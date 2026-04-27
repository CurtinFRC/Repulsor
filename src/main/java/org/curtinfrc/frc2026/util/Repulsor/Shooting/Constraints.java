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

/**
 * Provides constraints functionality for the Repulsor projectile and shot-planning layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public final class Constraints {

  /**
   * Defines the shot style values used by the Repulsor projectile and shot-planning layer. Use this
   * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public enum ShotStyle {
    ANY,
    DIRECT,
    ARC
  }

  private final double minLaunchSpeedMetersPerSecond;
  private final double maxLaunchSpeedMetersPerSecond;
  private final double minLaunchAngleDeg;
  private final double maxLaunchAngleDeg;
  private final ShotStyle shotStyle;

  /**
   * Returns the constraints value maintained by this Repulsor component.
   *
   * @param minLaunchSpeedMetersPerSecond distance or field-coordinate value in meters.
   * @param maxLaunchSpeedMetersPerSecond distance or field-coordinate value in meters.
   * @param minLaunchAngleDeg value used by this operation.
   * @param maxLaunchAngleDeg value used by this operation.
   */
  public Constraints(
      double minLaunchSpeedMetersPerSecond,
      double maxLaunchSpeedMetersPerSecond,
      double minLaunchAngleDeg,
      double maxLaunchAngleDeg) {
    this(
        minLaunchSpeedMetersPerSecond,
        maxLaunchSpeedMetersPerSecond,
        minLaunchAngleDeg,
        maxLaunchAngleDeg,
        ShotStyle.ANY);
  }

  /**
   * Returns the constraints value maintained by this Repulsor component.
   *
   * @param minLaunchSpeedMetersPerSecond distance or field-coordinate value in meters.
   * @param maxLaunchSpeedMetersPerSecond distance or field-coordinate value in meters.
   * @param minLaunchAngleDeg value used by this operation.
   * @param maxLaunchAngleDeg value used by this operation.
   * @param shotStyle value used by this operation.
   */
  public Constraints(
      double minLaunchSpeedMetersPerSecond,
      double maxLaunchSpeedMetersPerSecond,
      double minLaunchAngleDeg,
      double maxLaunchAngleDeg,
      ShotStyle shotStyle) {
    this.minLaunchSpeedMetersPerSecond = minLaunchSpeedMetersPerSecond;
    this.maxLaunchSpeedMetersPerSecond = maxLaunchSpeedMetersPerSecond;
    this.minLaunchAngleDeg = minLaunchAngleDeg;
    this.maxLaunchAngleDeg = maxLaunchAngleDeg;
    this.shotStyle = shotStyle == null ? ShotStyle.ANY : shotStyle;
  }

  /**
   * Returns the min launch speed meters per second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double minLaunchSpeedMetersPerSecond() {
    return minLaunchSpeedMetersPerSecond;
  }

  /**
   * Returns the max launch speed meters per second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double maxLaunchSpeedMetersPerSecond() {
    return maxLaunchSpeedMetersPerSecond;
  }

  /**
   * Returns the min launch angle deg value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double minLaunchAngleDeg() {
    return minLaunchAngleDeg;
  }

  /**
   * Returns the max launch angle deg value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double maxLaunchAngleDeg() {
    return maxLaunchAngleDeg;
  }

  /**
   * Returns the shot style value maintained by this Repulsor component.
   *
   * @return shot style result for shot style.
   */
  public ShotStyle shotStyle() {
    return shotStyle;
  }
}
