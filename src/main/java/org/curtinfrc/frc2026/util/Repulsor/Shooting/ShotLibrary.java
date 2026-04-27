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
import java.util.List;

/**
 * Provides shot library functionality for the Repulsor projectile and shot-planning layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public final class ShotLibrary {
  private final Translation2d targetFieldPosition;
  private final double targetHeightMeters;
  private final double shooterReleaseHeightMeters;
  private final double robotHalfLengthMeters;
  private final double robotHalfWidthMeters;
  private final Constraints constraints;
  private final List<ShotLibraryEntry> entries;
  private final boolean complete;

  /**
   * Returns the shot library value maintained by this Repulsor component.
   *
   * @param targetFieldPosition value used by this operation.
   * @param targetHeightMeters distance or field-coordinate value in meters.
   * @param shooterReleaseHeightMeters distance or field-coordinate value in meters.
   * @param robotHalfLengthMeters distance or field-coordinate value in meters.
   * @param robotHalfWidthMeters distance or field-coordinate value in meters.
   * @param constraints value used by this operation.
   * @param entries value used by this operation.
   * @param complete value used by this operation.
   */
  public ShotLibrary(
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      Constraints constraints,
      List<ShotLibraryEntry> entries,
      boolean complete) {
    this.targetFieldPosition = targetFieldPosition;
    this.targetHeightMeters = targetHeightMeters;
    this.shooterReleaseHeightMeters = shooterReleaseHeightMeters;
    this.robotHalfLengthMeters = robotHalfLengthMeters;
    this.robotHalfWidthMeters = robotHalfWidthMeters;
    this.constraints = constraints;
    this.entries = entries;
    this.complete = complete;
  }

  /**
   * Returns the target field position value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Translation2d targetFieldPosition() {
    return targetFieldPosition;
  }

  /**
   * Returns the target height meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double targetHeightMeters() {
    return targetHeightMeters;
  }

  /**
   * Returns the shooter release height meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double shooterReleaseHeightMeters() {
    return shooterReleaseHeightMeters;
  }

  /**
   * Returns the robot half length meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double robotHalfLengthMeters() {
    return robotHalfLengthMeters;
  }

  /**
   * Returns the robot half width meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double robotHalfWidthMeters() {
    return robotHalfWidthMeters;
  }

  /**
   * Returns the constraints value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Constraints constraints() {
    return constraints;
  }

  /**
   * Returns the entries value maintained by this Repulsor component.
   *
   * @return list of shot library entry values produced by this operation.
   */
  public List<ShotLibraryEntry> entries() {
    return entries;
  }

  /**
   * Returns the complete value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean complete() {
    return complete;
  }
}
