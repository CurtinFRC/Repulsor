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
 * Provides game piece physics functionality for the Repulsor projectile and shot-planning layer.
 * Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public abstract class GamePiecePhysics {
  /**
   * Returns the mass kg value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract double massKg();

  /**
   * Returns the cross section area m2 value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract double crossSectionAreaM2();

  /**
   * Returns the drag coefficient value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract double dragCoefficient();

  /**
   * Returns the air density kg per m3 value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double airDensityKgPerM3() {
    return 1.225;
  }
}
