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

/**
 * Provides cell key functionality for the Repulsor package-internal data structures used by the
 * surrounding Repulsor subsystem. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class _CellKey {
  private final int ix;
  private final int iy;

  /**
   * Returns the cell key value maintained by this Repulsor component.
   *
   * @param ix distance or field-coordinate value in meters.
   * @param iy distance or field-coordinate value in meters.
   */
  public _CellKey(int ix, int iy) {
    this.ix = ix;
    this.iy = iy;
  }

  /**
   * Returns the hash code value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public int hashCode() {
    return (ix * 73856093) ^ (iy * 19349663);
  }

  /**
   * Returns the equals value maintained by this Repulsor component.
   *
   * @param o value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (!(o instanceof _CellKey)) return false;
    _CellKey k = (_CellKey) o;
    return ix == k.ix && iy == k.iy;
  }
}
