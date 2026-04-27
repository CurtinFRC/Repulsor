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

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Provides spatial index functionality for the Repulsor package-internal data structures used by
 * the surrounding Repulsor subsystem. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class _SpatialIndex {
  private final double cell;
  private final HashMap<_CellKey, ArrayList<Translation2d>> map = new HashMap<>();

  /**
   * Returns the spatial index value maintained by this Repulsor component.
   *
   * @param cell value used by this operation.
   */
  public _SpatialIndex(double cell) {
    this.cell = Math.max(1e-6, cell);
  }

  /**
   * Runs add in the Repulsor runtime.
   *
   * @param p value used by this operation.
   */
  public void add(Translation2d p) {
    int ix = (int) Math.floor(p.getX() / cell);
    int iy = (int) Math.floor(p.getY() / cell);
    _CellKey k = new _CellKey(ix, iy);
    map.computeIfAbsent(k, kk -> new ArrayList<>()).add(p);
  }

  /**
   * Returns the get cell value maintained by this Repulsor component.
   *
   * @param ix distance or field-coordinate value in meters.
   * @param iy distance or field-coordinate value in meters.
   * @return array list of translation2d result for get cell.
   */
  public ArrayList<Translation2d> getCell(int ix, int iy) {
    return map.get(new _CellKey(ix, iy));
  }

  /**
   * Returns the ix value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public int ix(double x) {
    return (int) Math.floor(x / cell);
  }

  /**
   * Returns the iy value maintained by this Repulsor component.
   *
   * @param y distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public int iy(double y) {
    return (int) Math.floor(y / cell);
  }
}
