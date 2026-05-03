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
package org.curtinfrc.frc2026.util.Repulsor.Tracking;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Internal.ObjectiveCache;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElementModel;

/**
 * Provides field tracker objective cache functionality for the Repulsor field-object tracking and
 * collection objective layer. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
final class FieldTrackerObjectiveCache {
  private FieldTrackerObjectiveCache() {}

  /**
   * Returns the mix hash value maintained by this Repulsor component.
   *
   * @param h value used by this operation.
   * @param x distance or field-coordinate value in meters.
   * @param y distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  static int mixHash(int h, double x, double y) {
    long a = Double.doubleToLongBits(x);
    long b = Double.doubleToLongBits(y);
    h ^= (int) (a ^ (a >>> 32));
    h = (h * 16777619) ^ (int) (b ^ (b >>> 32));
    return h;
  }

  /**
   * Runs rebuild objective cache for category in the Repulsor runtime.
   *
   * @param cache value used by this operation.
   * @param cat value used by this operation.
   * @param fieldMap value used by this operation.
   */
  static void rebuildObjectiveCacheForCategory(
      ObjectiveCache cache, CategorySpec cat, GameElement[] fieldMap) {
    GameElement[] fm = fieldMap;
    if (fm == null || fm.length == 0) {
      cache.clear();
      return;
    }
    ArrayList<Translation2d> pts = new ArrayList<>(256);
    int h = 146959810;
    for (GameElement e : fm) {
      if (e == null) continue;
      if (e.getCategory() != cat) continue;
      GameElementModel m = e.getModel();
      if (m == null) continue;
      Pose3d p = m.getPosition();
      if (p == null) continue;
      double x = p.getX();
      double y = p.getY();
      pts.add(new Translation2d(x, y));
      h = mixHash(h, x, y);
    }
    Translation2d[] arr = pts.toArray(new Translation2d[0]);
    cache.update(arr, h);
  }
}
