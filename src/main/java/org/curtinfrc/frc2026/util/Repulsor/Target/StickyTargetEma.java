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

package org.curtinfrc.frc2026.util.Repulsor.Target;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * Provides sticky target ema functionality for the Repulsor sticky-target filtering and
 * target-selection layer. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
final class StickyTargetEma<T> {
  private static final double DEFAULT_TAU_SEC = 0.26;

  private final Map<T, EmaEntry> emaScore = new HashMap<>(8);
  private double tauSec = DEFAULT_TAU_SEC;
  private double evictSec = 3.5;

  /**
   * Updates set evict sec state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param sec time value in seconds.
   */
  void setEvictSec(double sec) {
    if (!Double.isFinite(sec)) return;
    evictSec = Math.max(0.25, sec);
  }

  /**
   * Updates clear state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  void clear() {
    emaScore.clear();
  }

  /**
   * Runs evict in the Repulsor runtime.
   *
   * @param now value used by this operation.
   */
  void evict(double now) {
    if (emaScore.isEmpty()) return;
    double ttl = Math.max(0.25, evictSec);

    Iterator<Map.Entry<T, EmaEntry>> it = emaScore.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<T, EmaEntry> e = it.next();
      EmaEntry v = e.getValue();
      if (v == null || (now - v.t) > ttl) it.remove();
    }
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param now value used by this operation.
   * @param key distance or field-coordinate value in meters.
   * @param raw value used by this operation.
   * @param dt value used by this operation.
   * @return value produced by this operation.
   */
  double update(double now, T key, double raw, double dt) {
    if (key == null) return raw;

    EmaEntry prevE = emaScore.get(key);
    double prev = prevE != null ? prevE.v : raw;

    double tau = Math.max(1e-3, tauSec);
    double a = 1.0 - Math.exp(-dt / tau);
    double v = prev + (raw - prev) * a;

    emaScore.put(key, new EmaEntry(v, now));
    return v;
  }

  private static final class EmaEntry {
    /**
     * Configuration value for v. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final double v;

    /**
     * Configuration value for t. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final double t;

    EmaEntry(double v, double t) {
      this.v = v;
      this.t = t;
    }
  }
}
