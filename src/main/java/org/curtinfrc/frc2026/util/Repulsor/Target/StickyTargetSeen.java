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
import java.util.Map;

/**
 * Provides sticky target seen functionality for the Repulsor sticky-target filtering and
 * target-selection layer. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
final class StickyTargetSeen<T> {
  private final Map<T, Double> lastSeenSec = new HashMap<>(16);
  private double timeoutSec = TargetConfig.DEFAULT_SEEN_TIMEOUT_SEC;

  /**
   * Updates set timeout sec state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param sec time value in seconds.
   */
  void setTimeoutSec(double sec) {
    if (!Double.isFinite(sec)) return;
    timeoutSec = Math.max(0.12, sec);
  }

  /**
   * Updates clear state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  void clear() {
    lastSeenSec.clear();
  }

  /**
   * Runs note seen in the Repulsor runtime.
   *
   * @param now value used by this operation.
   * @param value value used by this operation.
   */
  void noteSeen(double now, T value) {
    if (value == null) return;
    lastSeenSec.put(value, now);
  }

  /**
   * Runs note seen in the Repulsor runtime.
   *
   * @param now value used by this operation.
   * @param values value used by this operation.
   */
  void noteSeen(double now, Iterable<T> values) {
    if (values == null) return;
    for (T v : values) {
      if (v != null) lastSeenSec.put(v, now);
    }
  }

  /**
   * Returns the age value maintained by this Repulsor component.
   *
   * @param now value used by this operation.
   * @param t value used by this operation.
   * @return value produced by this operation.
   */
  double age(double now, T t) {
    if (t == null) return 1e9;
    Double s = lastSeenSec.get(t);
    if (s == null) return 1e9;
    return now - s;
  }

  /**
   * Returns the seen recently value maintained by this Repulsor component.
   *
   * @param now value used by this operation.
   * @param t value used by this operation.
   * @return value produced by this operation.
   */
  boolean seenRecently(double now, T t) {
    return age(now, t) <= Math.max(0.12, timeoutSec);
  }
}
