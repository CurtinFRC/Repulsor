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

/**
 * Provides sticky target hard lock functionality for the Repulsor sticky-target filtering and
 * target-selection layer. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
final class StickyTargetHardLock {
  private double untilSec = -1e9;

  /**
   * Updates clear state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  void clear() {
    untilSec = -1e9;
  }

  /**
   * Returns the is locked value maintained by this Repulsor component.
   *
   * @param now value used by this operation.
   * @return value produced by this operation.
   */
  boolean isLocked(double now) {
    return now < untilSec;
  }

  /**
   * Runs extend to at least in the Repulsor runtime.
   *
   * @param newUntilSec time value in seconds.
   */
  void extendToAtLeast(double newUntilSec) {
    if (!Double.isFinite(newUntilSec)) return;
    untilSec = Math.max(untilSec, newUntilSec);
  }

  /**
   * Runs arm in the Repulsor runtime.
   *
   * @param now value used by this operation.
   * @param switchDistM value used by this operation.
   */
  void arm(double now, double switchDistM) {
    double d = Double.isFinite(switchDistM) ? Math.max(0.0, switchDistM) : 0.0;

    double lock = TargetConfig.BASE_HARD_LOCK_SEC + TargetConfig.HARD_LOCK_PER_M_SEC * d;
    if (lock > TargetConfig.HARD_LOCK_MAX_SEC) lock = TargetConfig.HARD_LOCK_MAX_SEC;
    if (lock < TargetConfig.BASE_HARD_LOCK_SEC) lock = TargetConfig.BASE_HARD_LOCK_SEC;

    untilSec = now + lock;
  }
}
