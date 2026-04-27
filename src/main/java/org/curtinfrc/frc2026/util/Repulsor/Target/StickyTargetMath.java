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

import java.util.Objects;
import java.util.function.ToDoubleBiFunction;

/**
 * Provides sticky target math functionality for the Repulsor sticky-target filtering and
 * target-selection layer. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
final class StickyTargetMath {
  private StickyTargetMath() {}

  /**
   * Returns the clamp dt value maintained by this Repulsor component.
   *
   * @param dt value used by this operation.
   * @return value produced by this operation.
   */
  static double clampDt(double dt) {
    if (dt < 1e-3) return 1e-3;
    if (dt > 0.10) return 0.10;
    return dt;
  }

  /**
   * Returns the safe score value maintained by this Repulsor component.
   *
   * @param v value used by this operation.
   * @return value produced by this operation.
   */
  static double safeScore(double v) {
    if (!Double.isFinite(v)) return 0.0;
    if (v > 1e9) return 1e9;
    if (v < -1e9) return -1e9;
    return v;
  }

  /**
   * Returns the eq value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param distanceFn value used by this operation.
   * @param sameEps value used by this operation.
   * @return value produced by this operation.
   */
  static <T> boolean eq(T a, T b, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {
    if (a == null || b == null) return false;
    if (Objects.equals(a, b)) return true;
    if (distanceFn == null) return false;
    double d = distanceFn.applyAsDouble(a, b);
    if (!Double.isFinite(d) || d < 0.0) return false;
    return d <= Math.max(0.0, sameEps);
  }

  /**
   * Returns the eq n value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param distanceFn value used by this operation.
   * @param sameEps value used by this operation.
   * @return value produced by this operation.
   */
  static <T> boolean eqN(T a, T b, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {
    if (a == null && b == null) return true;
    if (a == null || b == null) return false;
    return eq(a, b, distanceFn, sameEps);
  }

  /**
   * Returns the dist value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param distanceFn value used by this operation.
   * @return value produced by this operation.
   */
  static <T> double dist(T a, T b, ToDoubleBiFunction<T, T> distanceFn) {
    if (a == null || b == null || distanceFn == null) return 0.0;
    double d = distanceFn.applyAsDouble(a, b);
    if (!Double.isFinite(d) || d < 0.0) return 0.0;
    return d;
  }

  /**
   * Returns the adaptive eps value maintained by this Repulsor component.
   *
   * @param sameEps value used by this operation.
   * @param flickerBoost value used by this operation.
   * @param motionBoost value used by this operation.
   * @return value produced by this operation.
   */
  static double adaptiveEps(double sameEps, double flickerBoost, double motionBoost) {
    double e = Math.max(0.0, sameEps);
    double baseFloor = 0.08;
    if (e < baseFloor) e = baseFloor;
    e *= (1.0 + 0.85 * flickerBoost);
    e *= (1.0 + 0.65 * motionBoost);
    if (e < baseFloor) e = baseFloor;
    return e;
  }
}
