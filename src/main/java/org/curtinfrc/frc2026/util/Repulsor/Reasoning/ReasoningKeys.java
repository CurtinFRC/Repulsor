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

package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

/**
 * Provides reasoning keys functionality for the Repulsor rule-based strategy and signal reasoning
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public final class ReasoningKeys {
  private ReasoningKeys() {}

  public static final SignalKey<Boolean> ENABLED = new SignalKey<>("enabled", Boolean.class);
  public static final SignalKey<Boolean> TELEOP = new SignalKey<>("teleop", Boolean.class);
  public static final SignalKey<Boolean> AUTO = new SignalKey<>("auto", Boolean.class);
  public static final SignalKey<Boolean> ENDGAME = new SignalKey<>("endgame", Boolean.class);

  /**
   * Returns the bool key value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @return signal key of boolean result for bool key.
   */
  public static SignalKey<Boolean> boolKey(String name) {
    return new SignalKey<>(name, Boolean.class);
  }

  /**
   * Returns the double key value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @return signal key of double result for double key.
   */
  public static SignalKey<Double> doubleKey(String name) {
    return new SignalKey<>(name, Double.class);
  }

  /**
   * Returns the int key value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @return signal key of integer result for int key.
   */
  public static SignalKey<Integer> intKey(String name) {
    return new SignalKey<>(name, Integer.class);
  }

  /**
   * Returns the long key value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @return signal key of long result for long key.
   */
  public static SignalKey<Long> longKey(String name) {
    return new SignalKey<>(name, Long.class);
  }

  /**
   * Returns the string key value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @return signal key of string result for string key.
   */
  public static SignalKey<String> stringKey(String name) {
    return new SignalKey<>(name, String.class);
  }
}
