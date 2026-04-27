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

import java.util.Objects;

/**
 * Provides signal key functionality for the Repulsor rule-based strategy and signal reasoning
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public final class SignalKey<T> {
  private final String name;
  private final Class<T> type;

  /**
   * Returns the signal key value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @param type value used by this operation.
   */
  public SignalKey(String name, Class<T> type) {
    this.name = Objects.requireNonNull(name);
    this.type = Objects.requireNonNull(type);
  }

  /**
   * Returns the name value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String name() {
    return name;
  }

  /**
   * Returns the type value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Class<T> type() {
    return type;
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
    if (!(o instanceof SignalKey<?> k)) return false;
    return name.equals(k.name) && type.equals(k.type);
  }

  /**
   * Returns the hash code value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public int hashCode() {
    return Objects.hash(name, type);
  }

  /**
   * Returns the to string value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public String toString() {
    return "SignalKey(" + name + ":" + type.getSimpleName() + ")";
  }
}
