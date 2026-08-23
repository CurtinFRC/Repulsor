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

package org.curtinfrc.frc2026.util.Repulsor;

import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldDefinition;

/**
 * Registration point for the default season used when {@link Constants} first resolves its field.
 * This type deliberately holds no reference to {@link Constants}, so registering a provider here
 * never triggers early field resolution. Call {@link #setDefaultFieldProvider(Supplier)} before any
 * other Repulsor class touches the field model.
 */
public final class RepulsorSeason {
  private static volatile Supplier<FieldDefinition> provider;

  private RepulsorSeason() {}

  /**
   * Registers the provider producing the default field definition. Must be called before {@link
   * Constants} initializes; later registrations have no effect on the resolved field.
   *
   * @param supplier provider returning a non-null field definition.
   */
  public static void setDefaultFieldProvider(Supplier<FieldDefinition> supplier) {
    if (supplier == null) {
      throw new IllegalArgumentException("provider cannot be null");
    }
    provider = supplier;
  }

  /** Clears any registered provider. Intended for tests. */
  public static void clearDefaultFieldProvider() {
    provider = null;
  }

  static Supplier<FieldDefinition> peekProvider() {
    return provider;
  }
}
