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

package org.curtinfrc.frc2026.util.Repulsor.Metrics;

import com.google.gson.Gson;
import java.lang.reflect.Type;
import java.util.Optional;

/**
 * Provides metric codecs functionality for the Repulsor metric aggregation and NetworkTables
 * recording layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class MetricCodecs {
  private MetricCodecs() {}

  /**
   * Provides gson codec functionality for the Repulsor metric aggregation and NetworkTables
   * recording layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static final class GsonCodec<T> implements MetricCodec<T> {
    private final Gson gson = new Gson();
    private final Type type;

    /**
     * Returns the gson codec value maintained by this Repulsor component.
     *
     * @param type value used by this operation.
     */
    public GsonCodec(Type type) {
      this.type = type;
    }

    /**
     * Returns the encode value maintained by this Repulsor component.
     *
     * @param value value used by this operation.
     * @return value produced by this operation.
     */
    @Override
    public String encode(T value) {
      return value == null ? "" : gson.toJson(value, type);
    }

    /**
     * Returns the decode value maintained by this Repulsor component.
     *
     * @param raw value used by this operation.
     * @return value produced by this operation.
     */
    @Override
    public Optional<T> decode(String raw) {
      if (raw == null || raw.isEmpty()) return Optional.empty();
      return Optional.ofNullable(gson.fromJson(raw, type));
    }
  }

  /**
   * Provides string codec functionality for the Repulsor metric aggregation and NetworkTables
   * recording layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static final class StringCodec implements MetricCodec<String> {
    /**
     * Returns the encode value maintained by this Repulsor component.
     *
     * @param value value used by this operation.
     * @return value produced by this operation.
     */
    @Override
    public String encode(String value) {
      return value == null ? "" : value;
    }
  }

  /**
   * Provides double codec functionality for the Repulsor metric aggregation and NetworkTables
   * recording layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static final class DoubleCodec implements MetricCodec<Double> {
    /**
     * Returns the encode value maintained by this Repulsor component.
     *
     * @param value value used by this operation.
     * @return value produced by this operation.
     */
    @Override
    public String encode(Double value) {
      return value == null ? "" : Double.toString(value);
    }
  }
}
