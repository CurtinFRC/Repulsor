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

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Provides hpstation metrics functionality for the Repulsor metric aggregation and NetworkTables
 * recording layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class HPStationMetrics {
  public static final String DEFAULT_TOPIC_PREFIX = "hp";

  public static final String DEFAULT_PICKUP_TIME_LABEL = "pickupTimeSeconds";

  private static final Map<String, MetricRecorder<Double>> byKey = new ConcurrentHashMap<>();
  private static final AtomicReference<Labels> labels = new AtomicReference<>(Labels.defaults());

  /**
   * Topic vocabulary used when composing recorder names. Defaults preserve the historical
   * {@code hp/<station>/pickupTimeSeconds} layout; another season can install different labels
   * through {@link #configure(Labels)} without code changes at call sites.
   *
   * @param topicPrefix root segment of composed recorder names, defaulting to {@code hp}
   * @param pickupTimeLabel leaf segment for pickup-time recorders, defaulting to {@code
   *     pickupTimeSeconds}
   */
  public record Labels(String topicPrefix, String pickupTimeLabel) {
    public Labels {
      topicPrefix = normalized(topicPrefix, DEFAULT_TOPIC_PREFIX);
      pickupTimeLabel = normalized(pickupTimeLabel, DEFAULT_PICKUP_TIME_LABEL);
    }

    public static Labels defaults() {
      return new Labels(DEFAULT_TOPIC_PREFIX, DEFAULT_PICKUP_TIME_LABEL);
    }

    private static String normalized(String value, String fallback) {
      return value == null || value.isBlank() ? fallback : value.trim();
    }
  }

  public static void configure(Labels custom) {
    labels.set(custom == null ? Labels.defaults() : custom);
  }

  public static void resetToDefaults() {
    labels.set(Labels.defaults());
  }

  public static Labels labels() {
    return labels.get();
  }

  /**
   * Updates recorder state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param stationKey distance or field-coordinate value in meters.
   * @return metric recorder of double result for recorder.
   */
  public static MetricRecorder<Double> recorder(String stationKey) {
    Labels current = labels.get();
    String topic =
        current.topicPrefix() + "/" + stationKey + "/" + current.pickupTimeLabel();
    return byKey.computeIfAbsent(topic, t -> new DoubleMeanNTRecorder(t));
  }

  private HPStationMetrics() {}
}
