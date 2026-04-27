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

import edu.wpi.first.networktables.*;
import java.util.Objects;

/**
 * Provides network tables metric recorder functionality for the Repulsor metric aggregation and
 * NetworkTables recording layer. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class NetworkTablesMetricRecorder<T> extends MetricRecorder<T> {
  private final MetricCodec<T> codec;
  private final NetworkTable table;
  private final BooleanPublisher enabledPub;
  private final IntegerPublisher countPub;
  private final StringPublisher lastPub;
  private final StringPublisher overallPub;

  /**
   * Returns the network tables metric recorder value maintained by this Repulsor component.
   *
   * @param metricName value used by this operation.
   * @param aggregator value used by this operation.
   * @param codec value used by this operation.
   */
  public NetworkTablesMetricRecorder(
      String metricName, MetricAggregator<T> aggregator, MetricCodec<T> codec) {
    super(metricName, aggregator);
    this.codec = Objects.requireNonNull(codec);
    NetworkTableInstance nti = NetworkTableInstance.getDefault();
    this.table = nti.getTable("metrics").getSubTable(metricName);
    this.enabledPub = table.getBooleanTopic("enabled").publish();
    this.countPub = table.getIntegerTopic("count").publish();
    this.lastPub = table.getStringTopic("last").publish();
    this.overallPub = table.getStringTopic("overall").publish();
    emit(getLastRecord(), getOverall(), getCount(), isEnabled());
  }

  @Override
  protected void emit(T latest, T overall, long count, boolean enabled) {
    enabledPub.set(enabled);
    countPub.set(count);
    lastPub.set(codec.encode(latest));
    overallPub.set(codec.encode(overall));
  }

  /** Runs close in the Repulsor runtime. */
  @Override
  public void close() {
    enabledPub.close();
    countPub.close();
    lastPub.close();
    overallPub.close();
  }
}
