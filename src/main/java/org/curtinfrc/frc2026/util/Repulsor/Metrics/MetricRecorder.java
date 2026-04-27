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

import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Provides metric recorder functionality for the Repulsor metric aggregation and NetworkTables
 * recording layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public abstract class MetricRecorder<T> {
  private final String name;
  private final MetricAggregator<T> aggregator;
  private final AtomicBoolean enabled = new AtomicBoolean(true);
  private final AtomicReference<T> last = new AtomicReference<>(null);
  private final AtomicLong count = new AtomicLong(0);

  protected MetricRecorder(String name, MetricAggregator<T> aggregator) {
    this.name = Objects.requireNonNull(name);
    this.aggregator = Objects.requireNonNull(aggregator);
  }

  /**
   * Updates record state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param data value used by this operation.
   */
  public final void record(T data) {
    if (!enabled.get()) return;
    last.set(data);
    aggregator.addSample(data);
    count.incrementAndGet();
    emit(data, aggregator.getOverall(), count.get(), enabled.get());
  }

  protected abstract void emit(T latest, T overall, long count, boolean enabled);

  /** Runs close in the Repulsor runtime. */
  public void close() {}

  /**
   * Returns the get name value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String getName() {
    return name;
  }

  /**
   * Updates set enabled state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param on value used by this operation.
   */
  public void setEnabled(boolean on) {
    enabled.set(on);
    emit(last.get(), aggregator.getOverall(), count.get(), enabled.get());
  }

  /**
   * Returns the is enabled value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isEnabled() {
    return enabled.get();
  }

  /**
   * Returns the get last record value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public T getLastRecord() {
    return last.get();
  }

  /**
   * Returns the get overall value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public T getOverall() {
    return aggregator.getOverall();
  }

  /**
   * Returns the get count value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public long getCount() {
    return count.get();
  }

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  public void reset() {
    last.set(null);
    aggregator.reset();
    count.set(0);
    emit(null, aggregator.getOverall(), 0, enabled.get());
  }
}
