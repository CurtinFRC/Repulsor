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

/**
 * Provides double mean aggregator functionality for the Repulsor metric aggregation and
 * NetworkTables recording layer. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class DoubleMeanAggregator implements MetricAggregator<Double> {
  private double sum = 0.0;
  private long n = 0;

  /**
   * Runs add sample in the Repulsor runtime.
   *
   * @param value value used by this operation.
   */
  @Override
  public void addSample(Double value) {
    if (value == null) return;
    sum += value;
    n += 1;
  }

  /**
   * Returns the get overall value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public Double getOverall() {
    return n == 0 ? null : (sum / n);
  }

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  @Override
  public void reset() {
    sum = 0.0;
    n = 0;
  }
}
