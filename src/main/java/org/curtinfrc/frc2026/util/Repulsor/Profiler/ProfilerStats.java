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

package org.curtinfrc.frc2026.util.Repulsor.Profiler;

import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.LongAdder;

/**
 * Provides profiler stats functionality for the Repulsor low-overhead profiling and event recording
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
final class ProfilerStats {
  /**
   * Configuration value for name. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final String name;

  final LongAdder wCount = new LongAdder();
  final LongAdder wTotalNs = new LongAdder();
  final AtomicLong wMinNs = new AtomicLong(Long.MAX_VALUE);
  final AtomicLong wMaxNs = new AtomicLong(Long.MIN_VALUE);

  /**
   * Creates a profiler stats instance with the dependencies and tuning values used by this Repulsor
   * component.
   *
   * @param name value used by this operation.
   */
  ProfilerStats(String name) {
    this.name = name;
  }

  /**
   * Updates record state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param durNs value used by this operation.
   */
  void record(long durNs) {
    if (durNs <= 0) return;
    wCount.increment();
    wTotalNs.add(durNs);
    updateMin(wMinNs, durNs);
    updateMax(wMaxNs, durNs);
  }

  /**
   * Returns the snapshot and reset window value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  Snapshot snapshotAndResetWindow() {
    long c = wCount.sum();
    long t = wTotalNs.sum();
    long mn = wMinNs.get();
    long mx = wMaxNs.get();
    if (mn == Long.MAX_VALUE) mn = 0L;
    if (mx == Long.MIN_VALUE) mx = 0L;

    wCount.reset();
    wTotalNs.reset();
    wMinNs.set(Long.MAX_VALUE);
    wMaxNs.set(Long.MIN_VALUE);

    return new Snapshot(name, c, t, mn, mx);
  }

  private static void updateMin(AtomicLong a, long v) {
    long cur = a.get();
    while (v < cur) {
      if (a.compareAndSet(cur, v)) return;
      cur = a.get();
    }
  }

  private static void updateMax(AtomicLong a, long v) {
    long cur = a.get();
    while (v > cur) {
      if (a.compareAndSet(cur, v)) return;
      cur = a.get();
    }
  }

  /**
   * Provides snapshot functionality for the Repulsor low-overhead profiling and event recording
   * layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  static final class Snapshot {
    /**
     * Configuration value for name. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final String name;

    /**
     * Configuration value for count. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final long count;

    /**
     * Configuration value for total ns. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final long totalNs;

    /**
     * Configuration value for min ns. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final long minNs;

    /**
     * Configuration value for max ns. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final long maxNs;

    Snapshot(String name, long count, long totalNs, long minNs, long maxNs) {
      this.name = name;
      this.count = count;
      this.totalNs = totalNs;
      this.minNs = minNs;
      this.maxNs = maxNs;
    }
  }
}
