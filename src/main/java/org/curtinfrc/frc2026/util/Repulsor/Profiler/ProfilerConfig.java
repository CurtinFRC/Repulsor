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

/**
 * Provides profiler config functionality for the Repulsor low-overhead profiling and event
 * recording layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class ProfilerConfig {
  /**
   * Configuration value for enabled. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final boolean enabled;

  /**
   * Configuration value for write file. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final boolean writeFile;

  /**
   * Configuration value for gzip file. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final boolean gzipFile;

  /**
   * Configuration value for summary period ms. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  public final int summaryPeriodMs;

  /**
   * Configuration value for queue capacity. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public final int queueCapacity;

  /**
   * Configuration value for top sections. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  public final int topSections;

  /**
   * Configuration value for top counters. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public final int topCounters;

  /**
   * Configuration value for top gauges. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final int topGauges;

  /**
   * Returns the profiler config value maintained by this Repulsor component.
   *
   * @param enabled value used by this operation.
   * @param writeFile value used by this operation.
   * @param gzipFile value used by this operation.
   * @param summaryPeriodMs value used by this operation.
   * @param queueCapacity distance or field-coordinate value in meters.
   * @param topSections value used by this operation.
   * @param topCounters value used by this operation.
   * @param topGauges value used by this operation.
   */
  public ProfilerConfig(
      boolean enabled,
      boolean writeFile,
      boolean gzipFile,
      int summaryPeriodMs,
      int queueCapacity,
      int topSections,
      int topCounters,
      int topGauges) {
    this.enabled = enabled;
    this.writeFile = writeFile;
    this.gzipFile = gzipFile;
    this.summaryPeriodMs = Math.max(250, summaryPeriodMs);
    this.queueCapacity = Math.max(1024, queueCapacity);
    this.topSections = clamp(topSections, 10, 2000);
    this.topCounters = clamp(topCounters, 0, 2000);
    this.topGauges = clamp(topGauges, 0, 2000);
  }

  /**
   * Returns the from system properties value maintained by this Repulsor component.
   *
   * @param defaultEnabled value used by this operation.
   * @return profiler config result for from system properties.
   */
  public static ProfilerConfig fromSystemProperties(boolean defaultEnabled) {
    boolean enabled = boolProp("repulsor.profiler.enabled", defaultEnabled);
    boolean writeFile = boolProp("repulsor.profiler.file", true);
    boolean gzipFile = boolProp("repulsor.profiler.gzip", false);
    int summaryMs = intProp("repulsor.profiler.summaryMs", 1500);
    int queueCap = intProp("repulsor.profiler.queueCapacity", 32768);
    int topSections = intProp("repulsor.profiler.topSections", 300);
    int topCounters = intProp("repulsor.profiler.topCounters", 120);
    int topGauges = intProp("repulsor.profiler.topGauges", 120);
    return new ProfilerConfig(
        enabled, writeFile, gzipFile, summaryMs, queueCap, topSections, topCounters, topGauges);
  }

  private static int clamp(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  private static boolean boolProp(String k, boolean def) {
    String v = System.getProperty(k);
    if (v == null) return def;
    v = v.trim().toLowerCase();
    if (v.equals("1") || v.equals("true") || v.equals("yes") || v.equals("y") || v.equals("on"))
      return true;
    if (v.equals("0") || v.equals("false") || v.equals("no") || v.equals("n") || v.equals("off"))
      return false;
    return def;
  }

  private static int intProp(String k, int def) {
    String v = System.getProperty(k);
    if (v == null) return def;
    try {
      return Integer.parseInt(v.trim());
    } catch (Exception e) {
      return def;
    }
  }
}
