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

package org.curtinfrc.frc2026.util.Repulsor.Tuning;

/**
 * Provides tuning functionality for the Repulsor drive and turn tuning model layer. Use this type
 * from robot code, field profiles, or tests when integrating the corresponding Repulsor subsystem.
 * Coordinates are field-relative unless a method documents robot-relative motion.
 */
public abstract class Tuning {
  private final String key;
  private boolean enabled = true;
  private double dtSeconds = 0.02;

  protected Tuning(String key) {
    this.key = key;
    applyDefaults();
  }

  protected static double clamp01(double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }

  protected static double smooth01(double x) {
    x = clamp01(x);
    return x * x * (3.0 - 2.0 * x);
  }

  protected static double sigmoid(double x) {
    return 1.0 / (1.0 + Math.exp(-x));
  }

  /**
   * Returns the key value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String key() {
    return key;
  }

  /**
   * Returns the enabled value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean enabled() {
    return enabled;
  }

  /**
   * Updates set enabled state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param enabled value used by this operation.
   */
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  /**
   * Returns the dt seconds value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double dtSeconds() {
    return dtSeconds;
  }

  /**
   * Updates set dt seconds state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param dtSeconds time value in seconds.
   */
  public void setDtSeconds(double dtSeconds) {
    this.dtSeconds = dtSeconds;
  }

  /** Runs apply defaults in the Repulsor runtime. */
  public abstract void applyDefaults();

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  public abstract void reset();
}
