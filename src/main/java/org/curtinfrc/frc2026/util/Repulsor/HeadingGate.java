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

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Provides heading gate functionality for the Repulsor core Repulsor coordination layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public final class HeadingGate {
  /**
   * Provides config functionality for the Repulsor core Repulsor coordination layer. Use this type
   * from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static final class Config {
    public double deadbandDeg = 6.0;
    public double releaseDeg = 9.0;
    public double snapDeg = 22.0;
    public double maxDegPerSec = 180.0;
  }

  private final Config cfg = new Config();
  private Rotation2d held;
  private boolean latched = false;

  /**
   * Updates configure state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param c value used by this operation.
   */
  public void configure(java.util.function.Consumer<Config> c) {
    c.accept(cfg);
  }

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param current value used by this operation.
   */
  public void reset(Rotation2d current) {
    held = current;
    latched = false;
  }

  /**
   * Returns the filter value maintained by this Repulsor component.
   *
   * @param currentYaw value used by this operation.
   * @param desiredYaw value used by this operation.
   * @param dtSeconds time value in seconds.
   * @return value produced by this operation.
   */
  public Rotation2d filter(Rotation2d currentYaw, Rotation2d desiredYaw, double dtSeconds) {
    if (held == null) held = currentYaw;

    double errDeg = Math.toDegrees(wrap(desiredYaw.getRadians() - held.getRadians()));
    if (!latched) {
      if (Math.abs(errDeg) > cfg.releaseDeg) latched = true;
      else return held;
    } else {
      if (Math.abs(errDeg) < cfg.deadbandDeg) latched = false;
    }

    double maxStepDeg = Math.max(0.0, cfg.maxDegPerSec * dtSeconds);
    double stepDeg = clamp(errDeg, -maxStepDeg, maxStepDeg);

    if (Math.abs(errDeg) >= cfg.snapDeg) held = desiredYaw;
    else held = Rotation2d.fromRadians(held.getRadians() + Math.toRadians(stepDeg));

    return held;
  }

  private static double wrap(double a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }
}
