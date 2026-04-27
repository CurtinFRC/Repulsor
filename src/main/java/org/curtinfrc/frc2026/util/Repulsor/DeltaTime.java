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

import edu.wpi.first.wpilibj.Timer;

/**
 * Provides delta time functionality for the Repulsor core Repulsor coordination layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public final class DeltaTime {
  private static final DeltaTime INSTANCE = new DeltaTime();

  /**
   * Returns the get instance value maintained by this Repulsor component.
   *
   * @return delta time result for get instance.
   */
  public static DeltaTime getInstance() {
    return INSTANCE;
  }

  /**
   * Returns the latest value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static double get() {
    return INSTANCE._get();
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  public static void update() {
    INSTANCE._update();
  }

  private double lastTimeS = Double.NaN;
  private volatile double dtS = 0.0;

  private static final double MAX_DT_S = 0.1;

  private DeltaTime() {}

  private void _update() {
    final double nowS = Timer.getFPGATimestamp();

    if (Double.isNaN(lastTimeS)) {
      dtS = 0.0;
    } else {
      double dt = nowS - lastTimeS;

      if (dt < 0.0) dt = 0.0;

      if (dt > MAX_DT_S) dt = MAX_DT_S;

      dtS = dt;
    }

    lastTimeS = nowS;
  }

  private double _get() {
    return dtS;
  }
}
