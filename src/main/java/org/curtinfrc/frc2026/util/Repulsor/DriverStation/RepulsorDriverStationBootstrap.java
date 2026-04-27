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

package org.curtinfrc.frc2026.util.Repulsor.DriverStation;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Objects;

/**
 * Provides repulsor driver station bootstrap functionality for the Repulsor driver-station and
 * NetworkTables control surface. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class RepulsorDriverStationBootstrap {
  private RepulsorDriverStationBootstrap() {}

  /** Runs use default nt in the Repulsor runtime. */
  public static void useDefaultNt() {
    useNt(NetworkTableInstance.getDefault(), "/Repulsor/DriverStation");
  }

  /**
   * Runs use nt in the Repulsor runtime.
   *
   * @param inst value used by this operation.
   * @param root value used by this operation.
   */
  public static void useNt(NetworkTableInstance inst, String root) {
    Objects.requireNonNull(inst, "inst");
    if (root == null || root.isEmpty()) throw new IllegalArgumentException("root");
    RepulsorDriverStation.setInstance(new DefaultNtRepulsorDriverStation(inst, root));
  }
}
