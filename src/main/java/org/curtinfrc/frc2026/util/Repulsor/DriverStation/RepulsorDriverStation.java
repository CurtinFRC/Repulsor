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

import java.util.Objects;

/**
 * Provides repulsor driver station functionality for the Repulsor driver-station and NetworkTables
 * control surface. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public abstract class RepulsorDriverStation implements AutoCloseable {
  private static volatile RepulsorDriverStation INSTANCE;

  /**
   * Returns the get instance value maintained by this Repulsor component.
   *
   * @return repulsor driver station result for get instance.
   */
  public static RepulsorDriverStation getInstance() {
    RepulsorDriverStation inst = INSTANCE;
    if (inst == null) throw new IllegalStateException("RepulsorDriverStation not initialized");
    return inst;
  }

  /**
   * Returns the is initialized value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static boolean isInitialized() {
    return INSTANCE != null;
  }

  /**
   * Updates set instance state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param instance value used by this operation.
   */
  public static void setInstance(RepulsorDriverStation instance) {
    Objects.requireNonNull(instance, "instance");
    RepulsorDriverStation prev;
    synchronized (RepulsorDriverStation.class) {
      prev = INSTANCE;
      INSTANCE = instance;
    }
    if (prev != null && prev != instance) prev.close();
  }

  /**
   * Updates clear instance state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   */
  public static void clearInstance() {
    RepulsorDriverStation prev;
    synchronized (RepulsorDriverStation.class) {
      prev = INSTANCE;
      INSTANCE = null;
    }
    if (prev != null) prev.close();
  }

  /** Runs tick in the Repulsor runtime. */
  public abstract void tick();

  /** Runs close in the Repulsor runtime. */
  @Override
  public abstract void close();
}
