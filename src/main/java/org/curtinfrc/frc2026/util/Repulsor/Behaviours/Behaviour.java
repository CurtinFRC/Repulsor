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

package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.EnumSet;

/**
 * Provides behaviour functionality for the Repulsor command-behaviour layer that converts strategy
 * and state into WPILib commands. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public abstract class Behaviour {
  /**
   * Returns the name value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract String name();

  /**
   * Returns the priority value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract int priority();

  /**
   * Returns the should run value maintained by this Repulsor component.
   *
   * @param flags value used by this operation.
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  public abstract boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx);

  /**
   * Builds the WPILib command sequence for the current behaviour context.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  public abstract Command build(BehaviourContext ctx);
}
