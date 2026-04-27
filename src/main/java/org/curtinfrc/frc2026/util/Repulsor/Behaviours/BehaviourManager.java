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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Reasoning.Reasoner;

/**
 * Provides behaviour manager functionality for the Repulsor command-behaviour layer that converts
 * strategy and state into WPILib commands. Use this type from robot code, field profiles, or tests
 * when integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
 * method documents robot-relative motion.
 */
public class BehaviourManager {
  private final List<Behaviour> behaviours = new ArrayList<>();
  private Behaviour active = null;
  private Command running = null;

  private Reasoner<BehaviourFlag, BehaviourContext> reasoner = null;

  /**
   * Returns the add value maintained by this Repulsor component.
   *
   * @param b value used by this operation.
   * @return behaviour manager result for add.
   */
  public BehaviourManager add(Behaviour b) {
    behaviours.add(b);
    return this;
  }

  /**
   * Returns the active value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Behaviour active() {
    return active;
  }

  /**
   * Updates set reasoner state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param reasoner value used by this operation.
   * @return behaviour manager result for set reasoner.
   */
  public BehaviourManager setReasoner(Reasoner<BehaviourFlag, BehaviourContext> reasoner) {
    this.reasoner = reasoner;
    return this;
  }

  /**
   * Updates clear state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @return behaviour manager result for clear.
   */
  public BehaviourManager clear() {
    stop();
    behaviours.clear();
    return this;
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   */
  public void update(BehaviourContext ctx) {
    EnumSet<BehaviourFlag> flags =
        reasoner != null ? reasoner.update(ctx) : EnumSet.noneOf(BehaviourFlag.class);

    update(flags, ctx);
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param flags value used by this operation.
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   */
  public void update(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
    Behaviour best = null;
    int bestP = Integer.MIN_VALUE;
    for (Behaviour b : behaviours) {
      if (b.shouldRun(flags, ctx) && b.priority() > bestP) {
        best = b;
        bestP = b.priority();
      }
    }
    if (best == active) return;

    if (running != null) {
      running.cancel();
      running = null;
    }
    active = best;
    if (active != null) {
      running = active.build(ctx);
      CommandScheduler.getInstance().schedule(running);
    }
  }

  /**
   * Updates stop state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  public void stop() {
    if (running != null) running.cancel();
    running = null;
    active = null;
    if (reasoner != null) reasoner.reset();
  }
}
