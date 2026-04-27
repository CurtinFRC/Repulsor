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
package org.curtinfrc.frc2026.util.Repulsor.State;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

/**
 * Provides state manager functionality for the Repulsor match-state storage and simulation driver
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public class StateManager {
  public static final StateManager INSTANCE = new StateManager();

  /**
   * Provides state registry functionality for the Repulsor match-state storage and simulation
   * driver layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  static class StateRegistry {
    private static final Map<String, State> states = new HashMap<>();

    /**
     * Updates register state state or telemetry as part of the Repulsor runtime loop. This may
     * mutate local state, NetworkTables output, planner caches, or command-side runtime state
     * depending on the owning type.
     *
     * @param state value used by this operation.
     */
    public static void registerState(State state) {
      states.put(state.getClass().getName(), state);
    }

    /**
     * Returns the get state value maintained by this Repulsor component.
     *
     * @param name value used by this operation.
     * @return value produced by this operation.
     */
    public static State getState(String name) {
      return states.get(name);
    }

    /**
     * Returns the get all states value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public static Collection<State> getAllStates() {
      return states.values();
    }
  }

  static {
    StateRegistry.registerState(new GameState());
  }

  private StateManager() {}

  /**
   * Updates register state state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param state value used by this operation.
   */
  public static void registerState(State state) {
    StateRegistry.registerState(state);
  }

  /**
   * Returns the get state value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @return value produced by this operation.
   */
  public static State getState(String name) {
    return StateRegistry.getState(name);
  }

  /**
   * Returns the get state value maintained by this Repulsor component.
   *
   * @param type value used by this operation.
   * @return value produced by this operation.
   */
  public static <T extends State> T getState(Class<T> type) {
    State state = StateRegistry.getState(type.getName());
    if (state == null) {
      return null;
    }
    return type.cast(state);
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param dt value used by this operation.
   */
  public static void update(double dt) {
    for (State state : StateRegistry.getAllStates()) {
      state.update(dt);
    }
  }
}
