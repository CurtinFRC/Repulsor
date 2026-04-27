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

package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;

/**
 * Provides rule reasoner functionality for the Repulsor rule-based strategy and signal reasoning
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public final class RuleReasoner<F extends Enum<F>, C> implements Reasoner<F, C> {
  /**
   * Provides rule functionality for the Repulsor rule-based strategy and signal reasoning layer.
   * Use this type from robot code, field profiles, or tests when integrating the corresponding
   * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
   * motion.
   */
  public static final class Rule<F extends Enum<F>, C> {
    /**
     * Configuration value for name. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final String name;

    /**
     * Configuration value for priority. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final int priority;

    /**
     * Configuration value for condition. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public final Condition<C> condition;

    /**
     * Configuration value for flags. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final EnumSet<F> flags;

    /**
     * Returns the rule value maintained by this Repulsor component.
     *
     * @param name value used by this operation.
     * @param priority distance or field-coordinate value in meters.
     * @param condition value used by this operation.
     * @param flags value used by this operation.
     */
    public Rule(String name, int priority, Condition<C> condition, EnumSet<F> flags) {
      this.name = Objects.requireNonNull(name);
      this.priority = priority;
      this.condition = Objects.requireNonNull(condition);
      this.flags = Objects.requireNonNull(flags);
    }
  }

  private final Class<F> enumClass;
  private final Signals signals;
  private final List<Rule<F, C>> rules = new ArrayList<>();
  private EnumSet<F> fallback;

  /**
   * Returns the rule reasoner value maintained by this Repulsor component.
   *
   * @param enumClass value used by this operation.
   */
  public RuleReasoner(Class<F> enumClass) {
    this(enumClass, new Blackboard());
  }

  /**
   * Returns the rule reasoner value maintained by this Repulsor component.
   *
   * @param enumClass value used by this operation.
   * @param signals value used by this operation.
   */
  public RuleReasoner(Class<F> enumClass, Signals signals) {
    this.enumClass = Objects.requireNonNull(enumClass);
    this.signals = Objects.requireNonNull(signals);
    this.fallback = EnumSet.noneOf(enumClass);
  }

  /**
   * Returns the signals value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Signals signals() {
    return signals;
  }

  /**
   * Updates set fallback state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param flags value used by this operation.
   * @return value produced by this operation.
   */
  public RuleReasoner<F, C> setFallback(EnumSet<F> flags) {
    this.fallback = Objects.requireNonNull(flags);
    return this;
  }

  /**
   * Returns the add rule value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @param priority distance or field-coordinate value in meters.
   * @param condition value used by this operation.
   * @param flags value used by this operation.
   * @return value produced by this operation.
   */
  public RuleReasoner<F, C> addRule(
      String name, int priority, Condition<C> condition, EnumSet<F> flags) {
    rules.add(new Rule<>(name, priority, condition, flags));
    return this;
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return enum set of f result for update.
   */
  @Override
  public EnumSet<F> update(C ctx) {
    Rule<F, C> best = null;
    int bestP = Integer.MIN_VALUE;
    for (Rule<F, C> r : rules) {
      if (r.condition.test(ctx, signals) && r.priority > bestP) {
        best = r;
        bestP = r.priority;
      }
    }
    return best != null ? EnumSet.copyOf(best.flags) : EnumSet.copyOf(fallback);
  }

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  @Override
  public void reset() {
    signals.clear();
  }
}
