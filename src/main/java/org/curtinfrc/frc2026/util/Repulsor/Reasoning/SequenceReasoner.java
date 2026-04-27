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
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;

/**
 * Provides sequence reasoner functionality for the Repulsor rule-based strategy and signal
 * reasoning layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class SequenceReasoner<F extends Enum<F>, C> implements Reasoner<F, C> {
  /**
   * Provides transition functionality for the Repulsor rule-based strategy and signal reasoning
   * layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static final class Transition<F extends Enum<F>, C> {
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
     * Configuration value for min phase age sec. Time values use seconds and should be tuned
     * against measured robot loop and mechanism latency.
     */
    public final double minPhaseAgeSec;

    /**
     * Configuration value for condition. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public final Condition<C> condition;

    /**
     * Configuration value for next phase index. The valid range and tuning source are defined by
     * the owning subsystem or field profile.
     */
    public final int nextPhaseIndex;

    /**
     * Returns the transition value maintained by this Repulsor component.
     *
     * @param name value used by this operation.
     * @param priority distance or field-coordinate value in meters.
     * @param minPhaseAgeSec time value in seconds.
     * @param condition value used by this operation.
     * @param nextPhaseIndex distance or field-coordinate value in meters.
     */
    public Transition(
        String name,
        int priority,
        double minPhaseAgeSec,
        Condition<C> condition,
        int nextPhaseIndex) {
      this.name = Objects.requireNonNull(name);
      this.priority = priority;
      this.minPhaseAgeSec = Math.max(0.0, minPhaseAgeSec);
      this.condition = Objects.requireNonNull(condition);
      this.nextPhaseIndex = nextPhaseIndex;
    }
  }

  /**
   * Provides phase functionality for the Repulsor rule-based strategy and signal reasoning layer.
   * Use this type from robot code, field profiles, or tests when integrating the corresponding
   * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
   * motion.
   */
  public static final class Phase<F extends Enum<F>, C> {
    /**
     * Configuration value for name. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final String name;

    /**
     * Configuration value for flags. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final EnumSet<F> flags;

    /**
     * Configuration value for min duration sec. Time values use seconds and should be tuned against
     * measured robot loop and mechanism latency.
     */
    public final double minDurationSec;

    /**
     * Configuration value for max duration sec. Time values use seconds and should be tuned against
     * measured robot loop and mechanism latency.
     */
    public final double maxDurationSec;

    /**
     * Configuration value for auto next index. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public final int autoNextIndex;

    /**
     * Configuration value for transitions. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public final List<Transition<F, C>> transitions;

    /**
     * Returns the phase value maintained by this Repulsor component.
     *
     * @param name value used by this operation.
     * @param flags value used by this operation.
     * @param minDurationSec time value in seconds.
     * @param maxDurationSec time value in seconds.
     * @param autoNextIndex distance or field-coordinate value in meters.
     * @param transitions value used by this operation.
     */
    public Phase(
        String name,
        EnumSet<F> flags,
        double minDurationSec,
        double maxDurationSec,
        int autoNextIndex,
        List<Transition<F, C>> transitions) {
      this.name = Objects.requireNonNull(name);
      this.flags = EnumSet.copyOf(Objects.requireNonNull(flags));
      this.minDurationSec = Math.max(0.0, minDurationSec);
      this.maxDurationSec = maxDurationSec <= 0.0 ? Double.POSITIVE_INFINITY : maxDurationSec;
      this.autoNextIndex = autoNextIndex;
      List<Transition<F, C>> copy = new ArrayList<>(Objects.requireNonNull(transitions));
      copy.sort(Comparator.comparingInt((Transition<F, C> t) -> t.priority).reversed());
      this.transitions = Collections.unmodifiableList(copy);
    }
  }

  /**
   * Provides builder functionality for the Repulsor rule-based strategy and signal reasoning layer.
   * Use this type from robot code, field profiles, or tests when integrating the corresponding
   * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
   * motion.
   */
  public static final class Builder<F extends Enum<F>, C> {
    private final Class<F> enumClass;
    private final Clock clock;
    private final Signals signals;
    private final List<Phase<F, C>> phases = new ArrayList<>();
    private int startIndex = 0;

    /**
     * Returns the builder value maintained by this Repulsor component.
     *
     * @param enumClass value used by this operation.
     * @param clock value used by this operation.
     */
    public Builder(Class<F> enumClass, Clock clock) {
      this(enumClass, clock, new Blackboard());
    }

    /**
     * Returns the builder value maintained by this Repulsor component.
     *
     * @param enumClass value used by this operation.
     * @param clock value used by this operation.
     * @param signals value used by this operation.
     */
    public Builder(Class<F> enumClass, Clock clock, Signals signals) {
      this.enumClass = Objects.requireNonNull(enumClass);
      this.clock = Objects.requireNonNull(clock);
      this.signals = Objects.requireNonNull(signals);
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
     * Updates start at state or telemetry as part of the Repulsor runtime loop. This may mutate
     * local state, NetworkTables output, planner caches, or command-side runtime state depending on
     * the owning type.
     *
     * @param index distance or field-coordinate value in meters.
     * @return value produced by this operation.
     */
    public Builder<F, C> startAt(int index) {
      this.startIndex = Math.max(0, index);
      return this;
    }

    /**
     * Returns the add phase value maintained by this Repulsor component.
     *
     * @param name value used by this operation.
     * @param flags value used by this operation.
     * @param minDurationSec time value in seconds.
     * @param maxDurationSec time value in seconds.
     * @param autoNextIndex distance or field-coordinate value in meters.
     * @return value produced by this operation.
     */
    public Builder<F, C> addPhase(
        String name,
        EnumSet<F> flags,
        double minDurationSec,
        double maxDurationSec,
        int autoNextIndex) {
      phases.add(
          new Phase<>(
              name, flags, minDurationSec, maxDurationSec, autoNextIndex, new ArrayList<>()));
      return this;
    }

    /**
     * Returns the add phase for value maintained by this Repulsor component.
     *
     * @param name value used by this operation.
     * @param flags value used by this operation.
     * @param durationSec time value in seconds.
     * @return value produced by this operation.
     */
    public Builder<F, C> addPhaseFor(String name, EnumSet<F> flags, double durationSec) {
      int next = phases.size() + 1;
      phases.add(new Phase<>(name, flags, durationSec, durationSec, next, new ArrayList<>()));
      return this;
    }

    /**
     * Returns the add transition value maintained by this Repulsor component.
     *
     * @param fromPhaseIndex distance or field-coordinate value in meters.
     * @param name value used by this operation.
     * @param priority distance or field-coordinate value in meters.
     * @param minPhaseAgeSec time value in seconds.
     * @param condition value used by this operation.
     * @param nextPhaseIndex distance or field-coordinate value in meters.
     * @return value produced by this operation.
     */
    public Builder<F, C> addTransition(
        int fromPhaseIndex,
        String name,
        int priority,
        double minPhaseAgeSec,
        Condition<C> condition,
        int nextPhaseIndex) {
      if (fromPhaseIndex < 0 || fromPhaseIndex >= phases.size()) {
        throw new IndexOutOfBoundsException(
            "fromPhaseIndex=" + fromPhaseIndex + " size=" + phases.size());
      }
      Phase<F, C> p = phases.get(fromPhaseIndex);
      List<Transition<F, C>> ts = new ArrayList<>(p.transitions);
      ts.add(new Transition<>(name, priority, minPhaseAgeSec, condition, nextPhaseIndex));
      phases.set(
          fromPhaseIndex,
          new Phase<>(p.name, p.flags, p.minDurationSec, p.maxDurationSec, p.autoNextIndex, ts));
      return this;
    }

    /**
     * Builds the WPILib command sequence for the current behaviour context.
     *
     * @return value produced by this operation.
     */
    public SequenceReasoner<F, C> build() {
      if (phases.isEmpty()) {
        phases.add(
            new Phase<>(
                "idle",
                EnumSet.noneOf(enumClass),
                0.0,
                Double.POSITIVE_INFINITY,
                0,
                new ArrayList<>()));
      }
      int s = Math.min(startIndex, phases.size() - 1);
      return new SequenceReasoner<>(enumClass, clock, signals, phases, s);
    }
  }

  private final Class<F> enumClass;
  private final Clock clock;
  private final Signals signals;
  private final List<Phase<F, C>> phases;

  private int phaseIndex;
  private double phaseStartSec;

  /**
   * Returns the sequence reasoner value maintained by this Repulsor component.
   *
   * @param enumClass value used by this operation.
   * @param clock value used by this operation.
   * @param signals value used by this operation.
   * @param phases value used by this operation.
   * @param startIndex distance or field-coordinate value in meters.
   */
  public SequenceReasoner(
      Class<F> enumClass, Clock clock, Signals signals, List<Phase<F, C>> phases, int startIndex) {
    this.enumClass = Objects.requireNonNull(enumClass);
    this.clock = Objects.requireNonNull(clock);
    this.signals = Objects.requireNonNull(signals);
    this.phases = List.copyOf(Objects.requireNonNull(phases));
    this.phaseIndex = Math.max(0, Math.min(startIndex, this.phases.size() - 1));
    this.phaseStartSec = Double.NaN;
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
   * Returns the phase index value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public int phaseIndex() {
    return phaseIndex;
  }

  /**
   * Returns the phase name value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String phaseName() {
    return phases.get(phaseIndex).name;
  }

  /**
   * Runs force phase in the Repulsor runtime.
   *
   * @param index distance or field-coordinate value in meters.
   */
  public void forcePhase(int index) {
    int clamped = Math.max(0, Math.min(index, phases.size() - 1));
    if (clamped != phaseIndex) {
      phaseIndex = clamped;
      phaseStartSec = clock.nowSec();
    }
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
    double now = clock.nowSec();
    if (!Double.isFinite(phaseStartSec)) phaseStartSec = now;

    Phase<F, C> p = phases.get(phaseIndex);
    double age = Math.max(0.0, now - phaseStartSec);

    int chosenNext = -1;
    int chosenPrio = Integer.MIN_VALUE;

    if (age >= p.minDurationSec) {
      for (Transition<F, C> t : p.transitions) {
        if (age < t.minPhaseAgeSec) continue;
        if (!t.condition.test(ctx, signals)) continue;
        if (t.priority > chosenPrio) {
          chosenPrio = t.priority;
          chosenNext = t.nextPhaseIndex;
        }
      }
    }

    if (chosenNext < 0
        && age >= p.maxDurationSec
        && p.autoNextIndex >= 0
        && p.autoNextIndex < phases.size()) {
      chosenNext = p.autoNextIndex;
    }

    if (chosenNext >= 0 && chosenNext < phases.size() && chosenNext != phaseIndex) {
      phaseIndex = chosenNext;
      phaseStartSec = now;
      p = phases.get(phaseIndex);
    }

    return EnumSet.copyOf(p.flags);
  }

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  @Override
  public void reset() {
    signals.clear();
    phaseIndex = 0;
    phaseStartSec = Double.NaN;
  }
}
