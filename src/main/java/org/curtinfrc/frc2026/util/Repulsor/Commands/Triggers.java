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

package org.curtinfrc.frc2026.util.Repulsor.Commands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Provides triggers functionality for the Repulsor WPILib command and Trigger integration layer.
 * Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public final class Triggers {
  private Triggers() {}

  private static double DEFAULT_DEBOUNCE = 0.05;

  /**
   * Updates set debounce default state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param seconds time value in seconds.
   */
  public static void setDebounceDefault(double seconds) {
    DEFAULT_DEBOUNCE = seconds;
  }

  /**
   * Returns the rising value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @param seconds time value in seconds.
   * @return value produced by this operation.
   */
  public static Trigger rising(Trigger src, double seconds) {
    return src.debounce(seconds, DebounceType.kRising);
  }

  /**
   * Returns the rising value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger rising(Trigger src) {
    return rising(src, DEFAULT_DEBOUNCE);
  }

  /**
   * Returns the falling value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @param seconds time value in seconds.
   * @return value produced by this operation.
   */
  public static Trigger falling(Trigger src, double seconds) {
    return src.debounce(seconds, DebounceType.kFalling);
  }

  /**
   * Returns the falling value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger falling(Trigger src) {
    return falling(src, DEFAULT_DEBOUNCE);
  }

  /**
   * Returns the debounce value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @param seconds time value in seconds.
   * @param type value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger debounce(Trigger src, double seconds, DebounceType type) {
    return src.debounce(seconds, type);
  }

  /**
   * Returns the cooldown value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @param cooldownSec time value in seconds.
   * @return value produced by this operation.
   */
  public static Trigger cooldown(Trigger src, double cooldownSec) {
    final double[] nextOkTime = {0.0};
    return new Trigger(
        () -> {
          boolean now = src.getAsBoolean();
          double t = Timer.getFPGATimestamp();
          if (now && t >= nextOkTime[0]) {
            nextOkTime[0] = t + cooldownSec;
            return true;
          }
          return false;
        });
  }

  /**
   * Returns the throttle value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @param cooldownSec time value in seconds.
   * @return value produced by this operation.
   */
  public static Trigger throttle(Trigger src, double cooldownSec) {
    return cooldown(src, cooldownSec);
  }

  /**
   * Returns the pulse on rise value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @param debounceSec time value in seconds.
   * @return value produced by this operation.
   */
  public static Trigger pulseOnRise(Trigger src, double debounceSec) {
    return rising(src, debounceSec);
  }

  /**
   * Returns the pulse on rise value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger pulseOnRise(Trigger src) {
    return rising(src);
  }

  /**
   * Returns the oneshot value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger oneshot(Trigger src) {
    final boolean[] fired = {false};
    return new Trigger(
        () -> {
          if (fired[0]) return false;
          boolean now = src.getAsBoolean();
          if (now) {
            fired[0] = true;
            return true;
          }
          return false;
        });
  }

  /**
   * Returns the oneshot until value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @param reset value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger oneshotUntil(Trigger src, Trigger reset) {
    final boolean[] fired = {false};
    reset.onTrue(Commands.runOnce(() -> fired[0] = false).ignoringDisable(true));
    return new Trigger(
        () -> {
          if (fired[0]) return false;
          boolean now = src.getAsBoolean();
          if (now) {
            fired[0] = true;
            return true;
          }
          return false;
        });
  }

  /**
   * Returns the latch until value maintained by this Repulsor component.
   *
   * @param risingSrc value used by this operation.
   * @param until value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger latchUntil(Trigger risingSrc, Trigger until) {
    final boolean[] latched = {false};
    return new Trigger(
        () -> {
          if (until.getAsBoolean()) {
            latched[0] = false;
            return false;
          }
          if (risingSrc.getAsBoolean()) latched[0] = true;
          return latched[0];
        });
  }

  /**
   * Returns the latch value maintained by this Repulsor component.
   *
   * @param arm value used by this operation.
   * @param disarm value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger latch(Trigger arm, Trigger disarm) {
    final boolean[] latched = {false};
    return new Trigger(
        () -> {
          if (disarm.getAsBoolean()) {
            latched[0] = false;
            return false;
          }
          if (arm.getAsBoolean()) {
            latched[0] = true;
          }
          return latched[0];
        });
  }

  /**
   * Returns the level value maintained by this Repulsor component.
   *
   * @param s value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger level(BooleanSupplier s) {
    return new Trigger(s::getAsBoolean);
  }

  /**
   * Returns the edge value maintained by this Repulsor component.
   *
   * @param s value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger edge(BooleanSupplier s) {
    return rising(level(s));
  }

  /**
   * Returns the and also value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @param fn value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger andAlso(Trigger src, BooleanSupplier fn) {
    return new Trigger(() -> src.getAsBoolean() && fn.getAsBoolean());
  }

  /**
   * Returns the map value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @param fn value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger map(Trigger src, java.util.function.Predicate<Boolean> fn) {
    return new Trigger(() -> fn.test(src.getAsBoolean()));
  }

  /**
   * Returns the any of value maintained by this Repulsor component.
   *
   * @param ts value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger anyOf(Trigger... ts) {
    return new Trigger(
        () -> {
          for (Trigger t : ts) if (t.getAsBoolean()) return true;
          return false;
        });
  }

  /**
   * Returns the all of value maintained by this Repulsor component.
   *
   * @param ts value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger allOf(Trigger... ts) {
    return new Trigger(
        () -> {
          for (Trigger t : ts) if (!t.getAsBoolean()) return false;
          return true;
        });
  }

  /**
   * Returns the rise of value maintained by this Repulsor component.
   *
   * @param level value used by this operation.
   * @param debounceSec time value in seconds.
   * @return value produced by this operation.
   */
  public static Trigger riseOf(Trigger level, double debounceSec) {
    return rising(level, debounceSec);
  }

  /**
   * Returns the rise of value maintained by this Repulsor component.
   *
   * @param level value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger riseOf(Trigger level) {
    return rising(level);
  }

  /**
   * Returns the fall of value maintained by this Repulsor component.
   *
   * @param level value used by this operation.
   * @param debounceSec time value in seconds.
   * @return value produced by this operation.
   */
  public static Trigger fallOf(Trigger level, double debounceSec) {
    return rising(level.negate(), debounceSec);
  }

  /**
   * Returns the fall of value maintained by this Repulsor component.
   *
   * @param level value used by this operation.
   * @return value produced by this operation.
   */
  public static Trigger fallOf(Trigger level) {
    return rising(level.negate(), DEFAULT_DEBOUNCE);
  }

  /**
   * Returns the while in phase value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param allowed value used by this operation.
   * @return value produced by this operation.
   */
  public static <T> Trigger whileInPhase(PhaseGate<T> gate, T allowed) {
    return new Trigger(() -> Objects.equals(gate.phase(), allowed));
  }

  /**
   * Returns the while in phase value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param allowed value used by this operation.
   * @param cond value used by this operation.
   * @return value produced by this operation.
   */
  public static <T> Trigger whileInPhase(PhaseGate<T> gate, T allowed, BooleanSupplier cond) {
    return new Trigger(() -> Objects.equals(gate.phase(), allowed) && cond.getAsBoolean());
  }

  /**
   * Returns the delay value maintained by this Repulsor component.
   *
   * @param src value used by this operation.
   * @param seconds time value in seconds.
   * @return value produced by this operation.
   */
  public static Trigger delay(Trigger src, double seconds) {
    final double[] armAt = {Double.NaN};
    return new Trigger(
        () -> {
          boolean now = src.getAsBoolean();
          double t = Timer.getFPGATimestamp();
          if (now) {
            if (Double.isNaN(armAt[0])) armAt[0] = t + seconds;
            return t >= armAt[0];
          } else {
            armAt[0] = Double.NaN;
            return false;
          }
        });
  }

  /**
   * Provides phase gate functionality for the Repulsor WPILib command and Trigger integration
   * layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static final class PhaseGate<T> {
    private final Supplier<T> get;
    private final Consumer<T> set;

    /**
     * Returns the phase gate value maintained by this Repulsor component.
     *
     * @param getter value used by this operation.
     * @param setter value used by this operation.
     */
    public PhaseGate(Supplier<T> getter, Consumer<T> setter) {
      this.get = Objects.requireNonNull(getter);
      this.set = Objects.requireNonNull(setter);
    }

    /**
     * Returns the phase value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public T phase() {
      return get.get();
    }

    /**
     * Updates set phase state or telemetry as part of the Repulsor runtime loop. This may mutate
     * local state, NetworkTables output, planner caches, or command-side runtime state depending on
     * the owning type.
     *
     * @param p value used by this operation.
     */
    public void setPhase(T p) {
      set.accept(p);
    }

    /**
     * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
     * state, NetworkTables output, planner caches, or command-side runtime state depending on the
     * owning type.
     *
     * @param p value used by this operation.
     */
    public void set(T p) {
      set.accept(p);
    }

    /**
     * Returns the when in phase value maintained by this Repulsor component.
     *
     * @param src value used by this operation.
     * @param allowed value used by this operation.
     * @return value produced by this operation.
     */
    public Trigger whenInPhase(Trigger src, T allowed) {
      return new Trigger(() -> Objects.equals(get.get(), allowed) && src.getAsBoolean());
    }

    /**
     * Returns the when value maintained by this Repulsor component.
     *
     * @param allowed value used by this operation.
     * @return value produced by this operation.
     */
    public Trigger when(T allowed) {
      return new Trigger(() -> Objects.equals(get.get(), allowed));
    }

    /**
     * Runs transition on in the Repulsor runtime.
     *
     * @param src value used by this operation.
     * @param toPhase value used by this operation.
     */
    public void transitionOn(Trigger src, T toPhase) {
      src.onTrue(Commands.runOnce(() -> set.accept(toPhase)).ignoringDisable(true));
    }

    /**
     * Returns the on enter value maintained by this Repulsor component.
     *
     * @param target value used by this operation.
     * @return value produced by this operation.
     */
    public Trigger onEnter(T target) {
      final Object[] last = {get.get()};
      return new Trigger(
          () -> {
            T cur = get.get();
            boolean enter = !Objects.equals(last[0], target) && Objects.equals(cur, target);
            last[0] = cur;
            return enter;
          });
    }

    /**
     * Returns the on exit value maintained by this Repulsor component.
     *
     * @param target value used by this operation.
     * @return value produced by this operation.
     */
    public Trigger onExit(T target) {
      final Object[] last = {get.get()};
      return new Trigger(
          () -> {
            T cur = get.get();
            boolean exit = Objects.equals(last[0], target) && !Objects.equals(cur, target);
            last[0] = cur;
            return exit;
          });
    }
  }

  @SuppressWarnings("unchecked")
  /**
   * Returns the local phase value maintained by this Repulsor component.
   *
   * @param initial value used by this operation.
   * @return phase gate of t result for local phase.
   */
  public static <T> PhaseGate<T> localPhase(T initial) {
    final Object[] box = {initial};
    return new PhaseGate<>(() -> (T) box[0], v -> box[0] = v);
  }

  /**
   * Provides state machine functionality for the Repulsor WPILib command and Trigger integration
   * layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static final class StateMachine<T> {
    private final PhaseGate<T> gate;

    private static final class Transition<T> {
      final T from;
      final T to;
      final Trigger when;
      final Runnable onTransition;

      Transition(T f, T t, Trigger w, Runnable r) {
        from = f;
        to = Objects.requireNonNull(t);
        when = Objects.requireNonNull(w);
        onTransition = r;
      }
    }

    private final List<Transition<T>> transitions = new ArrayList<>();
    private final Map<T, List<Runnable>> enterCbs = new HashMap<>();
    private final Map<T, List<Runnable>> exitCbs = new HashMap<>();
    private boolean wired = false;

    /**
     * Returns the state machine value maintained by this Repulsor component.
     *
     * @param gate value used by this operation.
     */
    public StateMachine(PhaseGate<T> gate) {
      this.gate = Objects.requireNonNull(gate);
    }

    /**
     * Returns the transition value maintained by this Repulsor component.
     *
     * @param from value used by this operation.
     * @param to value used by this operation.
     * @param when value used by this operation.
     * @return state machine of t result for transition.
     */
    public StateMachine<T> transition(T from, T to, Trigger when) {
      transitions.add(new Transition<>(from, to, when, null));
      return this;
    }

    /**
     * Returns the any value maintained by this Repulsor component.
     *
     * @param to value used by this operation.
     * @param when value used by this operation.
     * @return state machine of t result for any.
     */
    public StateMachine<T> any(T to, Trigger when) {
      transitions.add(new Transition<>(null, to, when, null));
      return this;
    }

    /**
     * Returns the transition value maintained by this Repulsor component.
     *
     * @param from value used by this operation.
     * @param to value used by this operation.
     * @param when value used by this operation.
     * @param onTransition value used by this operation.
     * @return state machine of t result for transition.
     */
    public StateMachine<T> transition(T from, T to, Trigger when, Runnable onTransition) {
      transitions.add(new Transition<>(from, to, when, onTransition));
      return this;
    }

    /**
     * Returns the any value maintained by this Repulsor component.
     *
     * @param to value used by this operation.
     * @param when value used by this operation.
     * @param onTransition value used by this operation.
     * @return state machine of t result for any.
     */
    public StateMachine<T> any(T to, Trigger when, Runnable onTransition) {
      transitions.add(new Transition<>(null, to, when, onTransition));
      return this;
    }

    /**
     * Returns the on enter value maintained by this Repulsor component.
     *
     * @param state value used by this operation.
     * @param cb value used by this operation.
     * @return state machine of t result for on enter.
     */
    public StateMachine<T> onEnter(T state, Runnable cb) {
      enterCbs.computeIfAbsent(state, k -> new ArrayList<>()).add(cb);
      return this;
    }

    /**
     * Returns the on exit value maintained by this Repulsor component.
     *
     * @param state value used by this operation.
     * @param cb value used by this operation.
     * @return state machine of t result for on exit.
     */
    public StateMachine<T> onExit(T state, Runnable cb) {
      exitCbs.computeIfAbsent(state, k -> new ArrayList<>()).add(cb);
      return this;
    }

    /** Runs wire in the Repulsor runtime. */
    public void wire() {
      if (wired) return;
      wired = true;
      final boolean[] armedThisTick = {false};
      for (Transition<T> tr : transitions) {
        Trigger gated = (tr.from == null) ? tr.when : gate.whenInPhase(tr.when, tr.from);
        gated.onTrue(
            Commands.runOnce(
                    () -> {
                      if (armedThisTick[0]) return;
                      T prev = gate.phase();
                      if (tr.from != null && !Objects.equals(prev, tr.from)) return;
                      if (Objects.equals(prev, tr.to)) return;

                      List<Runnable> exits = exitCbs.get(prev);
                      if (exits != null) exits.forEach(Runnable::run);
                      if (tr.onTransition != null) tr.onTransition.run();
                      gate.set(tr.to);
                      List<Runnable> enters = enterCbs.get(tr.to);
                      if (enters != null) enters.forEach(Runnable::run);

                      armedThisTick[0] = true;
                    })
                .ignoringDisable(true));
      }
    }
  }

  /**
   * Returns the state machine value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @return state machine of t result for state machine.
   */
  public static <T> StateMachine<T> stateMachine(PhaseGate<T> gate) {
    return new StateMachine<>(gate);
  }

  /**
   * Returns the local state machine value maintained by this Repulsor component.
   *
   * @param initial value used by this operation.
   * @return state machine of t result for local state machine.
   */
  public static <T> StateMachine<T> localStateMachine(T initial) {
    return stateMachine(localPhase(initial));
  }

  /**
   * Returns the while in phase value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param allowed value used by this operation.
   * @param cond value used by this operation.
   * @return value produced by this operation.
   */
  public static <T> Trigger whileInPhase(PhaseGate<T> gate, T allowed, Trigger cond) {
    return new Trigger(() -> Objects.equals(gate.phase(), allowed) && cond.getAsBoolean());
  }

  /**
   * Returns the phase rise after seen false value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param phase value used by this operation.
   * @param level value used by this operation.
   * @param debounceSec time value in seconds.
   * @return value produced by this operation.
   */
  public static <T> Trigger phaseRiseAfterSeenFalse(
      PhaseGate<T> gate, T phase, Trigger level, double debounceSec) {
    final Object[] lastPhase = {gate.phase()};
    final boolean[] seenFalse = {false};
    Trigger armed =
        new Trigger(
            () -> {
              T cur = gate.phase();
              if (!Objects.equals(cur, lastPhase[0])) {
                lastPhase[0] = cur;
                seenFalse[0] = false;
              }
              if (!Objects.equals(cur, phase)) return false;
              boolean v = level.getAsBoolean();
              if (!seenFalse[0] && !v) seenFalse[0] = true;
              return seenFalse[0] && v;
            });
    return rising(armed, debounceSec);
  }

  /**
   * Returns the phase rise after seen false value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param phase value used by this operation.
   * @param level value used by this operation.
   * @return value produced by this operation.
   */
  public static <T> Trigger phaseRiseAfterSeenFalse(PhaseGate<T> gate, T phase, Trigger level) {
    return phaseRiseAfterSeenFalse(gate, phase, level, DEFAULT_DEBOUNCE);
  }

  /**
   * Returns the phase fall after seen true value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param phase value used by this operation.
   * @param level value used by this operation.
   * @param debounceSec time value in seconds.
   * @return value produced by this operation.
   */
  public static <T> Trigger phaseFallAfterSeenTrue(
      PhaseGate<T> gate, T phase, Trigger level, double debounceSec) {
    final Object[] lastPhase = {gate.phase()};
    final boolean[] seenTrue = {false};
    Trigger armed =
        new Trigger(
            () -> {
              T cur = gate.phase();
              if (!Objects.equals(cur, lastPhase[0])) {
                lastPhase[0] = cur;
                seenTrue[0] = false;
              }
              if (!Objects.equals(cur, phase)) return false;
              boolean v = level.getAsBoolean();
              if (!seenTrue[0] && v) seenTrue[0] = true;
              return seenTrue[0] && !v;
            });
    return rising(armed, debounceSec);
  }

  /**
   * Returns the phase fall after seen true value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param phase value used by this operation.
   * @param level value used by this operation.
   * @return value produced by this operation.
   */
  public static <T> Trigger phaseFallAfterSeenTrue(PhaseGate<T> gate, T phase, Trigger level) {
    return phaseFallAfterSeenTrue(gate, phase, level, DEFAULT_DEBOUNCE);
  }

  /**
   * Provides state flow functionality for the Repulsor WPILib command and Trigger integration
   * layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static final class StateFlow<T> {
    private final PhaseGate<T> gate;
    private final List<Map.Entry<Trigger, T>> initial = new ArrayList<>();
    private T initialDefault;
    private final StateMachine<T> sm;

    private static final class BoundCondition<T> {
      final Trigger src;
      final T phase;
      final Consumer<Trigger> sink;

      BoundCondition(Trigger s, T p, Consumer<Trigger> k) {
        src = s;
        phase = p;
        sink = k;
      }
    }

    private final List<BoundCondition<T>> bindings = new ArrayList<>();

    /**
     * Returns the state flow value maintained by this Repulsor component.
     *
     * @param gate value used by this operation.
     */
    public StateFlow(PhaseGate<T> gate) {
      this.gate = gate;
      this.sm = new StateMachine<>(gate);
    }

    /**
     * Returns the initial default value maintained by this Repulsor component.
     *
     * @param phase value used by this operation.
     * @return state flow of t result for initial default.
     */
    public StateFlow<T> initialDefault(T phase) {
      this.initialDefault = phase;
      return this;
    }

    /**
     * Returns the initial when value maintained by this Repulsor component.
     *
     * @param cond value used by this operation.
     * @param phase value used by this operation.
     * @return state flow of t result for initial when.
     */
    public StateFlow<T> initialWhen(Trigger cond, T phase) {
      this.initial.add(Map.entry(cond, phase));
      return this;
    }

    /**
     * Returns the transition value maintained by this Repulsor component.
     *
     * @param from value used by this operation.
     * @param to value used by this operation.
     * @param when value used by this operation.
     * @return state flow of t result for transition.
     */
    public StateFlow<T> transition(T from, T to, Trigger when) {
      sm.transition(from, to, when);
      return this;
    }

    /**
     * Returns the transition value maintained by this Repulsor component.
     *
     * @param from value used by this operation.
     * @param to value used by this operation.
     * @param when value used by this operation.
     * @param onTransition value used by this operation.
     * @return state flow of t result for transition.
     */
    public StateFlow<T> transition(T from, T to, Trigger when, Runnable onTransition) {
      sm.transition(from, to, when, onTransition);
      return this;
    }

    /**
     * Returns the any value maintained by this Repulsor component.
     *
     * @param to value used by this operation.
     * @param when value used by this operation.
     * @return state flow of t result for any.
     */
    public StateFlow<T> any(T to, Trigger when) {
      sm.any(to, when);
      return this;
    }

    /**
     * Returns the any value maintained by this Repulsor component.
     *
     * @param to value used by this operation.
     * @param when value used by this operation.
     * @param onTransition value used by this operation.
     * @return state flow of t result for any.
     */
    public StateFlow<T> any(T to, Trigger when, Runnable onTransition) {
      sm.any(to, when, onTransition);
      return this;
    }

    /**
     * Returns the on enter value maintained by this Repulsor component.
     *
     * @param state value used by this operation.
     * @param cb value used by this operation.
     * @return state flow of t result for on enter.
     */
    public StateFlow<T> onEnter(T state, Runnable cb) {
      sm.onEnter(state, cb);
      return this;
    }

    /**
     * Returns the on exit value maintained by this Repulsor component.
     *
     * @param state value used by this operation.
     * @param cb value used by this operation.
     * @return state flow of t result for on exit.
     */
    public StateFlow<T> onExit(T state, Runnable cb) {
      sm.onExit(state, cb);
      return this;
    }

    /**
     * Returns the bind condition value maintained by this Repulsor component.
     *
     * @param edge value used by this operation.
     * @param onlyInPhase value used by this operation.
     * @param sink value used by this operation.
     * @return state flow of t result for bind condition.
     */
    public StateFlow<T> bindCondition(Trigger edge, T onlyInPhase, Consumer<Trigger> sink) {
      bindings.add(new BoundCondition<>(edge, onlyInPhase, sink));
      return this;
    }

    /** Runs wire in the Repulsor runtime. */
    public void wire() {
      boolean decided = false;
      for (var e : initial) {
        if (e.getKey().getAsBoolean()) {
          gate.setPhase(e.getValue());
          decided = true;
          break;
        }
      }
      if (!decided && initialDefault != null) gate.setPhase(initialDefault);
      for (var b : bindings) {
        b.sink.accept(gate.whenInPhase(b.src, b.phase));
      }
      sm.wire();
    }
  }

  /**
   * Returns the flow value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @return state flow of t result for flow.
   */
  public static <T> StateFlow<T> flow(PhaseGate<T> gate) {
    return new StateFlow<>(gate);
  }

  /**
   * Provides parallel gate functionality for the Repulsor WPILib command and Trigger integration
   * layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static final class ParallelGate<E extends Enum<E>> {
    private final EnumSet<E> active;
    private final Class<E> enumClass;

    /**
     * Returns the parallel gate value maintained by this Repulsor component.
     *
     * @param enumClass value used by this operation.
     * @param initial value used by this operation.
     */
    public ParallelGate(Class<E> enumClass, EnumSet<E> initial) {
      this.enumClass = Objects.requireNonNull(enumClass);
      this.active = initial.clone();
    }

    /**
     * Returns the is on value maintained by this Repulsor component.
     *
     * @param tag value used by this operation.
     * @return value produced by this operation.
     */
    public boolean isOn(E tag) {
      return active.contains(tag);
    }

    /**
     * Returns the snapshot value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public Set<E> snapshot() {
      return Collections.unmodifiableSet(active.clone());
    }

    /**
     * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
     * state, NetworkTables output, planner caches, or command-side runtime state depending on the
     * owning type.
     *
     * @param tag value used by this operation.
     * @param on value used by this operation.
     */
    public void set(E tag, boolean on) {
      if (on) active.add(tag);
      else active.remove(tag);
    }

    @SafeVarargs
    /**
     * Runs add in the Repulsor runtime.
     *
     * @param tags value used by this operation.
     */
    public final void add(E... tags) {
      for (E t : tags) active.add(t);
    }

    @SafeVarargs
    /**
     * Runs remove in the Repulsor runtime.
     *
     * @param tags value used by this operation.
     */
    public final void remove(E... tags) {
      for (E t : tags) active.remove(t);
    }

    /**
     * Updates set only state or telemetry as part of the Repulsor runtime loop. This may mutate
     * local state, NetworkTables output, planner caches, or command-side runtime state depending on
     * the owning type.
     *
     * @param tag value used by this operation.
     */
    public void setOnly(E tag) {
      active.clear();
      active.add(tag);
    }

    /**
     * Returns the when value maintained by this Repulsor component.
     *
     * @param tag value used by this operation.
     * @return value produced by this operation.
     */
    public Trigger when(E tag) {
      return new Trigger(() -> active.contains(tag));
    }

    @SafeVarargs
    /**
     * Returns the when all value maintained by this Repulsor component.
     *
     * @param req value used by this operation.
     * @return value produced by this operation.
     */
    public final Trigger whenAll(E... req) {
      return new Trigger(
          () -> {
            for (E t : req) if (!active.contains(t)) return false;
            return true;
          });
    }

    @SafeVarargs
    /**
     * Returns the when none value maintained by this Repulsor component.
     *
     * @param forb value used by this operation.
     * @return value produced by this operation.
     */
    public final Trigger whenNone(E... forb) {
      return new Trigger(
          () -> {
            for (E t : forb) if (active.contains(t)) return false;
            return true;
          });
    }

    /**
     * Returns the on enter value maintained by this Repulsor component.
     *
     * @param tag value used by this operation.
     * @return value produced by this operation.
     */
    public Trigger onEnter(E tag) {
      final boolean[] last = {active.contains(tag)};
      return new Trigger(
          () -> {
            boolean now = active.contains(tag);
            boolean enter = !last[0] && now;
            last[0] = now;
            return enter;
          });
    }

    /**
     * Returns the on exit value maintained by this Repulsor component.
     *
     * @param tag value used by this operation.
     * @return value produced by this operation.
     */
    public Trigger onExit(E tag) {
      final boolean[] last = {active.contains(tag)};
      return new Trigger(
          () -> {
            boolean now = active.contains(tag);
            boolean exit = last[0] && !now;
            last[0] = now;
            return exit;
          });
    }

    /**
     * Runs transition on in the Repulsor runtime.
     *
     * @param when value used by this operation.
     * @param add value used by this operation.
     * @param remove value used by this operation.
     */
    public void transitionOn(Trigger when, EnumSet<E> add, EnumSet<E> remove) {
      when.onTrue(
          Commands.runOnce(
                  () -> {
                    active.removeAll(remove);
                    active.addAll(add);
                  })
              .ignoringDisable(true));
    }

    /**
     * Returns the cls value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    Class<E> cls() {
      return enumClass;
    }
  }

  /**
   * Returns the local parallel gate value maintained by this Repulsor component.
   *
   * @param enumClass value used by this operation.
   * @param initial value used by this operation.
   * @return parallel gate of e result for local parallel gate.
   */
  public static <E extends Enum<E>> ParallelGate<E> localParallelGate(
      Class<E> enumClass, EnumSet<E> initial) {
    return new ParallelGate<>(enumClass, initial);
  }

  @SafeVarargs
  /**
   * Updates set of state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param cls value used by this operation.
   * @param tags value used by this operation.
   * @return enum set of e result for set of.
   */
  public static <E extends Enum<E>> EnumSet<E> setOf(Class<E> cls, E... tags) {
    EnumSet<E> s = EnumSet.noneOf(cls);
    Collections.addAll(s, tags);
    return s;
  }

  /**
   * Returns the tag rise after seen false value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param tag value used by this operation.
   * @param level value used by this operation.
   * @param debounceSec time value in seconds.
   * @return value produced by this operation.
   */
  public static <E extends Enum<E>> Trigger tagRiseAfterSeenFalse(
      ParallelGate<E> gate, E tag, Trigger level, double debounceSec) {
    final boolean[] seenFalse = {false};
    return rising(
        new Trigger(
            () -> {
              if (!gate.isOn(tag)) {
                seenFalse[0] = false;
                return false;
              }
              boolean v = level.getAsBoolean();
              if (!seenFalse[0] && !v) seenFalse[0] = true;
              return seenFalse[0] && v;
            }),
        debounceSec);
  }

  /**
   * Returns the tag rise after seen false value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param tag value used by this operation.
   * @param level value used by this operation.
   * @return value produced by this operation.
   */
  public static <E extends Enum<E>> Trigger tagRiseAfterSeenFalse(
      ParallelGate<E> gate, E tag, Trigger level) {
    return tagRiseAfterSeenFalse(gate, tag, level, DEFAULT_DEBOUNCE);
  }

  /**
   * Returns the tag fall after seen true value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param tag value used by this operation.
   * @param level value used by this operation.
   * @param debounceSec time value in seconds.
   * @return value produced by this operation.
   */
  public static <E extends Enum<E>> Trigger tagFallAfterSeenTrue(
      ParallelGate<E> gate, E tag, Trigger level, double debounceSec) {
    final boolean[] seenTrue = {false};
    return rising(
        new Trigger(
            () -> {
              if (!gate.isOn(tag)) {
                seenTrue[0] = false;
                return false;
              }
              boolean v = level.getAsBoolean();
              if (!seenTrue[0] && v) seenTrue[0] = true;
              return seenTrue[0] && !v;
            }),
        debounceSec);
  }

  /**
   * Returns the tag fall after seen true value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param tag value used by this operation.
   * @param level value used by this operation.
   * @return value produced by this operation.
   */
  public static <E extends Enum<E>> Trigger tagFallAfterSeenTrue(
      ParallelGate<E> gate, E tag, Trigger level) {
    return tagFallAfterSeenTrue(gate, tag, level, DEFAULT_DEBOUNCE);
  }

  /**
   * Provides flow parallel functionality for the Repulsor WPILib command and Trigger integration
   * layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static final class FlowParallel<E extends Enum<E>> {
    private final ParallelGate<E> gate;
    private final Class<E> cls;

    private static final class Rule<E extends Enum<E>> {
      final Trigger when;
      final EnumSet<E> add;
      final EnumSet<E> remove;
      final Runnable onTransition;

      Rule(Trigger w, EnumSet<E> a, EnumSet<E> r, Runnable cb) {
        when = w;
        add = a;
        remove = r;
        onTransition = cb;
      }
    }

    private final List<Rule<E>> rules = new ArrayList<>();

    private static final class Binding<E extends Enum<E>> {
      final Trigger src;
      final E[] required;
      final Consumer<Trigger> sink;

      Binding(Trigger s, E[] req, Consumer<Trigger> k) {
        src = s;
        required = req;
        sink = k;
      }
    }

    private final List<Binding<E>> bindings = new ArrayList<>();

    /**
     * Returns the flow parallel value maintained by this Repulsor component.
     *
     * @param gate value used by this operation.
     * @param cls value used by this operation.
     */
    public FlowParallel(ParallelGate<E> gate, Class<E> cls) {
      this.gate = gate;
      this.cls = cls;
    }

    /**
     * Returns the transition value maintained by this Repulsor component.
     *
     * @param when value used by this operation.
     * @param add value used by this operation.
     * @param remove value used by this operation.
     * @return flow parallel of e result for transition.
     */
    public FlowParallel<E> transition(Trigger when, EnumSet<E> add, EnumSet<E> remove) {
      rules.add(new Rule<>(when, add, remove, null));
      return this;
    }

    /**
     * Returns the transition value maintained by this Repulsor component.
     *
     * @param when value used by this operation.
     * @param add value used by this operation.
     * @param remove value used by this operation.
     * @param cb value used by this operation.
     * @return flow parallel of e result for transition.
     */
    public FlowParallel<E> transition(
        Trigger when, EnumSet<E> add, EnumSet<E> remove, Runnable cb) {
      rules.add(new Rule<>(when, add, remove, cb));
      return this;
    }

    @SafeVarargs
    /**
     * Returns the add on value maintained by this Repulsor component.
     *
     * @param when value used by this operation.
     * @param add value used by this operation.
     * @return flow parallel of e result for add on.
     */
    public final FlowParallel<E> addOn(Trigger when, E... add) {
      return transition(when, setOf(cls, add), setOf(cls));
    }

    @SafeVarargs
    /**
     * Returns the remove on value maintained by this Repulsor component.
     *
     * @param when value used by this operation.
     * @param remove value used by this operation.
     * @return flow parallel of e result for remove on.
     */
    public final FlowParallel<E> removeOn(Trigger when, E... remove) {
      return transition(when, setOf(cls), setOf(cls, remove));
    }

    @SafeVarargs
    /**
     * Returns the change on value maintained by this Repulsor component.
     *
     * @param when value used by this operation.
     * @param cb value used by this operation.
     * @param add value used by this operation.
     * @param remove value used by this operation.
     * @return flow parallel of e result for change on.
     */
    public final FlowParallel<E> changeOn(Trigger when, Runnable cb, E[] add, E... remove) {
      return transition(when, setOf(cls, add), setOf(cls, remove), cb);
    }

    /**
     * Returns the when value maintained by this Repulsor component.
     *
     * @param edge value used by this operation.
     * @return rule builder result for when.
     */
    public RuleBuilder when(Trigger edge) {
      return new RuleBuilder(edge);
    }

    /**
     * Provides rule builder functionality for the Repulsor WPILib command and Trigger integration
     * layer. Use this type from robot code, field profiles, or tests when integrating the
     * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
     * robot-relative motion.
     */
    public final class RuleBuilder {
      private final Trigger edge;
      private final EnumSet<E> add = EnumSet.noneOf(cls);
      private final EnumSet<E> remove = EnumSet.noneOf(cls);
      private Runnable cb;

      private RuleBuilder(Trigger e) {
        this.edge = e;
      }

      @SafeVarargs
      public final RuleBuilder add(E... tags) {
        Collections.addAll(add, tags);
        return this;
      }

      @SafeVarargs
      public final RuleBuilder remove(E... tags) {
        Collections.addAll(remove, tags);
        return this;
      }

      public RuleBuilder doRun(Runnable r) {
        this.cb = r;
        return this;
      }

      public FlowParallel<E> commit() {
        rules.add(new Rule<>(edge, add, remove, cb));
        return FlowParallel.this;
      }
    }

    @SafeVarargs
    /**
     * Returns the bind value maintained by this Repulsor component.
     *
     * @param edge value used by this operation.
     * @param sink value used by this operation.
     * @param requiredTags value used by this operation.
     * @return flow parallel of e result for bind.
     */
    public final FlowParallel<E> bind(Trigger edge, Consumer<Trigger> sink, E... requiredTags) {
      bindings.add(new Binding<>(edge, requiredTags, sink));
      return this;
    }

    @SuppressWarnings("unchecked")
    /** Runs wire in the Repulsor runtime. */
    public void wire() {
      for (Rule<E> r : rules) {
        r.when.onTrue(
            Commands.runOnce(
                    () -> {
                      gate.remove(
                          r.remove.toArray((E[]) java.lang.reflect.Array.newInstance(cls, 0)));
                      gate.add(r.add.toArray((E[]) java.lang.reflect.Array.newInstance(cls, 0)));
                      if (r.onTransition != null) r.onTransition.run();
                    })
                .ignoringDisable(true));
      }
      for (Binding<E> b : bindings) {
        Trigger gated =
            new Trigger(
                () -> {
                  for (E t : b.required) if (!gate.isOn(t)) return false;
                  return b.src.getAsBoolean();
                });
        b.sink.accept(gated);
      }
    }

    /**
     * Returns the merge value maintained by this Repulsor component.
     *
     * @param other value used by this operation.
     * @return flow parallel of e result for merge.
     */
    public FlowParallel<E> merge(FlowParallel<E> other) {
      if (this.gate != other.gate)
        throw new IllegalArgumentException("FlowParallel.merge requires the SAME gate instance");
      this.rules.addAll(other.rules);
      this.bindings.addAll(other.bindings);
      return this;
    }
  }

  /**
   * Returns the flow value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @param cls value used by this operation.
   * @return flow parallel of e result for flow.
   */
  public static <E extends Enum<E>> FlowParallel<E> flow(ParallelGate<E> gate, Class<E> cls) {
    return new FlowParallel<>(gate, cls);
  }

  /**
   * Returns the flow value maintained by this Repulsor component.
   *
   * @param gate value used by this operation.
   * @return flow parallel of e result for flow.
   */
  public static <E extends Enum<E>> FlowParallel<E> flow(ParallelGate<E> gate) {
    return new FlowParallel<>(gate, gate.cls());
  }

  @SafeVarargs
  /**
   * Returns the local parallel gate value maintained by this Repulsor component.
   *
   * @param first value used by this operation.
   * @param rest value used by this operation.
   * @return parallel gate of e result for local parallel gate.
   */
  public static <E extends Enum<E>> ParallelGate<E> localParallelGate(E first, E... rest) {
    Class<E> cls = first.getDeclaringClass();
    EnumSet<E> init = EnumSet.noneOf(cls);
    init.add(first);
    if (rest != null) Collections.addAll(init, rest);
    return new ParallelGate<>(cls, init);
  }

  @SafeVarargs
  /**
   * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param first value used by this operation.
   * @param rest value used by this operation.
   * @return enum set of e result for set.
   */
  public static <E extends Enum<E>> EnumSet<E> set(E first, E... rest) {
    Class<E> cls = first.getDeclaringClass();
    EnumSet<E> s = EnumSet.noneOf(cls);
    s.add(first);
    if (rest != null) Collections.addAll(s, rest);
    return s;
  }
}
