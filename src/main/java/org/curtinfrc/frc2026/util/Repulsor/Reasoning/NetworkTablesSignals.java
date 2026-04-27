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

import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;

/**
 * Provides network tables signals functionality for the Repulsor rule-based strategy and signal
 * reasoning layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class NetworkTablesSignals implements Signals, AutoCloseable {
  private final NetworkTableInstance inst;
  private final String basePath;
  private final Map<SignalKey<?>, NetworkTablesValue<?>> values = new HashMap<>();
  private final Map<SignalKey<?>, Object> initial = new HashMap<>();

  /**
   * Returns the network tables signals value maintained by this Repulsor component.
   *
   * @param inst value used by this operation.
   * @param basePath value used by this operation.
   */
  public NetworkTablesSignals(NetworkTableInstance inst, String basePath) {
    this.inst = Objects.requireNonNull(inst, "inst");
    if (basePath == null || basePath.isEmpty()) throw new IllegalArgumentException("basePath");
    this.basePath = normalizeBase(basePath);
  }

  /**
   * Updates register state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param key distance or field-coordinate value in meters.
   * @param initialValue value used by this operation.
   * @return network tables signals result for register.
   */
  public <T> NetworkTablesSignals register(SignalKey<T> key, T initialValue) {
    Objects.requireNonNull(key, "key");
    if (initialValue != null && !key.type().isInstance(initialValue)) {
      throw new IllegalArgumentException(
          "Initial for "
              + key
              + " must be "
              + key.type().getName()
              + ", got "
              + initialValue.getClass().getName());
    }
    initial.put(key, copyIfArrayObj(initialValue));
    return this;
  }

  /**
   * Runs put in the Repulsor runtime.
   *
   * @param key distance or field-coordinate value in meters.
   * @param value value used by this operation.
   */
  @Override
  public <T> void put(SignalKey<T> key, T value) {
    NetworkTablesValue<T> v = ensure(key);
    v.set(copyIfArray(value));
  }

  /**
   * Returns the latest value maintained by this Repulsor component.
   *
   * @param key distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  @Override
  public <T> Optional<T> get(SignalKey<T> key) {
    NetworkTablesValue<T> v = ensure(key);
    return Optional.ofNullable(v.get());
  }

  /**
   * Returns the get or value maintained by this Repulsor component.
   *
   * @param key distance or field-coordinate value in meters.
   * @param fallback value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public <T> T getOr(SignalKey<T> key, T fallback) {
    return get(key).orElse(fallback);
  }

  /**
   * Returns the has value maintained by this Repulsor component.
   *
   * @param key distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  @Override
  public boolean has(SignalKey<?> key) {
    Objects.requireNonNull(key, "key");
    ensureAny(key);
    return true;
  }

  /**
   * Updates clear state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  @Override
  public void clear() {
    for (Map.Entry<SignalKey<?>, NetworkTablesValue<?>> e : values.entrySet()) {
      SignalKey<?> key = e.getKey();
      NetworkTablesValue<?> v = e.getValue();
      Object init = initial.get(key);
      if (init == null) init = defaultValueFor(key);
      setTyped(key, v, init);
    }
  }

  /** Runs flush in the Repulsor runtime. */
  @Override
  public void flush() {
    for (NetworkTablesValue<?> v : values.values()) v.flush();
  }

  /** Runs close in the Repulsor runtime. */
  @Override
  public void close() {
    for (NetworkTablesValue<?> v : values.values()) v.close();
    values.clear();
    initial.clear();
  }

  private static String normalizeBase(String basePath) {
    String p = basePath.trim();
    if (!p.startsWith("/")) p = "/" + p;
    if (p.endsWith("/")) p = p.substring(0, p.length() - 1);
    return p;
  }

  private String topicFor(SignalKey<?> key) {
    return basePath + "/" + key.name();
  }

  @SuppressWarnings("unchecked")
  private <T> NetworkTablesValue<T> ensure(SignalKey<T> key) {
    return (NetworkTablesValue<T>) ensureAny(key);
  }

  private NetworkTablesValue<?> ensureAny(SignalKey<?> key) {
    NetworkTablesValue<?> existing = values.get(key);
    if (existing != null) return existing;

    String topic = topicFor(key);
    Object init = initial.get(key);
    if (init == null) init = defaultValueFor(key);

    NetworkTablesValue<?> created = createValue(key, topic, init);
    values.put(key, created);
    return created;
  }

  private static Object defaultValueFor(SignalKey<?> key) {
    Class<?> t = key.type();
    if (t == Boolean.class) return false;
    if (t == Double.class) return 0.0;
    if (t == Long.class) return 0L;
    if (t == Integer.class) return 0L;
    if (t == String.class) return "";
    if (t == double[].class) return new double[0];
    throw new IllegalArgumentException("Unsupported key type: " + key);
  }

  private NetworkTablesValue<?> createValue(SignalKey<?> key, String topic, Object init) {
    Class<?> t = key.type();
    if (t == Boolean.class) {
      return NetworkTablesValue.ofBoolean(inst, topic, (Boolean) init);
    }
    if (t == Double.class) {
      return NetworkTablesValue.ofDouble(inst, topic, (Double) init);
    }
    if (t == Long.class) {
      return NetworkTablesValue.ofInteger(inst, topic, (Long) init);
    }
    if (t == Integer.class) {
      long v = init instanceof Number ? ((Number) init).longValue() : 0L;
      return NetworkTablesValue.ofInteger(inst, topic, v);
    }
    if (t == String.class) {
      return NetworkTablesValue.ofString(inst, topic, (String) init);
    }
    if (t == double[].class) {
      return NetworkTablesValue.ofDoubleArray(inst, topic, (double[]) init);
    }
    throw new IllegalArgumentException("Unsupported key type: " + key);
  }

  @SuppressWarnings("unchecked")
  private static void setTyped(SignalKey<?> key, NetworkTablesValue<?> v, Object value) {
    Class<?> t = key.type();
    if (t == Boolean.class) {
      ((NetworkTablesValue<Boolean>) v).set(value instanceof Boolean b ? b : false);
      return;
    }
    if (t == Double.class) {
      double d = value instanceof Number n ? n.doubleValue() : 0.0;
      ((NetworkTablesValue<Double>) v).set(d);
      return;
    }
    if (t == Long.class) {
      long l = value instanceof Number n ? n.longValue() : 0L;
      ((NetworkTablesValue<Long>) v).set(l);
      return;
    }
    if (t == Integer.class) {
      long l = value instanceof Number n ? n.longValue() : 0L;
      ((NetworkTablesValue<Long>) v).set(l);
      return;
    }
    if (t == String.class) {
      ((NetworkTablesValue<String>) v).set(value != null ? value.toString() : "");
      return;
    }
    if (t == double[].class) {
      double[] arr = value instanceof double[] a ? a : new double[0];
      ((NetworkTablesValue<double[]>) v).set(arr.clone());
      return;
    }
    throw new IllegalArgumentException("Unsupported key type: " + key);
  }

  @SuppressWarnings("unchecked")
  private static <T> T copyIfArray(T v) {
    if (v instanceof double[] arr) return (T) arr.clone();
    return v;
  }

  private static Object copyIfArrayObj(Object v) {
    if (v instanceof double[] arr) return arr.clone();
    return v;
  }
}
