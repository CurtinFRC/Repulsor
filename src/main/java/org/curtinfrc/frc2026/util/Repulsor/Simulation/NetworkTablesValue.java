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

package org.curtinfrc.frc2026.util.Repulsor.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import java.util.Objects;

/**
 * Provides network tables value functionality for the Repulsor mechanism and sensor simulation
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public final class NetworkTablesValue<T> {
  /**
   * Contract for codec implementations used by the Repulsor mechanism and sensor simulation layer.
   * Use this type from robot code, field profiles, or tests when integrating the corresponding
   * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
   * motion.
   */
  public interface Codec<T> {
    /**
     * Returns the type value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    Class<T> type();

    void publish(NetworkTableInstance inst, String topicName, T initialValue);

    void close();

    T get();

    void set(T value);

    void flush();
  }

  private final Codec<T> codec;

  /**
   * Returns the to advantage kit value maintained by this Repulsor component.
   *
   * @param topicName value used by this operation.
   * @return value produced by this operation.
   */
  public static String toAdvantageKit(String topicName) {
    return "AdvantageKit/RealOutputs" + (topicName.startsWith("/") ? "" : "/") + topicName;
  }

  /**
   * Returns the network tables value value maintained by this Repulsor component.
   *
   * @param codec value used by this operation.
   * @param inst value used by this operation.
   * @param topicName value used by this operation.
   * @param initialValue value used by this operation.
   */
  public NetworkTablesValue(
      Codec<T> codec, NetworkTableInstance inst, String topicName, T initialValue) {
    this.codec = Objects.requireNonNull(codec, "codec");
    Objects.requireNonNull(inst, "inst");
    if (topicName == null || topicName.isEmpty()) throw new IllegalArgumentException("topicName");
    codec.publish(inst, topicName, initialValue);
  }

  /**
   * Returns the latest value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public T get() {
    return codec.get();
  }

  /**
   * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param value value used by this operation.
   */
  public void set(T value) {
    codec.set(value);
  }

  /** Runs flush in the Repulsor runtime. */
  public void flush() {
    codec.flush();
  }

  /** Runs close in the Repulsor runtime. */
  public void close() {
    codec.close();
  }

  /**
   * Returns the of double value maintained by this Repulsor component.
   *
   * @param inst value used by this operation.
   * @param topicName value used by this operation.
   * @param initialValue value used by this operation.
   * @return network tables value of double result for of double.
   */
  public static NetworkTablesValue<Double> ofDouble(
      NetworkTableInstance inst, String topicName, double initialValue) {
    return new NetworkTablesValue<>(new DoubleCodec(), inst, topicName, initialValue);
  }

  /**
   * Returns the of integer value maintained by this Repulsor component.
   *
   * @param inst value used by this operation.
   * @param topicName value used by this operation.
   * @param initialValue value used by this operation.
   * @return network tables value of long result for of integer.
   */
  public static NetworkTablesValue<Long> ofInteger(
      NetworkTableInstance inst, String topicName, long initialValue) {
    return new NetworkTablesValue<>(new IntegerCodec(), inst, topicName, initialValue);
  }

  /**
   * Returns the of boolean value maintained by this Repulsor component.
   *
   * @param inst value used by this operation.
   * @param topicName value used by this operation.
   * @param initialValue value used by this operation.
   * @return network tables value of boolean result for of boolean.
   */
  public static NetworkTablesValue<Boolean> ofBoolean(
      NetworkTableInstance inst, String topicName, boolean initialValue) {
    return new NetworkTablesValue<>(new BooleanCodec(), inst, topicName, initialValue);
  }

  /**
   * Returns the of string value maintained by this Repulsor component.
   *
   * @param inst value used by this operation.
   * @param topicName value used by this operation.
   * @param initialValue value used by this operation.
   * @return network tables value of string result for of string.
   */
  public static NetworkTablesValue<String> ofString(
      NetworkTableInstance inst, String topicName, String initialValue) {
    return new NetworkTablesValue<>(new StringCodec(), inst, topicName, initialValue);
  }

  /**
   * Returns the of double array value maintained by this Repulsor component.
   *
   * @param inst value used by this operation.
   * @param topicName value used by this operation.
   * @param initialValue value used by this operation.
   * @return network tables value of double[] result for of double array.
   */
  public static NetworkTablesValue<double[]> ofDoubleArray(
      NetworkTableInstance inst, String topicName, double[] initialValue) {
    return new NetworkTablesValue<>(new DoubleArrayCodec(), inst, topicName, initialValue);
  }

  /**
   * Returns the of pose2d value maintained by this Repulsor component.
   *
   * @param inst value used by this operation.
   * @param topicName value used by this operation.
   * @param initialValue value used by this operation.
   * @return network tables value of pose2d result for of pose2d.
   */
  public static NetworkTablesValue<Pose2d> ofPose2d(
      NetworkTableInstance inst, String topicName, Pose2d initialValue) {
    return new NetworkTablesValue<>(new Pose2dCodec(), inst, topicName, initialValue);
  }

  /**
   * Returns the of translation2d value maintained by this Repulsor component.
   *
   * @param inst value used by this operation.
   * @param topicName value used by this operation.
   * @param initialValue value used by this operation.
   * @return network tables value of translation2d result for of translation2d.
   */
  public static NetworkTablesValue<Translation2d> ofTranslation2d(
      NetworkTableInstance inst, String topicName, Translation2d initialValue) {
    return new NetworkTablesValue<>(new Translation2dCodec(), inst, topicName, initialValue);
  }

  private static final class DoubleCodec implements Codec<Double> {
    private DoublePublisher pub;
    private DoubleSubscriber sub;

    /**
     * Returns the type value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Class<Double> type() {
      return Double.class;
    }

    /**
     * Updates publish state or telemetry as part of the Repulsor runtime loop. This may mutate
     * local state, NetworkTables output, planner caches, or command-side runtime state depending on
     * the owning type.
     *
     * @param inst value used by this operation.
     * @param topicName value used by this operation.
     * @param initialValue value used by this operation.
     */
    @Override
    public void publish(NetworkTableInstance inst, String topicName, Double initialValue) {
      DoubleTopic topic = inst.getDoubleTopic(topicName);
      pub = topic.publish();
      double init = initialValue != null ? initialValue : 0.0;
      sub = topic.subscribe(init);
      edu.wpi.first.networktables.TimestampedDouble existing = sub.getAtomic();
      if (existing == null || existing.timestamp == 0) {
        pub.set(init);
      }
    }

    /**
     * Returns the latest value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Double get() {
      return sub.get();
    }

    /**
     * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
     * state, NetworkTables output, planner caches, or command-side runtime state depending on the
     * owning type.
     *
     * @param value value used by this operation.
     */
    @Override
    public void set(Double value) {
      pub.set(value != null ? value : 0.0);
    }

    /** Runs flush in the Repulsor runtime. */
    @Override
    public void flush() {
      if (pub != null) pub.getTopic().getInstance().flush();
    }

    /** Runs close in the Repulsor runtime. */
    @Override
    public void close() {
      if (sub != null) sub.close();
      if (pub != null) pub.close();
      sub = null;
      pub = null;
    }
  }

  private static final class IntegerCodec implements Codec<Long> {
    private IntegerPublisher pub;
    private IntegerSubscriber sub;

    /**
     * Returns the type value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Class<Long> type() {
      return Long.class;
    }

    /**
     * Updates publish state or telemetry as part of the Repulsor runtime loop. This may mutate
     * local state, NetworkTables output, planner caches, or command-side runtime state depending on
     * the owning type.
     *
     * @param inst value used by this operation.
     * @param topicName value used by this operation.
     * @param initialValue value used by this operation.
     */
    @Override
    public void publish(NetworkTableInstance inst, String topicName, Long initialValue) {
      IntegerTopic topic = inst.getIntegerTopic(topicName);
      pub = topic.publish();
      long init = initialValue != null ? initialValue : 0L;
      sub = topic.subscribe(init);
      edu.wpi.first.networktables.TimestampedInteger existing = sub.getAtomic();
      if (existing == null || existing.timestamp == 0) {
        pub.set(init);
      }
    }

    /**
     * Returns the latest value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Long get() {
      return sub.get();
    }

    /**
     * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
     * state, NetworkTables output, planner caches, or command-side runtime state depending on the
     * owning type.
     *
     * @param value value used by this operation.
     */
    @Override
    public void set(Long value) {
      pub.set(value != null ? value : 0L);
    }

    /** Runs flush in the Repulsor runtime. */
    @Override
    public void flush() {
      if (pub != null) pub.getTopic().getInstance().flush();
    }

    /** Runs close in the Repulsor runtime. */
    @Override
    public void close() {
      if (sub != null) sub.close();
      if (pub != null) pub.close();
      sub = null;
      pub = null;
    }
  }

  private static final class BooleanCodec implements Codec<Boolean> {
    private BooleanPublisher pub;
    private BooleanSubscriber sub;

    /**
     * Returns the type value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Class<Boolean> type() {
      return Boolean.class;
    }

    /**
     * Updates publish state or telemetry as part of the Repulsor runtime loop. This may mutate
     * local state, NetworkTables output, planner caches, or command-side runtime state depending on
     * the owning type.
     *
     * @param inst value used by this operation.
     * @param topicName value used by this operation.
     * @param initialValue value used by this operation.
     */
    @Override
    public void publish(NetworkTableInstance inst, String topicName, Boolean initialValue) {
      BooleanTopic topic = inst.getBooleanTopic(topicName);
      pub = topic.publish();
      boolean init = initialValue != null ? initialValue : false;
      sub = topic.subscribe(init);
      edu.wpi.first.networktables.TimestampedBoolean existing = sub.getAtomic();
      if (existing == null || existing.timestamp == 0) {
        pub.set(init);
      }
    }

    /**
     * Returns the latest value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Boolean get() {
      return sub.get();
    }

    /**
     * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
     * state, NetworkTables output, planner caches, or command-side runtime state depending on the
     * owning type.
     *
     * @param value value used by this operation.
     */
    @Override
    public void set(Boolean value) {
      pub.set(value != null ? value : false);
    }

    /** Runs flush in the Repulsor runtime. */
    @Override
    public void flush() {
      if (pub != null) pub.getTopic().getInstance().flush();
    }

    /** Runs close in the Repulsor runtime. */
    @Override
    public void close() {
      if (sub != null) sub.close();
      if (pub != null) pub.close();
      sub = null;
      pub = null;
    }
  }

  private static final class StringCodec implements Codec<String> {
    private StringPublisher pub;
    private StringSubscriber sub;

    /**
     * Returns the type value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Class<String> type() {
      return String.class;
    }

    /**
     * Updates publish state or telemetry as part of the Repulsor runtime loop. This may mutate
     * local state, NetworkTables output, planner caches, or command-side runtime state depending on
     * the owning type.
     *
     * @param inst value used by this operation.
     * @param topicName value used by this operation.
     * @param initialValue value used by this operation.
     */
    @Override
    public void publish(NetworkTableInstance inst, String topicName, String initialValue) {
      StringTopic topic = inst.getStringTopic(topicName);
      pub = topic.publish();
      String init = initialValue != null ? initialValue : "";
      sub = topic.subscribe(init);
      edu.wpi.first.networktables.TimestampedString existing = sub.getAtomic();
      if (existing == null || existing.timestamp == 0) {
        pub.set(init);
      }
    }

    /**
     * Returns the latest value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public String get() {
      return sub.get();
    }

    /**
     * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
     * state, NetworkTables output, planner caches, or command-side runtime state depending on the
     * owning type.
     *
     * @param value value used by this operation.
     */
    @Override
    public void set(String value) {
      pub.set(value != null ? value : "");
    }

    /** Runs flush in the Repulsor runtime. */
    @Override
    public void flush() {
      if (pub != null) pub.getTopic().getInstance().flush();
    }

    /** Runs close in the Repulsor runtime. */
    @Override
    public void close() {
      if (sub != null) sub.close();
      if (pub != null) pub.close();
      sub = null;
      pub = null;
    }
  }

  private static final class Pose2dCodec implements Codec<Pose2d> {
    private StringPublisher x;
    private StringPublisher y;
    private StringPublisher theta;
    private StringSubscriber xSub;
    private StringSubscriber ySub;
    private StringSubscriber thetaSub;

    /** Returns the pose2d codec value maintained by this Repulsor component. */
    public Pose2dCodec() {
      // No-op
    }

    /**
     * Returns the type value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Class<Pose2d> type() {
      return Pose2d.class;
    }

    /**
     * Updates publish state or telemetry as part of the Repulsor runtime loop. This may mutate
     * local state, NetworkTables output, planner caches, or command-side runtime state depending on
     * the owning type.
     *
     * @param inst value used by this operation.
     * @param topicName value used by this operation.
     * @param initialValue value used by this operation.
     */
    @Override
    public void publish(NetworkTableInstance inst, String topicName, Pose2d initialValue) {
      x = inst.getStringTopic(topicName + "/x").publish();
      y = inst.getStringTopic(topicName + "/y").publish();
      theta = inst.getStringTopic(topicName + "/theta").publish();
      xSub = inst.getStringTopic(topicName + "/x").subscribe("0.0");
      ySub = inst.getStringTopic(topicName + "/y").subscribe("0.0");
      thetaSub = inst.getStringTopic(topicName + "/theta").subscribe("0.0");
      Pose2d init = initialValue != null ? initialValue : new Pose2d();
      set(init);
    }

    /**
     * Returns the latest value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Pose2d get() {
      double xVal = Double.parseDouble(xSub.get());
      double yVal = Double.parseDouble(ySub.get());
      double thetaVal = Double.parseDouble(thetaSub.get());
      return new Pose2d(xVal, yVal, new Rotation2d(thetaVal));
    }

    /**
     * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
     * state, NetworkTables output, planner caches, or command-side runtime state depending on the
     * owning type.
     *
     * @param value value used by this operation.
     */
    @Override
    public void set(Pose2d value) {
      Pose2d val = value != null ? value : new Pose2d();
      x.set(Double.toString(val.getX()));
      y.set(Double.toString(val.getY()));
      theta.set(Double.toString(val.getRotation().getRadians()));
    }

    /** Runs flush in the Repulsor runtime. */
    @Override
    public void flush() {
      if (x != null) x.getTopic().getInstance().flush();
      if (y != null) y.getTopic().getInstance().flush();
      if (theta != null) theta.getTopic().getInstance().flush();
    }

    /** Runs close in the Repulsor runtime. */
    @Override
    public void close() {
      if (xSub != null) xSub.close();
      if (ySub != null) ySub.close();
      if (thetaSub != null) thetaSub.close();
      if (x != null) x.close();
      if (y != null) y.close();
      if (theta != null) theta.close();
      xSub = null;
      ySub = null;
      thetaSub = null;
      x = null;
      y = null;
      theta = null;
    }
  }

  private static final class Translation2dCodec implements Codec<Translation2d> {
    private StringPublisher x;
    private StringPublisher y;
    private StringSubscriber xSub;
    private StringSubscriber ySub;

    /** Returns the translation2d codec value maintained by this Repulsor component. */
    public Translation2dCodec() {
      // No-op
    }

    /**
     * Returns the type value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Class<Translation2d> type() {
      return Translation2d.class;
    }

    /**
     * Updates publish state or telemetry as part of the Repulsor runtime loop. This may mutate
     * local state, NetworkTables output, planner caches, or command-side runtime state depending on
     * the owning type.
     *
     * @param inst value used by this operation.
     * @param topicName value used by this operation.
     * @param initialValue value used by this operation.
     */
    @Override
    public void publish(NetworkTableInstance inst, String topicName, Translation2d initialValue) {
      x = inst.getStringTopic(topicName + "/x").publish();
      y = inst.getStringTopic(topicName + "/y").publish();
      xSub = inst.getStringTopic(topicName + "/x").subscribe("0.0");
      ySub = inst.getStringTopic(topicName + "/y").subscribe("0.0");
      Translation2d init = initialValue != null ? initialValue : new Translation2d();
      set(init);
    }

    /**
     * Returns the latest value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Translation2d get() {
      double xVal = Double.parseDouble(xSub.get());
      double yVal = Double.parseDouble(ySub.get());
      return new Translation2d(xVal, yVal);
    }

    /**
     * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
     * state, NetworkTables output, planner caches, or command-side runtime state depending on the
     * owning type.
     *
     * @param value value used by this operation.
     */
    @Override
    public void set(Translation2d value) {
      Translation2d val = value != null ? value : new Translation2d();
      x.set(Double.toString(val.getX()));
      y.set(Double.toString(val.getY()));
    }

    /** Runs flush in the Repulsor runtime. */
    @Override
    public void flush() {
      if (x != null) x.getTopic().getInstance().flush();
      if (y != null) y.getTopic().getInstance().flush();
    }

    /** Runs close in the Repulsor runtime. */
    @Override
    public void close() {
      if (xSub != null) xSub.close();
      if (ySub != null) ySub.close();
      if (x != null) x.close();
      if (y != null) y.close();
      xSub = null;
      ySub = null;
      x = null;
      y = null;
    }
  }

  private static final class DoubleArrayCodec implements Codec<double[]> {
    private DoubleArrayPublisher pub;
    private DoubleArraySubscriber sub;

    /**
     * Returns the type value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public Class<double[]> type() {
      return double[].class;
    }

    /**
     * Updates publish state or telemetry as part of the Repulsor runtime loop. This may mutate
     * local state, NetworkTables output, planner caches, or command-side runtime state depending on
     * the owning type.
     *
     * @param inst value used by this operation.
     * @param topicName value used by this operation.
     * @param initialValue value used by this operation.
     */
    @Override
    public void publish(NetworkTableInstance inst, String topicName, double[] initialValue) {
      DoubleArrayTopic topic = inst.getDoubleArrayTopic(topicName);
      pub = topic.publish();
      double[] init = initialValue != null ? initialValue : new double[0];
      sub = topic.subscribe(init);
      edu.wpi.first.networktables.TimestampedDoubleArray existing = sub.getAtomic();
      if (existing == null || existing.timestamp == 0) {
        pub.set(init);
      }
    }

    /**
     * Returns the latest value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    @Override
    public double[] get() {
      return sub.get();
    }

    /**
     * Updates set state or telemetry as part of the Repulsor runtime loop. This may mutate local
     * state, NetworkTables output, planner caches, or command-side runtime state depending on the
     * owning type.
     *
     * @param value value used by this operation.
     */
    @Override
    public void set(double[] value) {
      pub.set(value != null ? value : new double[0]);
    }

    /** Runs flush in the Repulsor runtime. */
    @Override
    public void flush() {
      if (pub != null) pub.getTopic().getInstance().flush();
    }

    /** Runs close in the Repulsor runtime. */
    @Override
    public void close() {
      if (sub != null) sub.close();
      if (pub != null) pub.close();
      sub = null;
      pub = null;
    }
  }
}
