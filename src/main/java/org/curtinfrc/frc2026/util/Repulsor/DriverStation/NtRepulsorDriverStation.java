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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;

/**
 * Provides nt repulsor driver station functionality for the Repulsor driver-station and
 * NetworkTables control surface. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public abstract class NtRepulsorDriverStation extends RepulsorDriverStation {
  protected final NetworkTableInstance inst;
  protected final String root;

  private final List<NetworkTablesValue<?>> owned = new ArrayList<>();

  private final Map<String, NetworkTablesValue<Boolean>> configBools = new HashMap<>();
  private final Map<String, NetworkTablesValue<Double>> configDoubles = new HashMap<>();
  private final Map<String, NetworkTablesValue<Long>> configInts = new HashMap<>();
  private final Map<String, NetworkTablesValue<String>> configStrings = new HashMap<>();
  private final Map<String, NetworkTablesValue<double[]>> configDoubleArrays = new HashMap<>();

  private final Map<String, PoseOverrideCommand> poseOverrideCommands = new HashMap<>();
  private final Map<String, PoseResetCommand> poseResetCommands = new HashMap<>();
  private final Map<String, GoalSetpointCommand> goalSetpointCommands = new HashMap<>();

  protected NtRepulsorDriverStation(NetworkTableInstance inst, String root) {
    this.inst = Objects.requireNonNull(inst, "inst");
    if (root == null || root.isEmpty()) throw new IllegalArgumentException("root");
    this.root = normalizeRoot(root);

    declareSharedConfig(new Schema(this));
    flushAll();
  }

  protected abstract void declareSharedConfig(Schema schema);

  protected final String configPath(String key) {
    return root + "/config/" + normalizeKey(key);
  }

  protected final String commandPath(String key) {
    return root + "/commands/" + normalizeKey(key);
  }

  /**
   * Returns the get config bool value maintained by this Repulsor component.
   *
   * @param key distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public final boolean getConfigBool(String key) {
    NetworkTablesValue<Boolean> v = require(configBools, key);
    return Boolean.TRUE.equals(v.get());
  }

  /**
   * Returns the get config bool value maintained by this Repulsor component, falling back to the
   * supplied default when the key was never declared in the schema.
   *
   * @param key distance or field-coordinate value in meters.
   * @param defaultValue value used by this operation.
   * @return value produced by this operation.
   */
  public final boolean getConfigBool(String key, boolean defaultValue) {
    NetworkTablesValue<Boolean> v = configBools.get(normalizeKey(key));
    return v == null ? defaultValue : Boolean.TRUE.equals(v.get());
  }

  /**
   * Updates set config bool state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param key distance or field-coordinate value in meters.
   * @param value value used by this operation.
   */
  public final void setConfigBool(String key, boolean value) {
    require(configBools, key).set(value);
  }

  /**
   * Returns the get config double value maintained by this Repulsor component.
   *
   * @param key distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public final double getConfigDouble(String key) {
    NetworkTablesValue<Double> v = require(configDoubles, key);
    Double d = v.get();
    return d != null ? d : 0.0;
  }

  /**
   * Returns the get config double value maintained by this Repulsor component, falling back to the
   * supplied default when the key was never declared in the schema.
   *
   * @param key distance or field-coordinate value in meters.
   * @param defaultValue value used by this operation.
   * @return value produced by this operation.
   */
  public final double getConfigDouble(String key, double defaultValue) {
    NetworkTablesValue<Double> v = configDoubles.get(normalizeKey(key));
    Double d = v == null ? null : v.get();
    return d != null ? d : defaultValue;
  }

  /**
   * Updates set config double state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param key distance or field-coordinate value in meters.
   * @param value value used by this operation.
   */
  public final void setConfigDouble(String key, double value) {
    require(configDoubles, key).set(value);
  }

  /**
   * Returns the get config int value maintained by this Repulsor component.
   *
   * @param key distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public final long getConfigInt(String key) {
    NetworkTablesValue<Long> v = require(configInts, key);
    Long d = v.get();
    return d != null ? d : 0L;
  }

  /**
   * Updates set config int state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param key distance or field-coordinate value in meters.
   * @param value value used by this operation.
   */
  public final void setConfigInt(String key, long value) {
    require(configInts, key).set(value);
  }

  /**
   * Returns the get config string value maintained by this Repulsor component.
   *
   * @param key distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public final String getConfigString(String key) {
    NetworkTablesValue<String> v = require(configStrings, key);
    String s = v.get();
    return s != null ? s : "";
  }

  /**
   * Updates set config string state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param key distance or field-coordinate value in meters.
   * @param value value used by this operation.
   */
  public final void setConfigString(String key, String value) {
    require(configStrings, key).set(value != null ? value : "");
  }

  /**
   * Returns the get config double array value maintained by this Repulsor component.
   *
   * @param key distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public final double[] getConfigDoubleArray(String key) {
    NetworkTablesValue<double[]> v = require(configDoubleArrays, key);
    double[] a = v.get();
    return a != null ? a : new double[0];
  }

  /**
   * Updates set config double array state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param key distance or field-coordinate value in meters.
   * @param value value used by this operation.
   */
  public final void setConfigDoubleArray(String key, double[] value) {
    require(configDoubleArrays, key).set(value != null ? value : new double[0]);
  }

  /**
   * Returns the consume pose override value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @return value produced by this operation.
   */
  public final Optional<Pose2d> consumePoseOverride(String name) {
    PoseOverrideCommand cmd = require(poseOverrideCommands, name);
    return cmd.consume();
  }

  /**
   * Runs request pose override in the Repulsor runtime.
   *
   * @param name value used by this operation.
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @param enabled value used by this operation.
   */
  public final void requestPoseOverride(String name, Pose2d pose, boolean enabled) {
    PoseOverrideCommand cmd = require(poseOverrideCommands, name);
    cmd.request(pose, enabled);
  }

  /**
   * Returns the consume pose reset value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @return value produced by this operation.
   */
  public final Optional<Pose2d> consumePoseReset(String name) {
    PoseResetCommand cmd = require(poseResetCommands, name);
    return cmd.consume();
  }

  /**
   * Runs request pose reset in the Repulsor runtime.
   *
   * @param name value used by this operation.
   * @param pose WPILib Pose2d in field-relative coordinates.
   */
  public final void requestPoseReset(String name, Pose2d pose) {
    PoseResetCommand cmd = require(poseResetCommands, name);
    cmd.request(pose);
  }

  /**
   * Returns the forced goal pose value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @return value produced by this operation.
   */
  public final Optional<Pose2d> forcedGoalPose(String name) {
    GoalSetpointCommand cmd = require(goalSetpointCommands, name);
    return cmd.forcedPose();
  }

  /**
   * Returns the consume goal setpoint applied value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @return optional goal setpoint produced by this operation.
   */
  public final Optional<GoalSetpoint> consumeGoalSetpointApplied(String name) {
    GoalSetpointCommand cmd = require(goalSetpointCommands, name);
    return cmd.consumeApplied();
  }

  /**
   * Runs request goal setpoint in the Repulsor runtime.
   *
   * @param name value used by this operation.
   * @param goalPose value used by this operation.
   * @param enabled value used by this operation.
   */
  public final void requestGoalSetpoint(String name, Pose2d goalPose, boolean enabled) {
    GoalSetpointCommand cmd = require(goalSetpointCommands, name);
    cmd.request(goalPose, enabled);
  }

  /** Runs tick in the Repulsor runtime. */
  @Override
  public final void tick() {
    for (PoseOverrideCommand c : poseOverrideCommands.values()) c.tick();
    for (PoseResetCommand c : poseResetCommands.values()) c.tick();
    for (GoalSetpointCommand c : goalSetpointCommands.values()) c.tick();
    onTick();
  }

  protected void onTick() {}

  /** Runs flush all in the Repulsor runtime. */
  public final void flushAll() {
    for (NetworkTablesValue<?> v : owned) v.flush();
  }

  /** Runs close in the Repulsor runtime. */
  @Override
  public void close() {
    for (NetworkTablesValue<?> v : owned) v.close();
    owned.clear();
    configBools.clear();
    configDoubles.clear();
    configInts.clear();
    configStrings.clear();
    configDoubleArrays.clear();
    poseOverrideCommands.clear();
    poseResetCommands.clear();
    goalSetpointCommands.clear();
  }

  /**
   * Provides goal setpoint functionality for the Repulsor driver-station and NetworkTables control
   * surface. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static final class GoalSetpoint {
    /**
     * Configuration value for pose. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final Pose2d pose;

    /**
     * Configuration value for enabled. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final boolean enabled;

    /**
     * Returns the goal setpoint value maintained by this Repulsor component.
     *
     * @param pose WPILib Pose2d in field-relative coordinates.
     * @param enabled value used by this operation.
     */
    public GoalSetpoint(Pose2d pose, boolean enabled) {
      this.pose = pose != null ? pose : new Pose2d();
      this.enabled = enabled;
    }
  }

  protected final class Schema {
    private final NtRepulsorDriverStation ds;

    private Schema(NtRepulsorDriverStation ds) {
      this.ds = ds;
    }

    /**
     * Runs config bool in the Repulsor runtime.
     *
     * @param key distance or field-coordinate value in meters.
     * @param initialValue value used by this operation.
     */
    public void configBool(String key, boolean initialValue) {
      String k = normalizeKey(key);
      putUnique(
          configBools, k, own(NetworkTablesValue.ofBoolean(inst, configPath(k), initialValue)));
    }

    /**
     * Runs config double in the Repulsor runtime.
     *
     * @param key distance or field-coordinate value in meters.
     * @param initialValue value used by this operation.
     */
    public void configDouble(String key, double initialValue) {
      String k = normalizeKey(key);
      putUnique(
          configDoubles, k, own(NetworkTablesValue.ofDouble(inst, configPath(k), initialValue)));
    }

    /**
     * Runs config int in the Repulsor runtime.
     *
     * @param key distance or field-coordinate value in meters.
     * @param initialValue value used by this operation.
     */
    public void configInt(String key, long initialValue) {
      String k = normalizeKey(key);
      putUnique(
          configInts, k, own(NetworkTablesValue.ofInteger(inst, configPath(k), initialValue)));
    }

    /**
     * Runs config string in the Repulsor runtime.
     *
     * @param key distance or field-coordinate value in meters.
     * @param initialValue value used by this operation.
     */
    public void configString(String key, String initialValue) {
      String k = normalizeKey(key);
      putUnique(
          configStrings, k, own(NetworkTablesValue.ofString(inst, configPath(k), initialValue)));
    }

    /**
     * Runs config double array in the Repulsor runtime.
     *
     * @param key distance or field-coordinate value in meters.
     * @param initialValue value used by this operation.
     */
    public void configDoubleArray(String key, double[] initialValue) {
      String k = normalizeKey(key);
      putUnique(
          configDoubleArrays,
          k,
          own(NetworkTablesValue.ofDoubleArray(inst, configPath(k), initialValue)));
    }

    /**
     * Runs pose override command in the Repulsor runtime.
     *
     * @param name value used by this operation.
     * @param initialPose value used by this operation.
     * @param initialEnabled value used by this operation.
     */
    public void poseOverrideCommand(String name, Pose2d initialPose, boolean initialEnabled) {
      String k = normalizeKey(name);
      putUnique(
          poseOverrideCommands, k, new PoseOverrideCommand(ds, k, initialPose, initialEnabled));
    }

    /**
     * Runs pose reset command in the Repulsor runtime.
     *
     * @param name value used by this operation.
     * @param initialPose value used by this operation.
     */
    public void poseResetCommand(String name, Pose2d initialPose) {
      String k = normalizeKey(name);
      putUnique(poseResetCommands, k, new PoseResetCommand(ds, k, initialPose));
    }

    /**
     * Runs goal setpoint command in the Repulsor runtime.
     *
     * @param name value used by this operation.
     * @param initialPose value used by this operation.
     * @param initialEnabled value used by this operation.
     */
    public void goalSetpointCommand(String name, Pose2d initialPose, boolean initialEnabled) {
      String k = normalizeKey(name);
      putUnique(
          goalSetpointCommands, k, new GoalSetpointCommand(ds, k, initialPose, initialEnabled));
    }
  }

  private static String normalizeRoot(String root) {
    String r = root.trim();
    if (!r.startsWith("/")) r = "/" + r;
    while (r.endsWith("/")) r = r.substring(0, r.length() - 1);
    return r;
  }

  private static String normalizeKey(String key) {
    if (key == null) throw new IllegalArgumentException("key");
    String k = key.trim();
    if (k.isEmpty()) throw new IllegalArgumentException("key");
    while (k.startsWith("/")) k = k.substring(1);
    while (k.endsWith("/")) k = k.substring(0, k.length() - 1);
    if (k.isEmpty()) throw new IllegalArgumentException("key");
    return k;
  }

  private <T> NetworkTablesValue<T> own(NetworkTablesValue<T> v) {
    owned.add(v);
    return v;
  }

  private static <T> void putUnique(Map<String, T> map, String key, T value) {
    if (map.containsKey(key)) throw new IllegalStateException("Duplicate key: " + key);
    map.put(key, value);
  }

  private static <V> V require(Map<String, V> map, String key) {
    String k = normalizeKey(key);
    V v = map.get(k);
    if (v == null) throw new IllegalStateException("Unknown key: " + k);
    return v;
  }

  private static final class PoseOverrideCommand {
    private final NetworkTablesValue<Boolean> apply;
    private final NetworkTablesValue<Boolean> enabled;
    private final NetworkTablesValue<double[]> pose;

    private volatile Optional<Pose2d> pending = Optional.empty();

    private PoseOverrideCommand(
        NtRepulsorDriverStation ds, String name, Pose2d initialPose, boolean initialEnabled) {
      String base = ds.commandPath(name + "/pose_override");
      this.apply = ds.own(NetworkTablesValue.ofBoolean(ds.inst, base + "/apply", false));
      this.enabled =
          ds.own(NetworkTablesValue.ofBoolean(ds.inst, base + "/enabled", initialEnabled));
      this.pose =
          ds.own(
              NetworkTablesValue.ofDoubleArray(
                  ds.inst, base + "/pose", PoseCodec.encode(initialPose)));
    }

    private void tick() {
      if (Boolean.TRUE.equals(apply.get())) {
        apply.set(false);
        apply.flush();
        if (Boolean.TRUE.equals(enabled.get())) {
          pending = Optional.of(PoseCodec.decode(pose.get()));
        }
      }
    }

    private void request(Pose2d p, boolean en) {
      pose.set(PoseCodec.encode(p));
      enabled.set(en);
      apply.set(true);
      apply.flush();
    }

    private Optional<Pose2d> consume() {
      Optional<Pose2d> out = pending;
      pending = Optional.empty();
      return out;
    }
  }

  private static final class PoseResetCommand {
    private final NetworkTablesValue<Boolean> apply;
    private final NetworkTablesValue<double[]> pose;

    private volatile Optional<Pose2d> pending = Optional.empty();

    private PoseResetCommand(NtRepulsorDriverStation ds, String name, Pose2d initialPose) {
      String base = ds.commandPath(name + "/pose_reset");
      this.apply = ds.own(NetworkTablesValue.ofBoolean(ds.inst, base + "/apply", false));
      this.pose =
          ds.own(
              NetworkTablesValue.ofDoubleArray(
                  ds.inst, base + "/pose", PoseCodec.encode(initialPose)));
    }

    private void tick() {
      if (Boolean.TRUE.equals(apply.get())) {
        apply.set(false);
        apply.flush();
        pending = Optional.of(PoseCodec.decode(pose.get()));
      }
    }

    private void request(Pose2d p) {
      pose.set(PoseCodec.encode(p));
      apply.set(true);
      apply.flush();
    }

    private Optional<Pose2d> consume() {
      Optional<Pose2d> out = pending;
      pending = Optional.empty();
      return out;
    }
  }

  private static final class GoalSetpointCommand {
    private final NetworkTablesValue<Boolean> apply;
    private final NetworkTablesValue<Boolean> enabled;
    private final NetworkTablesValue<double[]> pose;

    private volatile Pose2d forcedPose = new Pose2d();
    private volatile boolean forcedEnabled;

    private volatile Optional<GoalSetpoint> appliedEdge = Optional.empty();

    private GoalSetpointCommand(
        NtRepulsorDriverStation ds, String name, Pose2d initialPose, boolean initialEnabled) {
      String base = ds.commandPath(name + "/goal_setpoint");
      this.apply = ds.own(NetworkTablesValue.ofBoolean(ds.inst, base + "/apply", false));
      this.enabled =
          ds.own(NetworkTablesValue.ofBoolean(ds.inst, base + "/enabled", initialEnabled));
      this.pose =
          ds.own(
              NetworkTablesValue.ofDoubleArray(
                  ds.inst, base + "/pose", PoseCodec.encode(initialPose)));

      this.forcedPose = initialPose != null ? initialPose : new Pose2d();
      this.forcedEnabled = initialEnabled;
    }

    private void tick() {
      if (Boolean.TRUE.equals(apply.get())) {
        apply.set(false);
        apply.flush();

        Pose2d p = PoseCodec.decode(pose.get());
        boolean en = Boolean.TRUE.equals(enabled.get());

        forcedPose = p;
        forcedEnabled = en;

        appliedEdge = Optional.of(new GoalSetpoint(p, en));
      }
    }

    private void request(Pose2d p, boolean en) {
      pose.set(PoseCodec.encode(p));
      enabled.set(en);
      apply.set(true);
      apply.flush();
    }

    private Optional<Pose2d> forcedPose() {
      return forcedEnabled ? Optional.of(forcedPose) : Optional.empty();
    }

    private Optional<GoalSetpoint> consumeApplied() {
      Optional<GoalSetpoint> out = appliedEdge;
      appliedEdge = Optional.empty();
      return out;
    }
  }

  private static final class PoseCodec {
    private static double[] encode(Pose2d pose) {
      Pose2d p = pose != null ? pose : new Pose2d();
      return new double[] {p.getX(), p.getY(), p.getRotation().getRadians()};
    }

    private static Pose2d decode(double[] arr) {
      if (arr == null || arr.length < 3) return new Pose2d();
      return new Pose2d(arr[0], arr[1], new edu.wpi.first.math.geometry.Rotation2d(arr[2]));
    }
  }
}
