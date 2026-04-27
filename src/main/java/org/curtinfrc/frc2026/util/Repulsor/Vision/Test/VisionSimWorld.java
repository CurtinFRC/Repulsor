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

package org.curtinfrc.frc2026.util.Repulsor.Vision.Test;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Random;

/**
 * Provides vision sim world functionality for the Repulsor simulation test harness layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public final class VisionSimWorld {
  /**
   * Configuration value for field length. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  public static final double FIELD_LENGTH = 16.0;

  /**
   * Configuration value for field width. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  public static final double FIELD_WIDTH = 8.0;

  /**
   * Configuration value for max speed. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double MAX_SPEED = 4.3;

  /**
   * Configuration value for max accel. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double MAX_ACCEL = 4.2;

  /**
   * Configuration value for ou tau mean. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double OU_TAU_MEAN = 1.1;

  /**
   * Configuration value for ou sigma mean. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double OU_SIGMA_MEAN = 1.7;

  /**
   * Configuration value for heading lerp. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double HEADING_LERP = 0.22;

  /**
   * Configuration value for wall margin. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  public static final double WALL_MARGIN = 0.55;

  /**
   * Configuration value for wall gain. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double WALL_GAIN = 2.6;

  /**
   * Configuration value for dt min. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double DT_MIN = 0.005;

  /**
   * Configuration value for dt max. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double DT_MAX = 0.04;

  private static volatile Pose2d selfPose = null;

  /**
   * Updates set self pose state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param p value used by this operation.
   */
  public static void setSelfPose(Pose2d p) {
    selfPose = p;
  }

  /**
   * Returns the get self pose value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static Pose2d getSelfPose() {
    return selfPose;
  }

  /**
   * Returns the clamp value maintained by this Repulsor component.
   *
   * @param v value used by this operation.
   * @param lo value used by this operation.
   * @param hi value used by this operation.
   * @return value produced by this operation.
   */
  public static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  /**
   * Returns the wrap pi value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @return value produced by this operation.
   */
  public static double wrapPi(double a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  /**
   * Returns the lerp angle value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param t value used by this operation.
   * @return value produced by this operation.
   */
  public static double lerpAngle(double a, double b, double t) {
    double d = wrapPi(b - a);
    return a + t * d;
  }

  /**
   * Provides ou functionality for the Repulsor simulation test harness layer. Use this type from
   * robot code, field profiles, or tests when integrating the corresponding Repulsor subsystem.
   * Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static class OU {
    private final Random rng;
    private final double tau;
    private final double sigma;
    private double ax;
    private double ay;

    /**
     * Returns the ou value maintained by this Repulsor component.
     *
     * @param rng value used by this operation.
     * @param tau value used by this operation.
     * @param sigma value used by this operation.
     */
    public OU(Random rng, double tau, double sigma) {
      this.rng = rng;
      this.tau = tau;
      this.sigma = sigma;
    }

    /**
     * Returns the step value maintained by this Repulsor component.
     *
     * @param dt value used by this operation.
     * @return value produced by this operation.
     */
    public double[] step(double dt) {
      double k = Math.exp(-dt / tau);
      double s = sigma * Math.sqrt(1 - k * k);
      ax = k * ax + s * rng.nextGaussian();
      ay = k * ay + s * rng.nextGaussian();
      return new double[] {ax, ay};
    }
  }

  /**
   * Provides agent functionality for the Repulsor simulation test harness layer. Use this type from
   * robot code, field profiles, or tests when integrating the corresponding Repulsor subsystem.
   * Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static class Agent {
    /**
     * Configuration value for heading. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public double x, y, vx, vy, heading;

    /**
     * Configuration value for ou. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final OU ou;

    /**
     * Configuration value for role. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final AgentRole role;

    /**
     * Configuration value for team. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public int team;

    /**
     * Configuration value for ry. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public double rx, ry;

    /**
     * Configuration value for max speed. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public double maxSpeed;

    /**
     * Configuration value for max accel. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public double maxAccel;

    /**
     * Returns the agent value maintained by this Repulsor component.
     *
     * @param rng value used by this operation.
     * @param role value used by this operation.
     * @param team value used by this operation.
     * @param rx distance or field-coordinate value in meters.
     * @param ry distance or field-coordinate value in meters.
     */
    public Agent(Random rng, AgentRole role, int team, double rx, double ry) {
      this.role = role;
      this.team = team;
      this.rx = rx;
      this.ry = ry;
      this.x = 0.8 + rng.nextDouble() * (FIELD_LENGTH - 1.6);
      this.y = 0.6 + rng.nextDouble() * (FIELD_WIDTH - 1.2);
      this.vx = (rng.nextDouble() - 0.5) * 0.8;
      this.vy = (rng.nextDouble() - 0.5) * 0.8;
      this.heading = Math.atan2(vy, vx);
      double tau = OU_TAU_MEAN * (0.75 + 0.5 * rng.nextDouble());
      double sig = OU_SIGMA_MEAN * (0.75 + 0.6 * rng.nextDouble());
      this.ou = new OU(rng, tau, sig);
      this.maxSpeed = MAX_SPEED;
      this.maxAccel = MAX_ACCEL;
    }

    /**
     * Returns the with caps value maintained by this Repulsor component.
     *
     * @param speed value used by this operation.
     * @param accel value used by this operation.
     * @return value produced by this operation.
     */
    public Agent withCaps(double speed, double accel) {
      this.maxSpeed = speed;
      this.maxAccel = accel;
      return this;
    }
  }
}
