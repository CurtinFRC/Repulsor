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

package org.curtinfrc.frc2026.util.Repulsor.Tuning;

import edu.wpi.first.math.MathUtil;

/**
 * Provides default drive tuning functionality for the Repulsor drive and turn tuning model layer.
 * Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public class DefaultDriveTuning extends DriveTuning {
  private double maxSpeed = 5.14;
  private double sqrtScale = 6.0;
  private double minStep = 0.02;
  private double nearStart = 0.40;
  private double nearEnd = 0.02;
  private final double MAX_SPEED = 8;

  /** Returns the default drive tuning value maintained by this Repulsor component. */
  public DefaultDriveTuning() {
    super("Drive/Default");
  }

  /**
   * Returns the with max speed value maintained by this Repulsor component.
   *
   * @param mps value used by this operation.
   * @return default drive tuning result for with max speed.
   */
  public DefaultDriveTuning withMaxSpeed(double mps) {
    this.maxSpeed = mps;
    return this;
  }

  /**
   * Returns the with sqrt scale value maintained by this Repulsor component.
   *
   * @param s value used by this operation.
   * @return default drive tuning result for with sqrt scale.
   */
  public DefaultDriveTuning withSqrtScale(double s) {
    this.sqrtScale = s;
    return this;
  }

  /**
   * Returns the with min step value maintained by this Repulsor component.
   *
   * @param m value used by this operation.
   * @return default drive tuning result for with min step.
   */
  public DefaultDriveTuning withMinStep(double m) {
    this.minStep = m;
    return this;
  }

  /**
   * Returns the with near window value maintained by this Repulsor component.
   *
   * @param startM value used by this operation.
   * @param endM value used by this operation.
   * @return default drive tuning result for with near window.
   */
  public DefaultDriveTuning withNearWindow(double startM, double endM) {
    this.nearStart = startM;
    this.nearEnd = endM;
    return this;
  }

  /** Runs apply defaults in the Repulsor runtime. */
  @Override
  public void applyDefaults() {
    setDtSeconds(0.02);
  }

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  @Override
  public void reset() {}

  /**
   * Returns the max linear speed mps value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public double maxLinearSpeedMps() {
    return maxSpeed;
  }

  /**
   * Returns the min step meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public double minStepMeters() {
    return minStep;
  }

  // @Override public double baseStepMeters(double distanceMeters) { var speed = Math.min(8
  // /*5.14*/, Math.sqrt(24 * distanceMeters)); // Logger.recordOutput("Repulsor/Speed", speed);
  // return
  // speed * dtSeconds(); }
  /**
   * Returns the base step meters value maintained by this Repulsor component.
   *
   * @param distanceMeters distance or field-coordinate value in meters.
   * @param slowDown value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public double baseStepMeters(double distanceMeters, boolean slowDown) {
    double d = Math.max(0.0, distanceMeters);
    if (d <= 0.0) {
      // Logger.recordOutput("Repulsor/Speed", 0.0);
      // Logger.recordOutput("Repulsor/Remaining", 0.0);
      // Logger.recordOutput("Repulsor/Step", 0.0);
      return 0.0;
    }

    if (!slowDown) {
      return Math.min(maxSpeed * dtSeconds(), d);
    }

    double dt = dtSeconds();
    if (dt <= 0.0) {
      dt = 0.02;
    }

    double vMax = Math.min(maxSpeed, MAX_SPEED);
    double aMax = Math.max(0.01, sqrtScale);

    double dBrake = vMax * vMax / (2.0 * aMax);

    double v;
    if (d > dBrake) {
      v = vMax;
    } else {
      v = Math.sqrt(2.0 * aMax * d);
    }

    if (d < nearStart && nearStart > nearEnd) {
      double t = (d - nearEnd) / (nearStart - nearEnd);
      t = MathUtil.clamp(t, 0.0, 1.0);
      double s = t * t * (3.0 - 2.0 * t);
      v *= s;
    }

    double step = v * dt;

    if (step < minStep && d > minStep) {
      step = minStep;
    }

    if (step > d) {
      step = d;
    }

    // Logger.recordOutput("Repulsor/Speed", v);
    // Logger.recordOutput("Repulsor/Remaining", d);
    // Logger.recordOutput("Repulsor/Step", step);

    return step;
  }

  /**
   * Returns the near goal scale value maintained by this Repulsor component.
   *
   * @param distanceMeters distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  @Override
  public double nearGoalScale(double distanceMeters) {
    return 1.0;
  }

  /**
   * Returns the scale for turning value maintained by this Repulsor component.
   *
   * @param yawDeltaRad value used by this operation.
   * @param isScoring value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public double scaleForTurning(double yawDeltaRad, boolean isScoring) {
    return 1.0;
  }
}
