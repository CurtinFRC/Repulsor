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

/**
 * Provides drive tuning functionality for the Repulsor drive and turn tuning model layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public abstract class DriveTuning extends Tuning {
  protected DriveTuning(String key) {
    super(key);
  }

  /**
   * Returns the max linear speed mps value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract double maxLinearSpeedMps();

  /**
   * Returns the min step meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract double minStepMeters();

  /**
   * Returns the base step meters value maintained by this Repulsor component.
   *
   * @param distanceMeters distance or field-coordinate value in meters.
   * @param slowDown value used by this operation.
   * @return value produced by this operation.
   */
  public abstract double baseStepMeters(double distanceMeters, boolean slowDown);

  /**
   * Returns the near goal scale value maintained by this Repulsor component.
   *
   * @param distanceMeters distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public abstract double nearGoalScale(double distanceMeters);

  /**
   * Returns the scale for turning value maintained by this Repulsor component.
   *
   * @param yawDeltaRad value used by this operation.
   * @param isScoring value used by this operation.
   * @return value produced by this operation.
   */
  public abstract double scaleForTurning(double yawDeltaRad, boolean isScoring);

  /**
   * Returns the step size meters value maintained by this Repulsor component.
   *
   * @param distanceMeters distance or field-coordinate value in meters.
   * @param obstacleMag obstacle set used for safety checks, costs, or replanning.
   * @param isScoring value used by this operation.
   * @param slowDown value used by this operation.
   * @return value produced by this operation.
   */
  public double stepSizeMeters(
      double distanceMeters, double obstacleMag, boolean isScoring, boolean slowDown) {
    double base = baseStepMeters(Math.max(0.0, distanceMeters), slowDown);
    return base;
    // double near = nearGoalScale(distanceMeters);
    // double obs = 1.0 / (1.0 + obstacleMag);
    // return Math.max(
    //     minStepMeters(), Math.min(maxLinearSpeedMps() * dtSeconds(), base * near * obs));
  }
}
