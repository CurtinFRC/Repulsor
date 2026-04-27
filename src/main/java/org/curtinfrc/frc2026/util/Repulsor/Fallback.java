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

package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Provides fallback functionality for the Repulsor core Repulsor coordination layer. Use this type
 * from robot code, field profiles, or tests when integrating the corresponding Repulsor subsystem.
 * Coordinates are field-relative unless a method documents robot-relative motion.
 */
public class Fallback {
  /**
   * Provides planner fallback functionality for the Repulsor core Repulsor coordination layer. Use
   * this type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public abstract class PlannerFallback {
    /**
     * Computes the calculate value for the current Repulsor planning state. Call this from periodic
     * planning or tests when a fresh decision is required; inputs should already be expressed in
     * the coordinate frame expected by the parameter names.
     *
     * @param currentPose value used by this operation.
     * @param target value used by this operation.
     * @return chassis speeds result for calculate.
     */
    public abstract ChassisSpeeds calculate(Translation2d currentPose, Translation2d target);

    /**
     * Returns the within value maintained by this Repulsor component.
     *
     * @param err value used by this operation.
     * @return value produced by this operation.
     */
    public boolean within(Translation2d err) {
      if (_withinDist() < 0.04) {
        return false;
      }
      return err.getNorm() < _withinDist();
    }

    protected abstract double _withinDist();
  }

  /**
   * Provides pid functionality for the Repulsor core Repulsor coordination layer. Use this type
   * from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public class PID extends PlannerFallback {
    private PIDController xController;
    private PIDController yController;

    /**
     * Returns the pid value maintained by this Repulsor component.
     *
     * @param kP value used by this operation.
     * @param kI value used by this operation.
     * @param kD value used by this operation.
     */
    public PID(double kP, double kI, double kD) {
      xController = new PIDController(kP, kI, kD);
      yController = new PIDController(kP, kI, kD);
    }

    /**
     * Computes the calculate value for the current Repulsor planning state. Call this from periodic
     * planning or tests when a fresh decision is required; inputs should already be expressed in
     * the coordinate frame expected by the parameter names.
     *
     * @param currentPose value used by this operation.
     * @param target value used by this operation.
     * @return chassis speeds result for calculate.
     */
    @Override
    public ChassisSpeeds calculate(Translation2d currentPose, Translation2d target) {
      return new ChassisSpeeds(
          xController.calculate(currentPose.getX(), target.getX()),
          yController.calculate(currentPose.getY(), target.getY()),
          0);
    }

    @Override
    protected double _withinDist() {
      return 0.02;
    }
  }
}
