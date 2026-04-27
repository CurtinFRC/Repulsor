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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.StaticPoseSetpoint;

/**
 * Provides drive repulsor functionality for the Repulsor core Repulsor coordination layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public abstract class DriveRepulsor extends SubsystemBase {
  /**
   * Runs run velocity in the Repulsor runtime.
   *
   * @param speeds velocity input, normally field-relative unless the caller documents
   *     robot-relative motion.
   */
  public abstract void runVelocity(ChassisSpeeds speeds);

  /**
   * Returns the get pose value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract Pose2d getPose();

  /**
   * Returns the get omega pid value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract PIDController getOmegaPID();

  /**
   * Returns the align to value maintained by this Repulsor component.
   *
   * @param targetPose value used by this operation.
   * @return value produced by this operation.
   */
  public Command alignTo(Pose2d targetPose) {
    Repulsor re = StaticInstance.getInstance();

    RepulsorSetpoint setpoint =
        new RepulsorSetpoint(
            new StaticPoseSetpoint(
                "ALIGN_TARGET_" + targetPose.hashCode(), SetpointType.kOther, targetPose),
            HeightSetpoint.L1);

    assert re != null : "Repulsor instance is not initialized";
    assert re.isSameDrive(this) : "Repulsor instance does not match this DriveRepulsor";

    return re.alignTo(setpoint, CategorySpec.kEndgame);
  }
}
