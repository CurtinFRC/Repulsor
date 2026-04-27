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

package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.DriveRepulsor;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Repulsor;
import org.curtinfrc.frc2026.util.Repulsor.VisionPlanner;

/**
 * Provides behaviour context functionality for the Repulsor command-behaviour layer that converts
 * strategy and state into WPILib commands. Use this type from robot code, field profiles, or tests
 * when integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
 * method documents robot-relative motion.
 */
public class BehaviourContext {
  /**
   * Configuration value for repulsor. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Repulsor repulsor;

  /**
   * Configuration value for planner. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final FieldPlanner planner;

  /**
   * Configuration value for vision. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final VisionPlanner vision;

  /**
   * Configuration value for drive. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final DriveRepulsor drive;

  /**
   * Configuration value for robot y. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double robot_x, robot_y;

  /**
   * Configuration value for robot pose. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Supplier<Pose2d> robotPose;

  /**
   * Returns the behaviour context value maintained by this Repulsor component.
   *
   * @param repulsor value used by this operation.
   * @param planner value used by this operation.
   * @param vision value used by this operation.
   * @param drive value used by this operation.
   * @param robot_x distance or field-coordinate value in meters.
   * @param robot_y distance or field-coordinate value in meters.
   * @param robotPose WPILib Pose2d in field-relative coordinates.
   */
  public BehaviourContext(
      Repulsor repulsor,
      FieldPlanner planner,
      VisionPlanner vision,
      DriveRepulsor drive,
      double robot_x,
      double robot_y,
      Supplier<Pose2d> robotPose) {
    this.repulsor = repulsor;
    this.planner = planner;
    this.vision = vision;
    this.drive = drive;
    this.robot_x = robot_x;
    this.robot_y = robot_y;
    this.robotPose = robotPose;
  }
}
