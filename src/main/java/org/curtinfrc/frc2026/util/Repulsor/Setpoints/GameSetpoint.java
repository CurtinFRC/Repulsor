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

package org.curtinfrc.frc2026.util.Repulsor.Setpoints;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Provides game setpoint functionality for the Repulsor game setpoint abstraction layer for
 * field-relative goals and mechanisms. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public abstract class GameSetpoint {
  private final String name;
  private final SetpointType type;
  private final boolean canFlip;

  protected GameSetpoint(String name, SetpointType type) {
    this.name = name;
    this.type = type;
    this.canFlip = true;
  }

  protected GameSetpoint(String name, SetpointType type, boolean canFlip) {
    this.name = name;
    this.type = type;
    this.canFlip = canFlip;
  }

  /**
   * Returns the name value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public final String name() {
    return name;
  }

  /**
   * Returns the type value maintained by this Repulsor component.
   *
   * @return setpoint type result for type.
   */
  public final SetpointType type() {
    return type;
  }

  /**
   * Returns the blue pose value maintained by this Repulsor component.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  public abstract Pose2d bluePose(SetpointContext ctx);

  /**
   * Returns the red pose value maintained by this Repulsor component.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  public Pose2d redPose(SetpointContext ctx) {
    Pose2d blue = bluePose(ctx);
    return canFlip ? ChoreoAllianceFlipUtil.flip(blue) : blue;
  }

  /**
   * Returns the pose for alliance value maintained by this Repulsor component.
   *
   * @param alliance value used by this operation.
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  public final Pose2d poseForAlliance(Alliance alliance, SetpointContext ctx) {
    return alliance == Alliance.Red ? redPose(ctx) : bluePose(ctx);
  }

  /**
   * Returns the pose for current alliance value maintained by this Repulsor component.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  public final Pose2d poseForCurrentAlliance(SetpointContext ctx) {
    return poseForAlliance(SetpointUtil.currentAllianceOrBlue(), ctx);
  }

  /**
   * Returns the approximate blue pose value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Pose2d approximateBluePose() {
    return bluePose(SetpointContext.EMPTY);
  }

  /**
   * Returns the approximate red pose value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Pose2d approximateRedPose() {
    return redPose(SetpointContext.EMPTY);
  }
}
