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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Provides repulsor setpoint functionality for the Repulsor game setpoint abstraction layer for
 * field-relative goals and mechanisms. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class RepulsorSetpoint {
  private final GameSetpoint point;
  private final String levelId;
  private final HeightSetpoint mechanismSetpoint;

  /**
   * Returns the repulsor setpoint value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param height value used by this operation.
   */
  public RepulsorSetpoint(GameSetpoint point, HeightSetpoint height) {
    this(point, height == null ? "none" : height.name().toLowerCase(), height);
  }

  /**
   * Returns the repulsor setpoint value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param levelId value used by this operation.
   * @param mechanismSetpoint value used by this operation.
   */
  public RepulsorSetpoint(GameSetpoint point, String levelId, HeightSetpoint mechanismSetpoint) {
    if (point == null) throw new IllegalArgumentException("point cannot be null");
    this.point = point;
    this.levelId = levelId == null || levelId.isBlank() ? "none" : levelId.trim();
    this.mechanismSetpoint = mechanismSetpoint == null ? HeightSetpoint.NONE : mechanismSetpoint;
  }

  /**
   * Returns the point value maintained by this Repulsor component.
   *
   * @return game setpoint result for point.
   */
  public GameSetpoint point() {
    return point;
  }

  /**
   * Returns the height value maintained by this Repulsor component.
   *
   * @return height setpoint result for height.
   */
  public HeightSetpoint height() {
    return mechanismSetpoint;
  }

  /**
   * Returns the level id value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String levelId() {
    return levelId;
  }

  /**
   * Returns the mechanism setpoint value maintained by this Repulsor component.
   *
   * @return height setpoint result for mechanism setpoint.
   */
  public HeightSetpoint mechanismSetpoint() {
    return mechanismSetpoint;
  }

  /**
   * Returns the get blue value maintained by this Repulsor component.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  public Pose2d getBlue(SetpointContext ctx) {
    return point.bluePose(ctx == null ? SetpointContext.EMPTY : ctx);
  }

  /**
   * Returns the get red value maintained by this Repulsor component.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  public Pose2d getRed(SetpointContext ctx) {
    return point.redPose(ctx == null ? SetpointContext.EMPTY : ctx);
  }

  /**
   * Returns the get for alliance value maintained by this Repulsor component.
   *
   * @param alliance value used by this operation.
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  public Pose2d getForAlliance(Alliance alliance, SetpointContext ctx) {
    return point.poseForAlliance(alliance == null ? Alliance.Blue : alliance, ctx);
  }

  /**
   * Returns the latest value maintained by this Repulsor component.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  public Pose2d get(SetpointContext ctx) {
    return point.poseForCurrentAlliance(ctx == null ? SetpointContext.EMPTY : ctx);
  }
}
