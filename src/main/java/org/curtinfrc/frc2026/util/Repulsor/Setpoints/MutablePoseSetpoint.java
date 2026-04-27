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
import java.util.concurrent.atomic.AtomicReference;

/**
 * Provides mutable pose setpoint functionality for the Repulsor game setpoint abstraction layer for
 * field-relative goals and mechanisms. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class MutablePoseSetpoint extends GameSetpoint {
  private final AtomicReference<Pose2d> bluePoseRef;

  /**
   * Returns the mutable pose setpoint value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @param type value used by this operation.
   * @param bluePoseRef value used by this operation.
   */
  public MutablePoseSetpoint(String name, SetpointType type, AtomicReference<Pose2d> bluePoseRef) {
    super(name, type == null ? SetpointType.kOther : type, false);
    this.bluePoseRef = bluePoseRef == null ? new AtomicReference<>(Pose2d.kZero) : bluePoseRef;
  }

  /**
   * Returns the blue pose value maintained by this Repulsor component.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  @Override
  public Pose2d bluePose(SetpointContext ctx) {
    Pose2d p = bluePoseRef.get();
    return p != null ? p : Pose2d.kZero;
  }
}
