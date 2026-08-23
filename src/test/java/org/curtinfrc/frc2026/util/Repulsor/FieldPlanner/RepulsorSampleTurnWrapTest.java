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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class RepulsorSampleTurnWrapTest {
  private static final double NEAR_SEAM = 3.13;

  @Test
  void omegaAcrossPiSeamCommandsWrappedRotationNotFullSpin() {
    PIDController omegaPID = new PIDController(1.0, 0.0, 0.0);
    RepulsorSample sample =
        new RepulsorSample(new Translation2d(), 0.0, 0.0, Radians.of(-NEAR_SEAM));

    ChassisSpeeds speeds = sample.asChassisSpeeds(omegaPID, Rotation2d.fromRadians(NEAR_SEAM));

    double unwrappedError = 2.0 * NEAR_SEAM;
    assertTrue(
        Math.abs(speeds.omegaRadiansPerSecond) < unwrappedError / 2.0,
        "omega should use wrapped angular error, got " + speeds.omegaRadiansPerSecond);
    assertTrue(Math.abs(speeds.omegaRadiansPerSecond) < 0.25);
  }

  @Test
  void omegaAcrossNegativePiSeamCommandsWrappedRotationBothDirections() {
    PIDController omegaPID = new PIDController(1.0, 0.0, 0.0);
    RepulsorSample sample =
        new RepulsorSample(new Translation2d(), 0.0, 0.0, Radians.of(NEAR_SEAM));

    ChassisSpeeds speeds = sample.asChassisSpeeds(omegaPID, Rotation2d.fromRadians(-NEAR_SEAM));

    assertTrue(Math.abs(speeds.omegaRadiansPerSecond) < 0.25);
  }
}
