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

package org.curtinfrc.frc2026.util.Repulsor.Fields;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;

public record FieldActionProfile(Optional<ShuttleShotProfile> shuttleShot) {
  public FieldActionProfile {
    shuttleShot = shuttleShot == null ? Optional.empty() : shuttleShot;
  }

  public static FieldActionProfile none() {
    return new FieldActionProfile(Optional.empty());
  }

  public record ShuttleShotProfile(
      Function<DriverStation.Alliance, Translation2d> targetForAlliance,
      GamePiecePhysics gamePiecePhysics,
      double targetHeightMeters,
      Constraints constraints,
      HeightSetpoint routeHeight,
      double behindTargetMeters,
      double[] lateralOffsetsMeters,
      double fieldMarginMeters) {
    public ShuttleShotProfile {
      if (targetForAlliance == null) {
        throw new IllegalArgumentException("targetForAlliance cannot be null");
      }
      if (gamePiecePhysics == null) {
        throw new IllegalArgumentException("gamePiecePhysics cannot be null");
      }
      if (constraints == null) {
        throw new IllegalArgumentException("constraints cannot be null");
      }
      routeHeight = routeHeight == null ? HeightSetpoint.NONE : routeHeight;
      behindTargetMeters = Math.max(0.0, behindTargetMeters);
      fieldMarginMeters = Math.max(0.0, fieldMarginMeters);
      lateralOffsetsMeters =
          lateralOffsetsMeters == null || lateralOffsetsMeters.length == 0
              ? new double[] {0.0}
              : lateralOffsetsMeters.clone();
    }

    @Override
    public double[] lateralOffsetsMeters() {
      return lateralOffsetsMeters.clone();
    }

    public Translation2d target(DriverStation.Alliance alliance) {
      Translation2d target =
          targetForAlliance.apply(
              alliance == null ? DriverStation.Alliance.Blue : alliance);
      return target == null ? new Translation2d() : target;
    }
  }
}
