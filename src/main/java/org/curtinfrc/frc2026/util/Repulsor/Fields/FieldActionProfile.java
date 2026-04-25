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
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.MovingShotSolver;

public record FieldActionProfile(Map<String, ProjectileShotAction> projectileShots) {
  public FieldActionProfile {
    projectileShots =
        projectileShots == null ? Map.of() : Map.copyOf(new LinkedHashMap<>(projectileShots));
  }

  public static FieldActionProfile none() {
    return new FieldActionProfile(Map.of());
  }

  public Optional<ProjectileShotAction> projectileShot(String id) {
    if (id == null || id.isBlank()) return Optional.empty();
    return Optional.ofNullable(projectileShots.get(id));
  }

  public Optional<ProjectileShotAction> firstProjectileShot(ActionRole role) {
    if (role == null) return Optional.empty();
    return projectileShots.values().stream().filter(action -> action.role() == role).findFirst();
  }

  public Optional<ProjectileShotAction> transferProjectileShot() {
    return firstProjectileShot(ActionRole.TRANSFER_TO_SCORE);
  }

  public Optional<ProjectileShotAction> scoreProjectileShot() {
    return firstProjectileShot(ActionRole.SCORE);
  }

  /** Compatibility shim for older 2026-specific behavior code. Prefer transferProjectileShot(). */
  @Deprecated(forRemoval = false)
  public Optional<ProjectileShotAction> shuttleShot() {
    return transferProjectileShot();
  }

  public enum ActionRole {
    SCORE,
    TRANSFER_TO_SCORE,
    COLLECT,
    ENDGAME,
    OTHER
  }

  public record ProjectileShotAction(
      String id,
      ActionRole role,
      Function<DriverStation.Alliance, Translation2d> targetForAlliance,
      GamePiecePhysics gamePiecePhysics,
      double targetHeightMeters,
      Constraints constraints,
      String routeLevel,
      HeightSetpoint routeMechanismSetpoint,
      double behindTargetMeters,
      double[] lateralOffsetsMeters,
      double fieldMarginMeters,
      boolean movingShotEnabled,
      MovingShotSolver.Config movingShotConfig) {
    public ProjectileShotAction {
      id = id == null || id.isBlank() ? "projectileShot" : id.trim();
      role = role == null ? ActionRole.OTHER : role;
      if (targetForAlliance == null) {
        throw new IllegalArgumentException("targetForAlliance cannot be null");
      }
      if (gamePiecePhysics == null) {
        throw new IllegalArgumentException("gamePiecePhysics cannot be null");
      }
      if (constraints == null) {
        throw new IllegalArgumentException("constraints cannot be null");
      }
      routeLevel = routeLevel == null || routeLevel.isBlank() ? "none" : routeLevel.trim();
      routeMechanismSetpoint =
          routeMechanismSetpoint == null ? HeightSetpoint.NONE : routeMechanismSetpoint;
      behindTargetMeters = Math.max(0.0, behindTargetMeters);
      fieldMarginMeters = Math.max(0.0, fieldMarginMeters);
      movingShotConfig =
          movingShotConfig == null ? MovingShotSolver.Config.defaults() : movingShotConfig;
      lateralOffsetsMeters =
          lateralOffsetsMeters == null || lateralOffsetsMeters.length == 0
              ? new double[] {0.0}
              : lateralOffsetsMeters.clone();
    }

    @Override
    public double[] lateralOffsetsMeters() {
      return lateralOffsetsMeters.clone();
    }

    /** Compatibility name for old route-height callers. Prefer routeMechanismSetpoint(). */
    @Deprecated(forRemoval = false)
    public HeightSetpoint routeHeight() {
      return routeMechanismSetpoint;
    }

    public Translation2d target(DriverStation.Alliance alliance) {
      Translation2d target =
          targetForAlliance.apply(alliance == null ? DriverStation.Alliance.Blue : alliance);
      return target == null ? new Translation2d() : target;
    }
  }
}
