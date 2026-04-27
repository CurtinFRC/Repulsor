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

/**
 * Describes reusable field actions that behaviours and reasoners can request without depending on a
 * specific FRC game. A field profile maps stable action IDs to concrete projectile targets,
 * mechanism routes, and shot constraints for the active game.
 *
 * <p>All target positions are expressed as WPILib {@link Translation2d} values in field-relative
 * meters. The profile is immutable after construction so it can be safely shared between periodic
 * behaviour code, strategy evaluation, and tests.
 *
 * @param projectileShots projectile shot actions keyed by profile-specific action ID
 */
public record FieldActionProfile(Map<String, ProjectileShotAction> projectileShots) {
  public FieldActionProfile {
    projectileShots =
        projectileShots == null ? Map.of() : Map.copyOf(new LinkedHashMap<>(projectileShots));
  }

  /**
   * Creates an empty action profile for fields or tests that do not expose reusable projectile
   * actions.
   *
   * @return profile with no configured shot actions
   */
  public static FieldActionProfile none() {
    return new FieldActionProfile(Map.of());
  }

  /**
   * Looks up a projectile shot action by ID.
   *
   * @param id action ID from the field profile; blank and {@code null} IDs do not match
   * @return matching shot action, or {@link Optional#empty()} when the action is not configured
   */
  public Optional<ProjectileShotAction> projectileShot(String id) {
    if (id == null || id.isBlank()) return Optional.empty();
    return Optional.ofNullable(projectileShots.get(id));
  }

  /**
   * Selects the first projectile shot action with the requested semantic role.
   *
   * @param role generic action role requested by a behaviour or reasoner
   * @return first matching action, or {@link Optional#empty()} when no action has that role
   */
  public Optional<ProjectileShotAction> firstProjectileShot(ActionRole role) {
    if (role == null) return Optional.empty();
    return projectileShots.values().stream().filter(action -> action.role() == role).findFirst();
  }

  /**
   * Selects the configured action for moving a resource to a later scoring location.
   *
   * @return transfer-to-score action when the field profile defines one
   */
  public Optional<ProjectileShotAction> transferProjectileShot() {
    return firstProjectileShot(ActionRole.TRANSFER_TO_SCORE);
  }

  /**
   * Selects the configured action for directly scoring a resource.
   *
   * @return scoring action when the field profile defines one
   */
  public Optional<ProjectileShotAction> scoreProjectileShot() {
    return firstProjectileShot(ActionRole.SCORE);
  }

  /**
   * Compatibility shim for older 2026-specific behavior code.
   *
   * @return the generic transfer-to-score projectile action
   * @deprecated prefer {@link #transferProjectileShot()} so callers do not encode shuttle-specific
   *     game language
   */
  @Deprecated(forRemoval = false)
  public Optional<ProjectileShotAction> shuttleShot() {
    return transferProjectileShot();
  }

  /**
   * Semantic action categories used by strategy code. These roles let a profile describe how the
   * game is played while behaviours remain generic.
   */
  public enum ActionRole {
    /** Directly score the currently handled resource. */
    SCORE,
    /** Move the resource to a safer or more valuable scoring area for later completion. */
    TRANSFER_TO_SCORE,
    /** Collect a resource without necessarily scoring or transferring it immediately. */
    COLLECT,
    /** Run an endgame-specific action for the field profile. */
    ENDGAME,
    /** Fallback role for actions that do not fit the common strategy categories. */
    OTHER
  }

  /**
   * Profile entry for a generic projectile action. A behaviour uses this record to choose a
   * field-relative firing target, mechanism route, static shot constraints, and optional
   * moving-shot compensation without knowing whether the game piece is fuel, a pipe, or another
   * resource.
   *
   * @param id stable action identifier used in profiles, logs, and strategy directives
   * @param role generic role used by strategy selection
   * @param targetForAlliance function returning the field-relative target for the requested
   *     alliance
   * @param gamePiecePhysics projectile physics model used by the shot solver
   * @param targetHeightMeters target opening or impact-plane height in meters
   * @param constraints allowed projectile speed/angle/yaw constraints
   * @param routeLevel profile route label for mechanism coordination
   * @param routeMechanismSetpoint mechanism setpoint used to derive release height when available
   * @param behindTargetMeters preferred stand-off distance behind the target in meters
   * @param lateralOffsetsMeters candidate lateral offsets, in meters, tested around the base shot
   *     pose
   * @param fieldMarginMeters minimum clamp margin from the field boundary in meters
   * @param movingShotEnabled whether behaviours should use field-velocity shot compensation
   * @param movingShotConfig tuning for moving-shot latency, prediction, and release gates
   */
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

    /**
     * Returns a defensive copy of the lateral shot offsets.
     *
     * @return candidate offsets in meters, in the order the planner should evaluate them
     */
    @Override
    public double[] lateralOffsetsMeters() {
      return lateralOffsetsMeters.clone();
    }

    /**
     * Compatibility name for older route-height callers.
     *
     * @return mechanism setpoint used by this action
     * @deprecated prefer {@link #routeMechanismSetpoint()} because actions are not always height
     *     based
     */
    @Deprecated(forRemoval = false)
    public HeightSetpoint routeHeight() {
      return routeMechanismSetpoint;
    }

    /**
     * Resolves the target for an alliance, defaulting to blue when the alliance is unknown.
     *
     * @param alliance WPILib alliance whose field-relative target should be used
     * @return non-null field-relative target in meters
     */
    public Translation2d target(DriverStation.Alliance alliance) {
      Translation2d target =
          targetForAlliance.apply(alliance == null ? DriverStation.Alliance.Blue : alliance);
      return target == null ? new Translation2d() : target;
    }
  }
}
