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

package org.curtinfrc.frc2026.util.Repulsor.Tuning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides turn tuning functionality for the Repulsor drive and turn tuning model layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public abstract class TurnTuning extends Tuning {
  protected TurnTuning(String key) {
    super(key);
  }

  /**
   * Contract for collision checker implementations used by the Repulsor drive and turn tuning model
   * layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public interface CollisionChecker {
    boolean intersects(Translation2d[] rect);
  }

  /**
   * Provides turn result functionality for the Repulsor drive and turn tuning model layer. Use this
   * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static final class TurnResult {
    /**
     * Configuration value for yaw. Angles use WPILib rotation conventions; names ending in degrees
     * are degrees, otherwise radians are assumed by the API.
     */
    public final Rotation2d yaw;

    /**
     * Configuration value for speed scale. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public final double speedScale;

    /**
     * Returns the turn result value maintained by this Repulsor component.
     *
     * @param yaw value used by this operation.
     * @param speedScale velocity input, normally field-relative unless the caller documents
     *     robot-relative motion.
     */
    public TurnResult(Rotation2d yaw, double speedScale) {
      this.yaw = yaw;
      this.speedScale = speedScale;
    }
  }

  /**
   * Returns the max omega rad per sec value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract double maxOmegaRadPerSec();

  /**
   * Returns the turn margin meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract double turnMarginMeters();

  /**
   * Returns the turn samples value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract int turnSamples();

  /**
   * Computes the score turn margin mult value for the current Repulsor planning state.
   *
   * @return value produced by this operation.
   */
  public abstract double scoreTurnMarginMult();

  /**
   * Computes the score safety bubble meters value for the current Repulsor planning state.
   *
   * @return value produced by this operation.
   */
  public abstract double scoreSafetyBubbleMeters();

  /**
   * Returns the dock blend start meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract double dockBlendStartMeters();

  /**
   * Returns the dock blend end meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract double dockBlendEndMeters();

  /**
   * Returns the dock blend samples value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public abstract int dockBlendSamples();

  /**
   * Computes the score turn slowdown factor value for the current Repulsor planning state.
   *
   * @return value produced by this operation.
   */
  public abstract double scoreTurnSlowdownFactor();

  /**
   * Returns the plan value maintained by this Repulsor component.
   *
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @param goal value used by this operation.
   * @param pathHeading value used by this operation.
   * @param stepVec value used by this operation.
   * @param isScoring value used by this operation.
   * @param robotX distance or field-coordinate value in meters.
   * @param robotY distance or field-coordinate value in meters.
   * @param checker value used by this operation.
   * @return turn result result for plan.
   */
  public abstract TurnResult plan(
      Pose2d pose,
      Pose2d goal,
      Rotation2d pathHeading,
      Translation2d stepVec,
      boolean isScoring,
      double robotX,
      double robotY,
      CollisionChecker checker);

  /**
   * Returns the robot rect value maintained by this Repulsor component. Inputs are FULL footprint
   * length and width; half-extents are derived internally exactly once.
   *
   * @param center value used by this operation.
   * @param yaw value used by this operation.
   * @param lengthMeters full robot footprint length in meters.
   * @param widthMeters full robot footprint width in meters.
   * @return value produced by this operation.
   */
  public static Translation2d[] robotRect(
      Translation2d center, Rotation2d yaw, double lengthMeters, double widthMeters) {
    double hx = lengthMeters * 0.5, hy = widthMeters * 0.5;
    Translation2d[] local =
        new Translation2d[] {
          new Translation2d(+hx, +hy),
          new Translation2d(+hx, -hy),
          new Translation2d(-hx, -hy),
          new Translation2d(-hx, +hy)
        };
    Translation2d[] world = new Translation2d[4];
    for (int i = 0; i < 4; i++) world[i] = local[i].rotateBy(yaw).plus(center);
    return world;
  }
}
