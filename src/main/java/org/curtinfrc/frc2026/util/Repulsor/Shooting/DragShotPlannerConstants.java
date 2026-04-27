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

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

/**
 * Provides drag shot planner constants functionality for the Repulsor projectile and shot-planning
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
final class DragShotPlannerConstants {
  /**
   * Configuration value for eps. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  static final double EPS = 1e-6;

  /**
   * Configuration value for min range meters. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  static final double MIN_RANGE_METERS = 0.5;

  /**
   * Configuration value for max range meters. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  static final double MAX_RANGE_METERS = 7.0;

  /**
   * Configuration value for max robot travel meters. Distances use meters in WPILib field
   * coordinates and should be treated as tunable when sourced from profiles.
   */
  static final double MAX_ROBOT_TRAVEL_METERS = 7.0;

  /**
   * Configuration value for max robot travel meters sq. Distances use meters in WPILib field
   * coordinates and should be treated as tunable when sourced from profiles.
   */
  static final double MAX_ROBOT_TRAVEL_METERS_SQ =
      MAX_ROBOT_TRAVEL_METERS * MAX_ROBOT_TRAVEL_METERS;

  /**
   * Configuration value for acceptable vertical error meters. Distances use meters in WPILib field
   * coordinates and should be treated as tunable when sourced from profiles.
   */
  static final double ACCEPTABLE_VERTICAL_ERROR_METERS = 0.06;

  /**
   * Configuration value for fast acceptable vertical error meters. Distances use meters in WPILib
   * field coordinates and should be treated as tunable when sourced from profiles.
   */
  static final double FAST_ACCEPTABLE_VERTICAL_ERROR_METERS = 0.25;

  /**
   * Configuration value for deg to rad. Angles use WPILib rotation conventions; names ending in
   * degrees are degrees, otherwise radians are assumed by the API.
   */
  static final double DEG_TO_RAD = Math.PI / 180.0;

  /**
   * Configuration value for rad to deg. Angles use WPILib rotation conventions; names ending in
   * degrees are degrees, otherwise radians are assumed by the API.
   */
  static final double RAD_TO_DEG = 180.0 / Math.PI;

  private DragShotPlannerConstants() {}
}
