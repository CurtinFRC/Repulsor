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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides reactive bypass state functionality for the Repulsor runtime helper layer shared by
 * behaviours and planners. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
final class ReactiveBypassState {
  /**
   * Configuration value for latched subgoal. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  Pose2d latchedSubgoal = null;

  /**
   * Configuration value for latched at position. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  Translation2d latchedAtPosition = Translation2d.kZero;

  /**
   * Configuration value for time since latch s. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  double timeSinceLatchS = 1e9;

  /**
   * Configuration value for time since eval s. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  double timeSinceEvalS = 1e9;

  /**
   * Configuration value for time since side switch s. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  double timeSinceSideSwitchS = 1e9;

  /**
   * Configuration value for preferred side. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  int preferredSide = 0;

  /**
   * Configuration value for last occ. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  double lastOcc = 0.0;

  /**
   * Configuration value for last chosen cost. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  Double lastChosenCost = null;

  /**
   * Configuration value for side confidence. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  double sideConfidence = 0.0;

  /**
   * Configuration value for pinned mode. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  boolean pinnedMode = false;

  /**
   * Configuration value for pinned time s. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  double pinnedTimeS = 0.0;

  /**
   * Configuration value for blocked accum s. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  double blockedAccumS = 0.0;

  /**
   * Configuration value for pinned cooldown s. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  double pinnedCooldownS = 0.0;

  /**
   * Configuration value for pinned heading. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  Rotation2d pinnedHeading = Rotation2d.kZero;

  /**
   * Configuration value for consecutive bypass failures. Time values use seconds and should be
   * tuned against measured robot loop and mechanism latency.
   */
  int consecutiveBypassFailures = 0;

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  void reset() {
    latchedSubgoal = null;
    latchedAtPosition = Translation2d.kZero;
    timeSinceLatchS = 1e9;
    timeSinceEvalS = 1e9;
    timeSinceSideSwitchS = 1e9;
    preferredSide = 0;
    lastOcc = 0.0;
    lastChosenCost = null;
    sideConfidence = 0.0;
    pinnedMode = false;
    pinnedTimeS = 0.0;
    blockedAccumS = 0.0;
    pinnedCooldownS = 0.0;
    pinnedHeading = Rotation2d.kZero;
    consecutiveBypassFailures = 0;
  }
}
