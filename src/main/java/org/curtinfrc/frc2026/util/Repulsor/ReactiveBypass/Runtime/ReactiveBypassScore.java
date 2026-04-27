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

/**
 * Provides reactive bypass score functionality for the Repulsor runtime helper layer shared by
 * behaviours and planners. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
final class ReactiveBypassScore {
  /**
   * Configuration value for wp. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final Pose2d wp;

  /**
   * Configuration value for ok leg1. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final boolean okLeg1;

  /**
   * Configuration value for ok leg2. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final boolean okLeg2;

  /**
   * Configuration value for path len. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final double pathLen;

  /**
   * Configuration value for angle cost. Angles use WPILib rotation conventions; names ending in
   * degrees are degrees, otherwise radians are assumed by the API.
   */
  final double angleCost;

  /**
   * Configuration value for curvature cost. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final double curvatureCost;

  /**
   * Configuration value for wall penalty. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final double wallPenalty;

  /**
   * Configuration value for occ path cost. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final double occPathCost;

  /**
   * Configuration value for switch penalty. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final double switchPenalty;

  /**
   * Configuration value for zzz penalty. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final double zzzPenalty;

  /**
   * Configuration value for progress cost. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final double progressCost;

  /**
   * Configuration value for local occ cost. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final double localOccCost;

  /**
   * Configuration value for corner reward. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  final double cornerReward;

  /**
   * Configuration value for total cost. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final double totalCost;

  /**
   * Creates a reactive bypass score instance with the dependencies and tuning values used by this
   * Repulsor component.
   *
   * @param wp value used by this operation.
   * @param okLeg1 value used by this operation.
   * @param okLeg2 value used by this operation.
   * @param pathLen value used by this operation.
   * @param angleCost value used by this operation.
   * @param curvatureCost value used by this operation.
   * @param wallPenalty distance or field-coordinate value in meters.
   * @param occPathCost value used by this operation.
   * @param switchPenalty distance or field-coordinate value in meters.
   * @param zzzPenalty distance or field-coordinate value in meters.
   * @param progressCost value used by this operation.
   * @param localOccCost value used by this operation.
   * @param cornerReward value used by this operation.
   * @param totalCost value used by this operation.
   */
  ReactiveBypassScore(
      Pose2d wp,
      boolean okLeg1,
      boolean okLeg2,
      double pathLen,
      double angleCost,
      double curvatureCost,
      double wallPenalty,
      double occPathCost,
      double switchPenalty,
      double zzzPenalty,
      double progressCost,
      double localOccCost,
      double cornerReward,
      double totalCost) {
    this.wp = wp;
    this.okLeg1 = okLeg1;
    this.okLeg2 = okLeg2;
    this.pathLen = pathLen;
    this.angleCost = angleCost;
    this.curvatureCost = curvatureCost;
    this.wallPenalty = wallPenalty;
    this.occPathCost = occPathCost;
    this.switchPenalty = switchPenalty;
    this.zzzPenalty = zzzPenalty;
    this.progressCost = progressCost;
    this.localOccCost = localOccCost;
    this.cornerReward = cornerReward;
    this.totalCost = totalCost;
  }
}
