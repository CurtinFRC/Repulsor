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

import org.curtinfrc.frc2026.util.Repulsor.Constants;

/**
 * Provides reactive bypass config functionality for the Repulsor runtime helper layer shared by
 * behaviours and planners. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class ReactiveBypassConfig {
  public double fieldLen = Constants.FIELD_GEOMETRY.lengthMeters();
  public double fieldWid = Constants.FIELD_GEOMETRY.widthMeters();

  /**
   * Configuration value for inflation meters. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  public double inflationMeters = 0.10;

  /**
   * Configuration value for trigger ahead meters. Distances use meters in WPILib field coordinates
   * and should be treated as tunable when sourced from profiles.
   */
  public double triggerAheadMeters = 1.2;

  /**
   * Configuration value for trigger width meters. Distances use meters in WPILib field coordinates
   * and should be treated as tunable when sourced from profiles.
   */
  public double triggerWidthMeters = 0.7;

  /**
   * Configuration value for corridor samples. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public int corridorSamples = 15;

  /**
   * Configuration value for occ high. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double occHigh = 0.2079111216823982;

  /**
   * Configuration value for occ low. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double occLow = 0.08402471544016843;

  /**
   * Configuration value for lateral meters. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  public double lateralMeters = 0.50;

  /**
   * Configuration value for lateral max meters. Distances use meters in WPILib field coordinates
   * and should be treated as tunable when sourced from profiles.
   */
  public double lateralMaxMeters = 1.90;

  /**
   * Configuration value for min lateral meters. Distances use meters in WPILib field coordinates
   * and should be treated as tunable when sourced from profiles.
   */
  public double minLateralMeters = 0.55;

  /**
   * Configuration value for forward meters. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  public double forwardMeters = 1.4219386889821075;

  /**
   * Configuration value for forward max meters. Distances use meters in WPILib field coordinates
   * and should be treated as tunable when sourced from profiles.
   */
  public double forwardMaxMeters = 2.50;

  /**
   * Configuration value for hold min seconds. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  public double holdMinSeconds = 0.45;

  /**
   * Configuration value for recalc seconds. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  public double recalcSeconds = 0.18;

  /**
   * Configuration value for subgoal max seconds. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public double subgoalMaxSeconds = 2.0;

  /**
   * Configuration value for min heading err to trigger deg. Angles use WPILib rotation conventions;
   * names ending in degrees are degrees, otherwise radians are assumed by the API.
   */
  public double minHeadingErrToTriggerDeg = 6.0;

  /**
   * Configuration value for release ahead meters. Distances use meters in WPILib field coordinates
   * and should be treated as tunable when sourced from profiles.
   */
  public double releaseAheadMeters = 1.35;

  /**
   * Configuration value for angle cost weight. Angles use WPILib rotation conventions; names ending
   * in degrees are degrees, otherwise radians are assumed by the API.
   */
  public double angleCostWeight = 0.6538189117352257;

  /**
   * Configuration value for curvature weight. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double curvatureWeight = 0.5598688771152425;

  /**
   * Configuration value for wall penalty gain. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double wallPenaltyGain = 0.55;

  /**
   * Configuration value for occ cost gain. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double occCostGain = 2;

  /**
   * Configuration value for side switch penalty. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double sideSwitchPenalty = 0.35;

  /**
   * Configuration value for heading deadband deg. Angles use WPILib rotation conventions; names
   * ending in degrees are degrees, otherwise radians are assumed by the API.
   */
  public double headingDeadbandDeg = 4.0;

  /**
   * Configuration value for min progress meters. Distances use meters in WPILib field coordinates
   * and should be treated as tunable when sourced from profiles.
   */
  public double minProgressMeters = 0.08;

  /**
   * Configuration value for probe occ leg step. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double probeOccLegStep = 0.25;

  /**
   * Configuration value for side bias probe angle deg. Angles use WPILib rotation conventions;
   * names ending in degrees are degrees, otherwise radians are assumed by the API.
   */
  public double sideBiasProbeAngleDeg = 10.0;

  /**
   * Configuration value for side stick seconds. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  public double sideStickSeconds = 0.70;

  /**
   * Configuration value for subgoal slew rate mps. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double subgoalSlewRateMps = 2.0;

  /**
   * Configuration value for subgoal jitter meters. Distances use meters in WPILib field coordinates
   * and should be treated as tunable when sourced from profiles.
   */
  public double subgoalJitterMeters = 0.04;

  /**
   * Configuration value for arc angle deg. Angles use WPILib rotation conventions; names ending in
   * degrees are degrees, otherwise radians are assumed by the API.
   */
  public double arcAngleDeg = 18.0;

  /**
   * Configuration value for arc rmin. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double arcRMin = 0.9;

  /**
   * Configuration value for arc rmax. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double arcRMax = 1.8;

  /**
   * Configuration value for arc radial steps. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public int arcRadialSteps = 2;

  /**
   * Configuration value for arc angular steps per side. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public int arcAngularStepsPerSide = 2;

  /**
   * Configuration value for relatch improve frac. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double relatchImproveFrac = 0.12;

  /**
   * Configuration value for zzz max yaw delta deg. Angles use WPILib rotation conventions; names
   * ending in degrees are degrees, otherwise radians are assumed by the API.
   */
  public double zzzMaxYawDeltaDeg = 22.0;

  /**
   * Configuration value for zzz penalty. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double zzzPenalty = 0.40;

  /**
   * Configuration value for vib window s. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double vibWindowS = 0.6;

  /**
   * Configuration value for vib min disp. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double vibMinDisp = 0.10;

  /**
   * Configuration value for vib max dir flips. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public int vibMaxDirFlips = 8;

  /**
   * Configuration value for escape forward. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double escapeForward = 1.60;

  /**
   * Configuration value for escape lateral. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double escapeLateral = 1.30;

  /**
   * Configuration value for escape hold s. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double escapeHoldS = 0.60;

  /**
   * Configuration value for escape occ boost. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double escapeOccBoost = 0.25;

  /**
   * Configuration value for progress cost weight. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double progressCostWeight = 0.05;

  /**
   * Configuration value for min forward progress meters. Distances use meters in WPILib field
   * coordinates and should be treated as tunable when sourced from profiles.
   */
  public double minForwardProgressMeters = 0.50;

  /**
   * Configuration value for near goal dist meters. Distances use meters in WPILib field coordinates
   * and should be treated as tunable when sourced from profiles.
   */
  public double nearGoalDistMeters = 3.0;

  /**
   * Configuration value for near goal forward scale. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public double nearGoalForwardScale = 0.60;

  /**
   * Configuration value for near goal lateral scale. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public double nearGoalLateralScale = 0.70;

  /**
   * Configuration value for stuck min forward progress. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public double stuckMinForwardProgress = 0.15;

  /**
   * Configuration value for stuck occ min. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double stuckOccMin = 0.30;

  /**
   * Configuration value for stuck lookback frac. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double stuckLookbackFrac = 0.7;

  /**
   * Configuration value for side switch stuck penalty scale. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public double sideSwitchStuckPenaltyScale = 0.25;

  /**
   * Configuration value for pinned occ min. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double pinnedOccMin = 0.40;

  /**
   * Configuration value for pinned min time seconds. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public double pinnedMinTimeSeconds = 0.35;

  /**
   * Configuration value for pinned max time seconds. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public double pinnedMaxTimeSeconds = 3.0;

  /**
   * Configuration value for pinned cooldown seconds. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public double pinnedCooldownSeconds = 0.70;

  /**
   * Configuration value for corner wall thresh. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double cornerWallThresh = 3.0;

  /**
   * Configuration value for corner escape lat boost. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public double cornerEscapeLatBoost = 1.8;

  /**
   * Configuration value for corner escape fwd boost. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public double cornerEscapeFwdBoost = 1.4;

  /**
   * Configuration value for corner escape blend. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double cornerEscapeBlend = 0.65;

  /**
   * Configuration value for corner reward gain. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double cornerRewardGain = 0.6;
}
