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

package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.IntakeFootprint;
import org.curtinfrc.frc2026.util.Repulsor.Interval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.FootprintEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.HeadingPick;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.IntentAgg;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.ResourceRegions;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.Track;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.CollectProbe;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceCollectionProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Runtime.*;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.ResourceRegionSummary;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;

/**
 * Provides predictive field state ops functionality for the Repulsor predictive field-state and
 * collection-planning layer. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class PredictiveFieldStateOps {

  /**
   * Returns the error value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static boolean error() {
    return false;
  }

  /**
   * Returns the probe collect value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return collect probe result for probe collect.
   */
  public CollectProbe probeCollect(Translation2d p) {
    return probeCollect(p, 0.75);
  }

  /**
   * Returns the probe collect value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param countR value used by this operation.
   * @return collect probe result for probe collect.
   */
  public CollectProbe probeCollect(Translation2d p, double countR) {
    if (p == null) return new CollectProbe(0, 0.0);
    SpatialDyn dyn = cachedDyn();
    if (dyn == null) return new CollectProbe(0, 0.0);
    int c = dyn.countResourcesWithin(p, Math.max(0.05, countR));
    double u = dyn.valueAt(p);
    return new CollectProbe(c, u);
  }

  /**
   * Returns the footprint has collect resource value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public boolean footprintHasCollectResource(Translation2d center, double cellM) {
    if (center == null) return false;
    SpatialDyn dyn = cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return false;
    if (dyn != lastFootprintDyn) {
      footprintCache.clear();
      lastFootprintDyn = dyn;
    }
    long key = footprintKey(center, cellM);
    Boolean cached = footprintCache.get(key);
    if (cached != null) return cached;
    double rCore = coreRadiusFor(cellM);
    double searchR = Math.max(0.70, rCore * 3.0);
    Translation2d nearest = dyn.nearestResourceTo(center, searchR);
    if (nearest == null) {
      footprintCache.put(key, false);
      return false;
    }
    double minUnits =
        Math.max(
            0.02, Math.min(COLLECT_FINE_MIN_UNITS, dynamicMinUnits(dyn.totalEvidence()) * 0.75));
    Rotation2d base = face(center, nearest, Rotation2d.kZero);
    HeadingPick pick = bestHeadingForFootprint(dyn, center, nearest, base, rCore, minUnits);
    boolean ok = pick != null;
    if (footprintCache.size() > 2048) footprintCache.clear();
    footprintCache.put(key, ok);
    return ok;
  }

  /**
   * Returns the footprint has fuel value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public boolean footprintHasFuel(Translation2d center, double cellM) {
    return footprintHasCollectResource(center, cellM);
  }

  /**
   * Runs mark collect depleted in the Repulsor runtime.
   *
   * @param p value used by this operation.
   * @param cellM value used by this operation.
   * @param strength value used by this operation.
   */
  public void markCollectDepleted(Translation2d p, double cellM, double strength) {
    addDepletedMark(p, Math.max(0.10, cellM * 2.0), strength, DEPLETED_TTL_S, false);
  }

  /**
   * Configuration value for min dt. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double MIN_DT = 0.02;

  /**
   * Configuration value for max meas dt. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double MAX_MEAS_DT = 0.20;

  /**
   * Configuration value for eta floor. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double ETA_FLOOR = 0.05;

  // Keep offload runtime independent from robot-wide constants static init.
  /**
   * Configuration value for offload safe robot x m. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double OFFLOAD_SAFE_ROBOT_X_M = 0.85;

  /**
   * Configuration value for offload safe robot y m. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double OFFLOAD_SAFE_ROBOT_Y_M = 0.85;

  /**
   * Configuration value for default enemy speed. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double DEFAULT_ENEMY_SPEED = 2.2;

  /**
   * Configuration value for default ally speed. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double DEFAULT_ALLY_SPEED = 3.0;

  /**
   * Configuration value for default our speed. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double DEFAULT_OUR_SPEED = 3.5;

  /**
   * Configuration value for acc limit. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double ACC_LIMIT = 2.5;

  /**
   * Configuration value for vel ema. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double VEL_EMA = 0.35;

  /**
   * Configuration value for pos ema. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double POS_EMA = 0.15;

  /**
   * Configuration value for softmax temp. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  static final double SOFTMAX_TEMP = 1.35;

  /**
   * Configuration value for reservation radius. Distances use meters in WPILib field coordinates
   * and should be treated as tunable when sourced from profiles.
   */
  public static final double RESERVATION_RADIUS = 0.85;

  /**
   * Configuration value for kernel sigma. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double KERNEL_SIGMA = 0.95;

  /**
   * Configuration value for hyst persist s. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double HYST_PERSIST_S = 0.8;

  /**
   * Configuration value for hyst bonus. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double HYST_BONUS = 0.22;

  /**
   * Configuration value for adv gain. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double ADV_GAIN = 1.1;

  /**
   * Configuration value for dist cost. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double DIST_COST = 0.10;

  /**
   * Configuration value for pressure gain. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double PRESSURE_GAIN = 0.72;

  /**
   * Configuration value for congest cost. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double CONGEST_COST = 0.95;

  /**
   * Configuration value for capacity gain. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double CAPACITY_GAIN = 0.45;

  /**
   * Configuration value for heading gain. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double HEADING_GAIN = 0.18;

  /**
   * Configuration value for collect value gain. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_VALUE_GAIN = 1.10;

  /**
   * Configuration value for collect eta cost. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double COLLECT_ETA_COST = 1.05;

  /**
   * Configuration value for collect enemy press cost. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_ENEMY_PRESS_COST = 1.15;

  /**
   * Configuration value for collect ally congest cost. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_ALLY_CONGEST_COST = 0.95;

  /**
   * Configuration value for collect enemy intent cost. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_ENEMY_INTENT_COST = 0.75;

  /**
   * Configuration value for collect ally intent cost. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_ALLY_INTENT_COST = 0.55;

  /**
   * Configuration value for collect value sat k. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_VALUE_SAT_K = 0.75;

  /**
   * Configuration value for collect age decay. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double COLLECT_AGE_DECAY = 1.25;

  /**
   * Configuration value for collect local avoid r. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_LOCAL_AVOID_R = 0.9;

  /**
   * Configuration value for collect activity sigma. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_ACTIVITY_SIGMA = 1.05;

  /**
   * Configuration value for collect activity ally w. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_ACTIVITY_ALLY_W = 0.80;

  /**
   * Configuration value for collect activity enemy w. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_ACTIVITY_ENEMY_W = 0.55;

  /**
   * Configuration value for collect activity dyn w. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_ACTIVITY_DYN_W = 0.60;

  /**
   * Configuration value for collect region samples w. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_REGION_SAMPLES_W = 2.70;

  /**
   * Configuration value for collect cell m. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double COLLECT_CELL_M = 0.10;

  /**
   * Configuration value for collect near bonus. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_NEAR_BONUS = 0.85;

  /**
   * Configuration value for collect near decay. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_NEAR_DECAY = 1.1; // 1.35;

  /**
   * Configuration value for collect spread score r. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_SPREAD_SCORE_R = 0.85;

  /**
   * Configuration value for collect spread min. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_SPREAD_MIN = 0.30;

  /**
   * Configuration value for collect spread max. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_SPREAD_MAX = 0.65;

  /**
   * Configuration value for shoot x end band m. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double SHOOT_X_END_BAND_M = 12.5631260802;

  /**
   * Configuration value for band width m. Distances use meters in WPILib field coordinates and
   * should be treated as tunable when sourced from profiles.
   */
  public static final double BAND_WIDTH_M = 2.167294751;

  /**
   * Configuration value for x left band. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final Interval<Double> X_LEFT_BAND =
      Interval.closed(SHOOT_X_END_BAND_M - BAND_WIDTH_M, SHOOT_X_END_BAND_M);

  /**
   * Configuration value for x right band. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final Interval<Double> X_RIGHT_BAND =
      Interval.closed(
          Constants.FIELD_LENGTH - SHOOT_X_END_BAND_M,
          Constants.FIELD_LENGTH - (SHOOT_X_END_BAND_M - BAND_WIDTH_M));

  /**
   * Configuration value for default collect resource type. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final String DEFAULT_COLLECT_RESOURCE_TYPE = "fuel";

  /**
   * Configuration value for collect core r min. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_CORE_R_MIN = 0.05;

  /**
   * Configuration value for collect core r max. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_CORE_R_MAX = 0.12;

  /**
   * Configuration value for collect snap r min. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_SNAP_R_MIN = 0.35;

  /**
   * Configuration value for collect snap r max. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_SNAP_R_MAX = 0.60;

  /**
   * Configuration value for collect micro centroid r min. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_MICRO_CENTROID_R_MIN = 0.18;

  /**
   * Configuration value for collect micro centroid r max. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_MICRO_CENTROID_R_MAX = 0.35;

  /**
   * Configuration value for collect jitter r min. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_JITTER_R_MIN = 0.06;

  /**
   * Configuration value for collect jitter r max. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_JITTER_R_MAX = 0.14;

  /**
   * Configuration value for collect hole penalty. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_HOLE_PENALTY = 2.35;

  /**
   * Configuration value for collect edge penalty. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_EDGE_PENALTY = 0.85;

  /**
   * Configuration value for collect nofuel penalty. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_NOFUEL_PENALTY = 3.00;

  /**
   * Configuration value for collect grid topk. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  static final int COLLECT_GRID_TOPK = 420;

  /**
   * Configuration value for collect resource seeds max. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  static final int COLLECT_RESOURCE_SEEDS_MAX = 64;

  /**
   * Configuration value for collect cand gate r. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double COLLECT_CAND_GATE_R = 0.60;

  /**
   * Configuration value for collect peak gate r. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double COLLECT_PEAK_GATE_R = 0.60;

  /**
   * Configuration value for collect peak score r. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double COLLECT_PEAK_SCORE_R = 0.85;

  /**
   * Configuration value for collect coarse min region units. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_COARSE_MIN_REGION_UNITS = 0.12;

  /**
   * Configuration value for collect fine min units. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_FINE_MIN_UNITS = 0.08;

  /**
   * Configuration value for resource sigma abs max. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  static final double RESOURCE_SIGMA_ABS_MAX = 0.45;

  /**
   * Configuration value for resource sigma rel max. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  static final double RESOURCE_SIGMA_REL_MAX = 1.25;

  /**
   * Configuration value for resource sigma min. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double RESOURCE_SIGMA_MIN = 0.06;

  /**
   * Configuration value for collect cluster max. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final int COLLECT_CLUSTER_MAX = 72;

  /**
   * Configuration value for collect cluster bin m. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double COLLECT_CLUSTER_BIN_M = 0.14;

  /**
   * Configuration value for collect grid fallback max. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  static final int COLLECT_GRID_FALLBACK_MAX = 1400;

  /**
   * Configuration value for depleted ttl s. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double DEPLETED_TTL_S = 3.25;

  /**
   * Configuration value for depleted pen w. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double DEPLETED_PEN_W = 2.75;

  /**
   * Configuration value for depleted mark near m. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double DEPLETED_MARK_NEAR_M = 0.65;

  /**
   * Configuration value for depleted mark empty units. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double DEPLETED_MARK_EMPTY_UNITS = 0.07;

  /**
   * Configuration value for resource hard max age s. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  static final double RESOURCE_HARD_MAX_AGE_S = 0.95;

  /**
   * Configuration value for collect shortlist eta. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final int COLLECT_SHORTLIST_ETA = 36;

  /**
   * Configuration value for collect shortlist coarse. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  static final int COLLECT_SHORTLIST_COARSE = 28;

  /**
   * Configuration value for collect fine offsets grid. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final int COLLECT_FINE_OFFSETS_GRID = 3;

  /**
   * Configuration value for collect fine offsets scale. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_FINE_OFFSETS_SCALE = 0.65;

  /**
   * Configuration value for collect commit min s. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_COMMIT_MIN_S = 0.12;

  /**
   * Configuration value for collect commit max s. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_COMMIT_MAX_S = 0.34;

  /**
   * Configuration value for collect switch base. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_SWITCH_BASE = 0.12;

  /**
   * Configuration value for collect switch eta w. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_SWITCH_ETA_W = 0.07;

  /**
   * Configuration value for collect progress min drop m. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_PROGRESS_MIN_DROP_M = 0.25;

  /**
   * Configuration value for collect progress window s. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final double COLLECT_PROGRESS_WINDOW_S = 0.70;

  /**
   * Configuration value for collect arrive r. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double COLLECT_ARRIVE_R = 0.75;

  /**
   * Configuration value for collect arrive verify s. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_ARRIVE_VERIFY_S = 0.28;

  /**
   * Configuration value for collect fail cooldown s. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final double COLLECT_FAIL_COOLDOWN_S = 2.10;

  /**
   * Configuration value for evidence r. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static final double EVIDENCE_R = 0.85;

  /**
   * Configuration value for evidence min base. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double EVIDENCE_MIN_BASE = 0.05;

  /**
   * Configuration value for evidence min max. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double EVIDENCE_MIN_MAX = 0.18;

  /**
   * Configuration value for activity cap. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double ACTIVITY_CAP = 1.05;

  /**
   * Configuration value for path samples. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  static final int PATH_SAMPLES = 7;

  /**
   * Configuration value for path cost w. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  static final double PATH_COST_W = 0.60;

  /**
   * Configuration value for enemy regions max. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  static final int ENEMY_REGIONS_MAX = 24;

  /**
   * Configuration value for enemy region sigma. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  static final double ENEMY_REGION_SIGMA = 0.85;

  /**
   * Configuration value for res overlap r. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double RES_OVERLAP_R = 0.75;

  /**
   * Configuration value for res overlap gain. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final double RES_OVERLAP_GAIN = 1.05;

  /**
   * Configuration value for peak finder topn. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  static final int PEAK_FINDER_TOPN = 28;

  /**
   * Configuration value for scratch logits. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  double[] scratchLogits = new double[0];

  /**
   * Configuration value for scratch logits2. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  double[] scratchLogits2 = new double[0];

  private final PredictiveSpatialDynCache spatialDynCache = new PredictiveSpatialDynCache();

  /**
   * Configuration value for last dyn ref. Cache ownership lives in {@link
   * PredictiveSpatialDynCache}; this mirror is retained for package-private diagnostics/backwards
   * compatibility.
   */
  volatile List<DynamicObject> lastDynRef = null;

  /**
   * Configuration value for last dyn. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  volatile SpatialDyn lastDyn = null;

  /**
   * Configuration value for specs version. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public volatile int specsVersion = 0;

  /**
   * Configuration value for last dyn specs version. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  volatile int lastDynSpecsVersion = -1;

  /**
   * Configuration value for last footprint dyn. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  volatile SpatialDyn lastFootprintDyn = null;

  final HashMap<Long, Boolean> footprintCache = new HashMap<>(512);

  /**
   * Runs ensure scratch in the Repulsor runtime.
   *
   * @param n value used by this operation.
   */
  void ensureScratch(int n) {
    if (scratchLogits.length < n)
      scratchLogits = new double[Math.max(n, scratchLogits.length * 2 + 8)];
    if (scratchLogits2.length < n)
      scratchLogits2 = new double[Math.max(n, scratchLogits2.length * 2 + 8)];
  }

  /**
   * Returns the cached dyn value maintained by this Repulsor component.
   *
   * @return spatial dyn result for cached dyn.
   */
  public SpatialDyn cachedDyn() {
    SpatialDyn dyn = spatialDynCache.cached(this);
    lastDynRef = dynamicObjects;
    lastDyn = dyn;
    lastDynSpecsVersion = specsVersion;
    return dyn;
  }

  /** Runs invalidate dyn cache in the Repulsor runtime. */
  public void invalidateDynCache() {
    spatialDynCache.invalidate();
    lastDynRef = null;
    lastDyn = null;
    lastDynSpecsVersion = -1;
    lastFootprintDyn = null;
    footprintCache.clear();
  }

  public final HashMap<Integer, Track> allyMap = new HashMap<>();
  public final HashMap<Integer, Track> enemyMap = new HashMap<>();

  public volatile List<GameElement> worldElements = List.of();

  /**
   * Configuration value for our alliance. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public volatile Alliance ourAlliance = Alliance.kBlue;

  /**
   * Configuration value for last chosen. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public volatile RepulsorSetpoint lastChosen = null;

  /**
   * Configuration value for last chosen ts. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public volatile double lastChosenTs = 0.0;

  public volatile List<DynamicObject> dynamicObjects = List.of();
  public final HashMap<String, ResourceSpec> resourceSpecs = new HashMap<>();
  public final HashMap<String, Double> otherTypeWeights = new HashMap<>();

  /** Profile-driven collection/freshness configuration used by the predictive spatial snapshot. */
  public volatile ResourceCollectionProfile collectionProfile =
      ResourceCollectionProfile.fuel2026(Constants.FIELD_GEOMETRY);

  /**
   * Configuration value for collect resource types. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public final HashSet<String> collectResourceTypes =
      new HashSet<>(Set.of(DEFAULT_COLLECT_RESOURCE_TYPE));

  /**
   * Configuration value for collect resource position filter. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile Predicate<Translation2d> collectResourcePositionFilter =
      collectionProfile::accepts;

  /**
   * Configuration value for penalty tracker. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public final PredictiveCollectPenaltyTracker penaltyTracker =
      new PredictiveCollectPenaltyTracker();

  /**
   * Configuration value for last returned collect. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public volatile Translation2d lastReturnedCollect = null;

  /**
   * Configuration value for last returned collect ts. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public volatile double lastReturnedCollectTs = 0.0;

  /**
   * Configuration value for current collect target. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public volatile Translation2d currentCollectTarget = null;

  /**
   * Configuration value for current collect chosen ts. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public volatile double currentCollectChosenTs = 0.0;

  /**
   * Configuration value for current collect score. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public volatile double currentCollectScore = -1e18;

  /**
   * Configuration value for current collect units. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double currentCollectUnits = 0.0;

  /**
   * Configuration value for current collect eta. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double currentCollectEta = 0.0;

  /**
   * Configuration value for collect progress last ts. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public double collectProgressLastTs = 0.0;

  /**
   * Configuration value for collect progress last dist. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public double collectProgressLastDist = Double.POSITIVE_INFINITY;

  /**
   * Configuration value for collect arrival ts. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double collectArrivalTs = -1.0;

  /**
   * Configuration value for last our pos for collect. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public volatile Translation2d lastOurPosForCollect = null;

  /**
   * Configuration value for last our cap for collect. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public double lastOurCapForCollect = DEFAULT_OUR_SPEED;

  /**
   * Configuration value for last goal units for collect. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public int lastGoalUnitsForCollect = 1;

  /**
   * Configuration value for last cell mfor collect. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public double lastCellMForCollect = COLLECT_CELL_M;

  /**
   * Configuration value for secondary collect api. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  final PredictiveCollectSecondaryRankers.Api secondaryCollectApi =
      new PredictiveSecondaryCollectApi(this);

  /**
   * Updates register resource spec state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param type value used by this operation.
   * @param spec value used by this operation.
   */
  public void registerResourceSpec(String type, ResourceSpec spec) {
    PredictiveCollectConfigRuntime.registerResourceSpec(this, type, spec);
  }

  /**
   * Installs a profile-driven collection model for the predictive resource state. This updates the
   * default collectable type, registers its {@link ResourceSpec}, replaces the default position
   * filter with the profile bounds/exclusion filter, and invalidates cached spatial state.
   *
   * @param profile collection profile for the active game or field variant
   */
  public void configureCollectionProfile(ResourceCollectionProfile profile) {
    PredictiveCollectConfigRuntime.configureCollectionProfile(this, profile);
  }

  /**
   * Replaces only the field geometry used by predictive filters and wall-cost helpers.
   *
   * @param geometry field dimensions in WPILib field-relative meters
   */
  public void setFieldGeometry(FieldGeometry geometry) {
    PredictiveCollectConfigRuntime.setFieldGeometry(this, geometry);
  }

  /**
   * Returns the field geometry currently used by predictive collection filters.
   *
   * @return active predictive field geometry
   */
  public FieldGeometry getFieldGeometry() {
    return collectionProfile.fieldGeometry();
  }

  /**
   * Updates register other type weight state or telemetry as part of the Repulsor runtime loop.
   * This may mutate local state, NetworkTables output, planner caches, or command-side runtime
   * state depending on the owning type.
   *
   * @param type value used by this operation.
   * @param weight value used by this operation.
   */
  public void registerOtherTypeWeight(String type, double weight) {
    PredictiveCollectConfigRuntime.registerOtherTypeWeight(this, type, weight);
  }

  /**
   * Updates set collect resource types state or telemetry as part of the Repulsor runtime loop.
   * This may mutate local state, NetworkTables output, planner caches, or command-side runtime
   * state depending on the owning type.
   *
   * @param types value used by this operation.
   */
  public void setCollectResourceTypes(Set<String> types) {
    PredictiveCollectConfigRuntime.setCollectResourceTypes(this, types);
  }

  /**
   * Returns the get collect resource types value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Set<String> getCollectResourceTypes() {
    return PredictiveCollectConfigRuntime.getCollectResourceTypes(this);
  }

  /**
   * Runs add collect resource type in the Repulsor runtime.
   *
   * @param type value used by this operation.
   */
  public void addCollectResourceType(String type) {
    PredictiveCollectConfigRuntime.addCollectResourceType(this, type);
  }

  /**
   * Runs remove collect resource type in the Repulsor runtime.
   *
   * @param type value used by this operation.
   */
  public void removeCollectResourceType(String type) {
    PredictiveCollectConfigRuntime.removeCollectResourceType(this, type);
  }

  /**
   * Returns the is collect resource type value maintained by this Repulsor component.
   *
   * @param type value used by this operation.
   * @return value produced by this operation.
   */
  public boolean isCollectResourceType(String type) {
    return PredictiveCollectConfigRuntime.isCollectResourceType(this, type);
  }

  /**
   * Updates set collect resource position filter state or telemetry as part of the Repulsor runtime
   * loop. This may mutate local state, NetworkTables output, planner caches, or command-side
   * runtime state depending on the owning type.
   *
   * @param filter value used by this operation.
   */
  public void setCollectResourcePositionFilter(Predicate<Translation2d> filter) {
    PredictiveCollectConfigRuntime.setCollectResourcePositionFilter(this, filter);
  }

  /**
   * Updates set dynamic objects state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param objs value used by this operation.
   */
  public void setDynamicObjects(List<DynamicObject> objs) {
    PredictiveCollectConfigRuntime.setDynamicObjects(this, objs);
  }

  /**
   * Updates set world state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param elements value used by this operation.
   * @param ours value used by this operation.
   */
  public void setWorld(List<GameElement> elements, Alliance ours) {
    PredictiveCollectConfigRuntime.setWorld(this, elements, ours);
  }

  /**
   * Updates update ally state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param id value used by this operation.
   * @param pos value used by this operation.
   * @param velHint value used by this operation.
   * @param speedCap value used by this operation.
   */
  public void updateAlly(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    PredictiveFieldStateTrackingRuntime.updateAlly(this, id, pos, velHint, speedCap);
  }

  /**
   * Updates update enemy state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param id value used by this operation.
   * @param pos value used by this operation.
   * @param velHint value used by this operation.
   * @param speedCap value used by this operation.
   */
  public void updateEnemy(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    PredictiveFieldStateTrackingRuntime.updateEnemy(this, id, pos, velHint, speedCap);
  }

  /**
   * Updates clear stale state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param maxAgeS value used by this operation.
   */
  public void clearStale(double maxAgeS) {
    PredictiveFieldStateTrackingRuntime.clearStale(this, maxAgeS);
  }

  /**
   * Returns the rank value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param cat value used by this operation.
   * @param limit value used by this operation.
   * @return value produced by this operation.
   */
  public List<Candidate> rank(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return PredictiveFieldStateTrackingRuntime.rank(this, ourPos, ourSpeedCap, cat, limit);
  }

  /**
   * Runs sort idx by key in the Repulsor runtime.
   *
   * @param key distance or field-coordinate value in meters.
   * @param idx distance or field-coordinate value in meters.
   */
  static void sortIdxByKey(double[] key, int[] idx) {
    PredictiveFieldStateTrackingRuntime.sortIdxByKey(key, idx);
  }

  /**
   * Runs quick sort idx in the Repulsor runtime.
   *
   * @param key distance or field-coordinate value in meters.
   * @param idx distance or field-coordinate value in meters.
   * @param lo value used by this operation.
   * @param hi value used by this operation.
   */
  static void quickSortIdx(double[] key, int[] idx, int lo, int hi) {
    PredictiveFieldStateTrackingRuntime.quickSortIdx(key, idx, lo, hi);
  }

  /**
   * Returns the rank setpoints value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param cat value used by this operation.
   * @param limit value used by this operation.
   * @return list of repulsor setpoint values produced by this operation.
   */
  public List<RepulsorSetpoint> rankSetpoints(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return PredictiveFieldStateTrackingRuntime.rankSetpoints(this, ourPos, ourSpeedCap, cat, limit);
  }

  /**
   * Returns the resource observation count value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public int resourceObservationCount() {
    return PredictiveFieldStateTrackingRuntime.resourceObservationCount(this);
  }

  /**
   * Returns the snap to collect centroid value maintained by this Repulsor component.
   *
   * @param seed value used by this operation.
   * @param r value used by this operation.
   * @param minMass value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d snapToCollectCentroid(Translation2d seed, double r, double minMass) {
    return PredictiveFieldStateTrackingRuntime.snapToCollectCentroid(this, seed, r, minMass);
  }

  /**
   * Computes the nearest collect resource value for the current Repulsor planning state.
   *
   * @param p value used by this operation.
   * @param maxDist value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d nearestCollectResource(Translation2d p, double maxDist) {
    return PredictiveFieldStateTrackingRuntime.nearestCollectResource(this, p, maxDist);
  }

  /**
   * Summarizes collectable resources inside a profile-defined or caller-supplied region. Strategy
   * evaluators use this to compare scoring-side, center-field, contested, and fallback regions
   * without knowing the current game piece type.
   *
   * @param id stable telemetry identifier for the summarized region
   * @param regionFilter field-relative predicate selecting resources in the region; {@code null}
   *     accepts every collectable resource
   * @param robotPos current robot position in field-relative meters for distance and risk estimates
   * @param maxDistanceMeters maximum robot-to-resource distance considered actionable
   * @return normalized resource amount, nearest point, and traffic/obstacle risk for the region
   */
  public ResourceRegionSummary summarizeResourceRegion(
      String id,
      Predicate<Translation2d> regionFilter,
      Translation2d robotPos,
      double maxDistanceMeters) {
    return PredictiveFieldStateTrackingRuntime.summarizeResourceRegion(
        this, id, regionFilter, robotPos, maxDistanceMeters);
  }

  /**
   * Summarizes a region using the full field diagonal as the actionable distance bound.
   *
   * @param id stable telemetry identifier for the summarized region
   * @param regionFilter field-relative predicate selecting resources in the region
   * @param robotPos current robot position in field-relative meters
   * @return normalized resource amount, nearest point, and traffic/obstacle risk for the region
   */
  public ResourceRegionSummary summarizeResourceRegion(
      String id, Predicate<Translation2d> regionFilter, Translation2d robotPos) {
    return summarizeResourceRegion(id, regionFilter, robotPos, getFieldGeometry().diagonalMeters());
  }

  /**
   * Returns the peak finder value maintained by this Repulsor component.
   *
   * @param gridPoints value used by this operation.
   * @param dyn value used by this operation.
   * @param topN value used by this operation.
   * @return value produced by this operation.
   */
  Translation2d[] peakFinder(Translation2d[] gridPoints, SpatialDyn dyn, int topN) {
    return PredictiveCollectCandidateBuilder.peakFinder(gridPoints, dyn, topN);
  }

  /**
   * Configuration value for collect intake. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public static final Supplier<IntakeFootprint> COLLECT_INTAKE = IntakeFootprint::getFootprint;

  /**
   * Configuration value for collect footprint samples. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final Translation2d[] COLLECT_FOOTPRINT_SAMPLES =
      new Translation2d[] {
        new Translation2d(0.0, 0.0),
        new Translation2d(0.24, 0.0),
        new Translation2d(-0.24, 0.0),
        new Translation2d(0.0, 0.24),
        new Translation2d(0.0, -0.24),
        new Translation2d(0.18, 0.18),
        new Translation2d(0.18, -0.18),
        new Translation2d(-0.18, 0.18),
        new Translation2d(-0.18, -0.18)
      };

  /**
   * Configuration value for collect keep bonus. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final double COLLECT_KEEP_BONUS = 0.20;

  public Rotation2d currentCollectHeading = new Rotation2d();

  /**
   * Configuration value for current collect touch. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public volatile Translation2d currentCollectTouch = null;

  /**
   * Returns the face value maintained by this Repulsor component.
   *
   * @param from value used by this operation.
   * @param to value used by this operation.
   * @param fallback value used by this operation.
   * @return value produced by this operation.
   */
  public static Rotation2d face(Translation2d from, Translation2d to, Rotation2d fallback) {
    return PredictiveCollectFootprintRuntime.face(from, to, fallback);
  }

  /**
   * Returns the resolve collect touch value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param center value used by this operation.
   * @param heading value used by this operation.
   * @param rCore value used by this operation.
   * @param rSnap value used by this operation.
   * @param rCentroid value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d resolveCollectTouch(
      SpatialDyn dyn,
      Translation2d center,
      Rotation2d heading,
      double rCore,
      double rSnap,
      double rCentroid) {
    return PredictiveCollectFootprintRuntime.resolveCollectTouch(
        this, dyn, center, heading, rCore, rSnap, rCentroid);
  }

  /**
   * Returns the eval footprint value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param center value used by this operation.
   * @param heading value used by this operation.
   * @param rCore value used by this operation.
   * @return footprint eval result for eval footprint.
   */
  FootprintEval evalFootprint(
      SpatialDyn dyn, Translation2d center, Rotation2d heading, double rCore) {
    return PredictiveCollectFootprintRuntime.evalFootprint(this, dyn, center, heading, rCore);
  }

  /**
   * Runs fill footprint evidence in the Repulsor runtime.
   *
   * @param dyn value used by this operation.
   * @param center value used by this operation.
   * @param heading value used by this operation.
   * @param e value used by this operation.
   */
  void fillFootprintEvidence(
      SpatialDyn dyn, Translation2d center, Rotation2d heading, FootprintEval e) {
    PredictiveCollectFootprintRuntime.fillFootprintEvidence(this, dyn, center, heading, e);
  }

  /**
   * Returns the footprint ok value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param center value used by this operation.
   * @param heading value used by this operation.
   * @param rCore value used by this operation.
   * @param minUnits value used by this operation.
   * @return value produced by this operation.
   */
  public boolean footprintOk(
      SpatialDyn dyn, Translation2d center, Rotation2d heading, double rCore, double minUnits) {
    return PredictiveCollectFootprintRuntime.footprintOk(
        this, dyn, center, heading, rCore, minUnits);
  }

  /**
   * Returns the best heading for footprint value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param desiredCenter value used by this operation.
   * @param fuelTouch value used by this operation.
   * @param baseHeading value used by this operation.
   * @param rCore value used by this operation.
   * @param minUnits value used by this operation.
   * @return heading pick result for best heading for footprint.
   */
  public HeadingPick bestHeadingForFootprint(
      SpatialDyn dyn,
      Translation2d desiredCenter,
      Translation2d fuelTouch,
      Rotation2d baseHeading,
      double rCore,
      double minUnits) {
    return PredictiveCollectFootprintRuntime.bestHeadingForFootprint(
        this, dyn, desiredCenter, fuelTouch, baseHeading, rCore, minUnits);
  }

  /**
   * Returns the compare footprint evidence value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param p value used by this operation.
   * @param h value used by this operation.
   * @param fp value used by this operation.
   * @param best value used by this operation.
   * @return value produced by this operation.
   */
  boolean compareFootprintEvidence(
      SpatialDyn dyn, Translation2d p, Rotation2d h, FootprintEval fp, HeadingPick best) {
    return PredictiveCollectFootprintRuntime.compareFootprintEvidence(this, dyn, p, h, fp, best);
  }

  /**
   * Returns the footprint key value maintained by this Repulsor component.
   *
   * @param center value used by this operation.
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  static long footprintKey(Translation2d center, double cellM) {
    return PredictiveCollectFootprintRuntime.footprintKey(center, cellM);
  }

  /**
   * Returns the rank collect nearest value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param points value used by this operation.
   * @param cellM value used by this operation.
   * @param goalUnits value used by this operation.
   * @param limit value used by this operation.
   * @return point candidate result for rank collect nearest.
   */
  public PointCandidate rankCollectNearest(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int limit) {

    if (ourPos == null) return null;

    SpatialDyn dyn = cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return null;

    final double robotHalf = 0.5 * Math.max(OFFLOAD_SAFE_ROBOT_X_M, OFFLOAD_SAFE_ROBOT_Y_M);
    final double robotWallMargin = robotHalf + 0.03;
    final double wallClearMin = robotWallMargin + 0.05;
    final FieldGeometry geometry = getFieldGeometry();
    final java.util.function.ToDoubleFunction<Translation2d> wallPenalty =
        (pt) -> {
          if (pt == null) return 0.0;
          double d = wallDistanceForCollection(pt);
          if (d >= wallClearMin) return 0.0;
          double t = (wallClearMin - d) / Math.max(1e-6, wallClearMin);
          return 1.4 * t * t;
        };

    final java.util.function.Predicate<Translation2d> inShootBand =
        (pt) -> {
          if (pt == null) return true;
          double x = pt.getX();
          double y = pt.getY();
          return x < robotWallMargin
              || x > (geometry.lengthMeters() - robotWallMargin)
              || y < robotWallMargin
              || y > (geometry.widthMeters() - robotWallMargin);
        };

    lastOurPosForCollect = ourPos;
    lastOurCapForCollect = ourSpeedCap > 0.0 ? ourSpeedCap : DEFAULT_OUR_SPEED;
    lastGoalUnitsForCollect = Math.max(1, goalUnits);
    lastCellMForCollect = Math.max(0.10, cellM);

    sweepDepletedMarks();

    Translation2d[] seeds = buildCollectCandidates(points, dyn);
    if (seeds.length == 0) return null;

    double cap = lastOurCapForCollect;
    int goal = lastGoalUnitsForCollect;
    double half = Math.max(0.05, cellM * 0.5);

    double totalEv = dyn.totalEvidence();
    double minUnits = dynamicMinUnits(totalEv);
    int minCount = dynamicMinCount(totalEv);
    double minEv = minEvidence(totalEv);

    double nearHalf = Math.max(0.16, Math.max(half, cellM * 0.85));
    double nearR = Math.max(COLLECT_ARRIVE_R, 0.60);

    double onHalf = Math.max(0.10, Math.min(nearHalf * 0.60, 0.22));
    double onR = Math.max(0.20, Math.min(0.30, Math.max(0.20, COLLECT_ARRIVE_R * 0.70)));

    double rCore = coreRadiusFor(cellM);
    double rSnap = snapRadiusFor(cellM);
    double rCentroid = microCentroidRadiusFor(cellM);
    double rJitter = jitterRadiusFor(cellM);

    double minHardUnits = Math.max(0.025, Math.min(COLLECT_FINE_MIN_UNITS, minUnits * 0.80));
    double footprintMinUnits = Math.max(minHardUnits, minUnits * 0.95);

    if (lastReturnedCollect != null) {
      double now = PredictiveClock.nowSeconds();
      double age = now - lastReturnedCollectTs;
      if (age >= 0.0
          && age <= 0.55
          && ourPos.getDistance(lastReturnedCollect) <= DEPLETED_MARK_NEAR_M) {
        double u = dyn.valueAt(lastReturnedCollect);
        if (u < DEPLETED_MARK_EMPTY_UNITS) {
          addDepletedMark(lastReturnedCollect, 0.65, 1.35, DEPLETED_TTL_S, false);
          addDepletedRing(lastReturnedCollect, 0.35, 0.95, 0.85, DEPLETED_TTL_S);
        }
      }
    }

    int n = seeds.length;
    double[] etaKey = new double[n];
    double[] coarseKey = new double[n];
    int[] orderEta = new int[n];
    int[] orderCoarse = new int[n];

    ResourceRegions enemyRegions = buildResourceRegions(dyn, ENEMY_REGIONS_MAX);
    IntentAggCont enemyIntent = enemyIntentToRegions(enemyMap, enemyRegions);
    IntentAggCont allyIntent = allyIntentToRegions(allyMap, enemyRegions);

    for (int i = 0; i < n; i++) {
      Translation2d p = seeds[i];
      orderEta[i] = i;
      orderCoarse[i] = i;

      if (p == null || inShootBand.test(p)) {
        etaKey[i] = Double.POSITIVE_INFINITY;
        coarseKey[i] = 1e18;
        continue;
      }

      double eta = estimateTravelTime(ourPos, p, cap);
      etaKey[i] = eta;

      double regionUnits = dyn.valueInSquare(p, half);
      double dep = depletedPenaltySoft(p);
      double ev = dyn.evidenceMassWithin(p, EVIDENCE_R);
      double activity =
          Math.min(
              ACTIVITY_CAP,
              COLLECT_ACTIVITY_ALLY_W * radialDensity(allyMap, p, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_ENEMY_W * radialDensity(enemyMap, p, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_DYN_W * dyn.otherDensity(p, COLLECT_ACTIVITY_SIGMA));
      double wallPen = wallPenalty.applyAsDouble(p);
      int coarseCore = dyn.countResourcesWithin(p, Math.max(0.04, rCore));

      double evGate = ev < minEv ? -2.25 : 0.0;
      double coarseScore =
          regionUnits * COLLECT_REGION_SAMPLES_W
              - eta * 0.55
              - activity * 0.70
              - dep * DEPLETED_PEN_W
              - wallPen
              + evGate
              + Math.min(0.6, 0.25 * coarseCore);

      coarseKey[i] = -coarseScore;
    }

    sortIdxByKey(etaKey, orderEta);
    sortIdxByKey(coarseKey, orderCoarse);

    int shortlistCap =
        Math.max(
            12,
            Math.min(
                n,
                Math.max(
                    Math.min(COLLECT_SHORTLIST_ETA, n), Math.min(COLLECT_SHORTLIST_COARSE, n))));

    boolean[] chosen = new boolean[n];
    ArrayList<Integer> shortlist = new ArrayList<>(shortlistCap * 2);

    int kEta = Math.min(n, Math.max(1, Math.min(COLLECT_SHORTLIST_ETA, shortlistCap + 8)));
    int kCoarse = Math.min(n, Math.max(1, Math.min(COLLECT_SHORTLIST_COARSE, shortlistCap + 8)));

    for (int i = 0; i < kEta && shortlist.size() < shortlistCap; i++) {
      int idx = orderEta[i];
      if (!chosen[idx]) {
        chosen[idx] = true;
        shortlist.add(idx);
      }
    }
    for (int i = 0; i < kCoarse && shortlist.size() < shortlistCap; i++) {
      int idx = orderCoarse[i];
      if (!chosen[idx]) {
        chosen[idx] = true;
        shortlist.add(idx);
      }
    }

    int maxCheck =
        limit > 0 ? Math.min(Math.max(1, limit), Math.max(1, shortlist.size())) : shortlist.size();

    PredictiveCollectNearestSearchResult search =
        PredictiveCollectNearestSearchStep.search(
            this,
            seeds,
            shortlist,
            maxCheck,
            ourPos,
            cap,
            goal,
            cellM,
            dyn,
            enemyIntent,
            allyIntent,
            inShootBand,
            wallPenalty,
            nearHalf,
            nearR,
            onHalf,
            onR,
            minHardUnits,
            minUnits,
            minCount,
            minEv,
            rCore,
            rSnap,
            rCentroid,
            footprintMinUnits);

    Translation2d bestP = search.bestPoint();
    Translation2d bestTouch = search.bestTouch();
    Rotation2d bestHeading = search.bestHeading();
    CollectEval bestE = search.bestEval();

    if (bestP == null || bestE == null || bestTouch == null || inShootBand.test(bestP)) {
      return null;
    }

    Translation2d chosenPt = bestP;
    Translation2d chosenTouch = bestTouch;
    Rotation2d chosenHeading = bestHeading;
    CollectEval chosenE = bestE;

    return PredictiveCollectNearestResolutionStep.resolve(
        this,
        ourPos,
        cap,
        goal,
        cellM,
        dyn,
        enemyIntent,
        allyIntent,
        inShootBand,
        chosenPt,
        chosenTouch,
        chosenHeading,
        chosenE,
        totalEv,
        minUnits,
        minCount,
        minEv,
        onHalf,
        onR,
        minHardUnits,
        footprintMinUnits,
        rCore,
        rSnap,
        rCentroid);
  }

  /**
   * Returns the rank collect hierarchical value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param points value used by this operation.
   * @param cellM value used by this operation.
   * @param goalUnits value used by this operation.
   * @param coarseTopK value used by this operation.
   * @param refineGrid value used by this operation.
   * @return point candidate result for rank collect hierarchical.
   */
  public PointCandidate rankCollectHierarchical(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int coarseTopK,
      int refineGrid) {
    return PredictiveCollectSecondaryRankers.rankCollectHierarchical(
        secondaryCollectApi, ourPos, ourSpeedCap, points, cellM, goalUnits, coarseTopK, refineGrid);
  }

  /**
   * Returns the best collect hotspot value maintained by this Repulsor component.
   *
   * @param points value used by this operation.
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d bestCollectHotspot(Translation2d[] points, double cellM) {
    return PredictiveCollectSecondaryRankers.bestCollectHotspot(secondaryCollectApi, points, cellM);
  }

  /**
   * Returns the rank collect points value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param points value used by this operation.
   * @param goalUnits value used by this operation.
   * @param limit value used by this operation.
   * @return point candidate result for rank collect points.
   */
  public PointCandidate rankCollectPoints(
      Translation2d ourPos, double ourSpeedCap, Translation2d[] points, int goalUnits, int limit) {
    return PredictiveCollectSecondaryRankers.rankCollectPoints(
        secondaryCollectApi, ourPos, ourSpeedCap, points, goalUnits, limit);
  }

  /**
   * Returns the build collect candidates value maintained by this Repulsor component.
   *
   * @param gridPoints value used by this operation.
   * @param dyn value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d[] buildCollectCandidates(Translation2d[] gridPoints, SpatialDyn dyn) {
    return PredictiveCollectCandidateBuilder.buildCollectCandidates(
        gridPoints, dyn, PEAK_FINDER_TOPN);
  }

  /**
   * Returns the dedup points value maintained by this Repulsor component.
   *
   * @param in value used by this operation.
   * @param dupSkipM value used by this operation.
   * @return array list of translation2d result for dedup points.
   */
  public ArrayList<Translation2d> dedupPoints(ArrayList<Translation2d> in, double dupSkipM) {
    return PredictiveCollectCandidateBuilder.dedupPoints(in, dupSkipM);
  }

  /**
   * Returns the spread collect points value maintained by this Repulsor component.
   *
   * @param in value used by this operation.
   * @param dyn value used by this operation.
   * @return array list of translation2d result for spread collect points.
   */
  public ArrayList<Translation2d> spreadCollectPoints(ArrayList<Translation2d> in, SpatialDyn dyn) {
    return PredictiveCollectCandidateBuilder.spreadCollectPoints(in, dyn);
  }

  /**
   * Returns the adaptive dup skip value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @return value produced by this operation.
   */
  public double adaptiveDupSkip(SpatialDyn dyn) {
    return PredictiveCollectCandidateBuilder.adaptiveDupSkip(dyn);
  }

  /**
   * Returns the adaptive collect separation value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @return value produced by this operation.
   */
  public double adaptiveCollectSeparation(SpatialDyn dyn) {
    return PredictiveCollectCandidateBuilder.adaptiveCollectSeparation(dyn);
  }

  /**
   * Returns the build resource clusters multi value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param maxClusters value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d[] buildResourceClustersMulti(SpatialDyn dyn, int maxClusters) {
    return PredictiveCollectCandidateBuilder.buildResourceClustersMulti(dyn, maxClusters);
  }

  /**
   * Returns the mean shift refine value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param seed value used by this operation.
   * @param r value used by this operation.
   * @param iters value used by this operation.
   * @return value produced by this operation.
   */
  Translation2d meanShiftRefine(SpatialDyn dyn, Translation2d seed, double r, int iters) {
    return PredictiveCollectCandidateBuilder.meanShiftRefine(dyn, seed, r, iters);
  }

  /** Runs sweep depleted marks in the Repulsor runtime. */
  public void sweepDepletedMarks() {
    penaltyTracker.sweepDepletedMarks(error());
  }

  /**
   * Runs add depleted mark in the Repulsor runtime.
   *
   * @param p value used by this operation.
   * @param radiusM value used by this operation.
   * @param strength value used by this operation.
   * @param ttlS value used by this operation.
   * @param merge value used by this operation.
   */
  public void addDepletedMark(
      Translation2d p, double radiusM, double strength, double ttlS, boolean merge) {
    penaltyTracker.addDepletedMark(p, radiusM, strength, ttlS, merge, error());
  }

  /**
   * Runs add depleted ring in the Repulsor runtime.
   *
   * @param p value used by this operation.
   * @param r0 value used by this operation.
   * @param r1 value used by this operation.
   * @param strength value used by this operation.
   * @param ttlS value used by this operation.
   */
  public void addDepletedRing(Translation2d p, double r0, double r1, double strength, double ttlS) {
    penaltyTracker.addDepletedRing(p, r0, r1, strength, ttlS, error());
  }

  /**
   * Returns the depleted penalty soft value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return value produced by this operation.
   */
  public double depletedPenaltySoft(Translation2d p) {
    return penaltyTracker.depletedPenaltySoft(p, error());
  }

  /**
   * Returns the lerp vec value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param alpha value used by this operation.
   * @return value produced by this operation.
   */
  public static Translation2d lerpVec(Translation2d a, Translation2d b, double alpha) {
    double t = Math.max(0.0, Math.min(1.0, alpha));
    return new Translation2d(
        a.getX() + (b.getX() - a.getX()) * t, a.getY() + (b.getY() - a.getY()) * t);
  }

  /**
   * Returns the ema alpha value maintained by this Repulsor component.
   *
   * @param baseAlpha value used by this operation.
   * @param dt value used by this operation.
   * @return value produced by this operation.
   */
  public static double emaAlpha(double baseAlpha, double dt) {
    double a = Math.max(0.0, Math.min(1.0, baseAlpha));
    double x = Math.max(0.0, dt / MIN_DT);
    return 1.0 - Math.pow(1.0 - a, x);
  }

  /**
   * Returns the clamp delta v value maintained by this Repulsor component.
   *
   * @param oldV value used by this operation.
   * @param newV value used by this operation.
   * @param aMax distance or field-coordinate value in meters.
   * @param dt value used by this operation.
   * @return value produced by this operation.
   */
  public static Translation2d clampDeltaV(
      Translation2d oldV, Translation2d newV, double aMax, double dt) {
    double lim = Math.max(0.0, aMax) * Math.max(0.0, dt);
    Translation2d dv = newV.minus(oldV);
    double n = dv.getNorm();
    if (n <= lim || n <= 1e-9) return newV;
    Translation2d dvClamped = dv.div(n).times(lim);
    return oldV.plus(dvClamped);
  }

  /**
   * Returns the eta path value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param speed value used by this operation.
   * @return value produced by this operation.
   */
  static double etaPath(Translation2d a, Translation2d b, double speed) {
    double d = a.getDistance(b);
    return Math.max(ETA_FLOOR, d / Math.max(0.1, speed));
  }

  /**
   * Returns the estimate travel time value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param speed value used by this operation.
   * @return value produced by this operation.
   */
  public double estimateTravelTime(Translation2d a, Translation2d b, double speed) {
    return PredictiveCollectDynamics.estimateTravelTime(
        a, b, speed, cachedDyn(), allyMap, enemyMap);
  }

  /**
   * Returns the min eta to target value maintained by this Repulsor component.
   *
   * @param map value used by this operation.
   * @param t value used by this operation.
   * @return value produced by this operation.
   */
  public static double minEtaToTarget(HashMap<Integer, Track> map, Translation2d t) {
    return PredictiveCollectDynamics.minEtaToTarget(map, t);
  }

  /**
   * Returns the dot norm value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  static double dotNorm(Translation2d a, Translation2d b) {
    return PredictiveCollectDynamics.dotNorm(a, b);
  }

  /**
   * Returns the radial kernel value maintained by this Repulsor component.
   *
   * @param dist value used by this operation.
   * @return value produced by this operation.
   */
  double radialKernel(double dist) {
    return PredictiveCollectDynamics.radialKernel(dist);
  }

  /**
   * Returns the density kernel value maintained by this Repulsor component.
   *
   * @param d value used by this operation.
   * @param sigma value used by this operation.
   * @return value produced by this operation.
   */
  static double densityKernel(double d, double sigma) {
    return PredictiveCollectDynamics.densityKernel(d, sigma);
  }

  /**
   * Returns the radial density value maintained by this Repulsor component.
   *
   * @param map value used by this operation.
   * @param target value used by this operation.
   * @param sigma value used by this operation.
   * @return value produced by this operation.
   */
  public static double radialDensity(
      HashMap<Integer, Track> map, Translation2d target, double sigma) {
    return PredictiveCollectDynamics.radialDensity(map, target, sigma);
  }

  /**
   * Returns the predict at value maintained by this Repulsor component.
   *
   * @param r value used by this operation.
   * @param horizonS value used by this operation.
   * @return value produced by this operation.
   */
  public static Translation2d predictAt(Track r, double horizonS) {
    return PredictiveCollectDynamics.predictAt(r, horizonS);
  }

  /**
   * Returns the radial pressure value maintained by this Repulsor component.
   *
   * @param enemies value used by this operation.
   * @param target value used by this operation.
   * @param ourEtaS value used by this operation.
   * @param intentMass value used by this operation.
   * @param count value used by this operation.
   * @return value produced by this operation.
   */
  public double radialPressure(
      HashMap<Integer, Track> enemies,
      Translation2d target,
      double ourEtaS,
      double intentMass,
      int count) {
    return PredictiveCollectDynamics.radialPressure(
        enemies, target, ourEtaS, RESERVATION_RADIUS, intentMass, count);
  }

  /**
   * Returns the radial congestion value maintained by this Repulsor component.
   *
   * @param allies value used by this operation.
   * @param target value used by this operation.
   * @param ourEtaS value used by this operation.
   * @param intentMass value used by this operation.
   * @param count value used by this operation.
   * @return value produced by this operation.
   */
  public double radialCongestion(
      HashMap<Integer, Track> allies,
      Translation2d target,
      double ourEtaS,
      double intentMass,
      int count) {
    return PredictiveCollectDynamics.radialCongestion(
        allies, target, ourEtaS, RESERVATION_RADIUS, intentMass, count);
  }

  /**
   * Returns the heading affinity value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param target value used by this operation.
   * @param allies value used by this operation.
   * @param enemies value used by this operation.
   * @return value produced by this operation.
   */
  public double headingAffinity(
      Translation2d ourPos,
      Translation2d target,
      HashMap<Integer, Track> allies,
      HashMap<Integer, Track> enemies) {
    return PredictiveCollectDynamics.headingAffinity(ourPos, target, allies, enemies);
  }

  /**
   * Returns the soft intent agg value maintained by this Repulsor component.
   *
   * @param map value used by this operation.
   * @param targets value used by this operation.
   * @return intent agg result for soft intent agg.
   */
  public IntentAgg softIntentAgg(HashMap<Integer, Track> map, List<Translation2d> targets) {
    return PredictiveCollectDynamics.softIntentAgg(map, targets);
  }

  /**
   * Returns the build resource regions value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param maxRegions value used by this operation.
   * @return resource regions result for build resource regions.
   */
  public ResourceRegions buildResourceRegions(SpatialDyn dyn, int maxRegions) {
    return PredictiveCollectDynamics.buildResourceRegions(dyn, maxRegions);
  }

  /**
   * Returns the enemy intent to regions value maintained by this Repulsor component.
   *
   * @param map value used by this operation.
   * @param regs value used by this operation.
   * @return intent agg cont result for enemy intent to regions.
   */
  public IntentAggCont enemyIntentToRegions(HashMap<Integer, Track> map, ResourceRegions regs) {
    return PredictiveCollectDynamics.enemyIntentToRegions(map, regs, ENEMY_REGION_SIGMA);
  }

  /**
   * Returns the ally intent to regions value maintained by this Repulsor component.
   *
   * @param map value used by this operation.
   * @param regs value used by this operation.
   * @return intent agg cont result for ally intent to regions.
   */
  public IntentAggCont allyIntentToRegions(HashMap<Integer, Track> map, ResourceRegions regs) {
    return PredictiveCollectDynamics.allyIntentToRegions(map, regs, ENEMY_REGION_SIGMA);
  }

  /**
   * Returns the clamp value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @param lo value used by this operation.
   * @param hi value used by this operation.
   * @return value produced by this operation.
   */
  static double clamp(double x, double lo, double hi) {
    return PredictiveCollectPlacementRuntime.clamp(x, lo, hi);
  }

  /**
   * Returns the core radius for value maintained by this Repulsor component.
   *
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public static double coreRadiusFor(double cellM) {
    return PredictiveCollectPlacementRuntime.coreRadiusFor(cellM);
  }

  /**
   * Returns the snap radius for value maintained by this Repulsor component.
   *
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public static double snapRadiusFor(double cellM) {
    return PredictiveCollectPlacementRuntime.snapRadiusFor(cellM);
  }

  /**
   * Returns the micro centroid radius for value maintained by this Repulsor component.
   *
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public static double microCentroidRadiusFor(double cellM) {
    return PredictiveCollectPlacementRuntime.microCentroidRadiusFor(cellM);
  }

  /**
   * Returns the jitter radius for value maintained by this Repulsor component.
   *
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public static double jitterRadiusFor(double cellM) {
    return PredictiveCollectPlacementRuntime.jitterRadiusFor(cellM);
  }

  /**
   * Returns the snap to nearest then micro centroid value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param dyn value used by this operation.
   * @param rSnap value used by this operation.
   * @param rCentroid value used by this operation.
   * @param minMass value used by this operation.
   * @return value produced by this operation.
   */
  Translation2d snapToNearestThenMicroCentroid(
      Translation2d p, SpatialDyn dyn, double rSnap, double rCentroid, double minMass) {
    return PredictiveCollectPlacementRuntime.snapToNearestThenMicroCentroid(
        this, p, dyn, rSnap, rCentroid, minMass);
  }

  /**
   * Returns the enforce hard stop on fuel value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param p value used by this operation.
   * @param rCore value used by this operation.
   * @param rSnap value used by this operation.
   * @param rCentroid value used by this operation.
   * @param minMass value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d enforceHardStopOnCollectResource(
      SpatialDyn dyn,
      Translation2d p,
      double rCore,
      double rSnap,
      double rCentroid,
      double minMass) {
    return PredictiveCollectPlacementRuntime.enforceHardStopOnCollectResource(
        this, dyn, p, rCore, rSnap, rCentroid, minMass);
  }

  /**
   * Compatibility wrapper for 2026 fuel-specific callers.
   *
   * @param dyn spatial dynamic-object snapshot
   * @param p candidate collect point in field-relative meters
   * @param rCore core resource-capture radius in meters
   * @param rSnap snap-search radius in meters
   * @param rCentroid centroid smoothing radius in meters
   * @param minMass minimum evidence mass needed for centroiding
   * @return adjusted collect point on a live resource, or {@code null} when no resource is present
   * @deprecated prefer {@link #enforceHardStopOnCollectResource(SpatialDyn, Translation2d, double,
   *     double, double, double)}
   */
  @Deprecated(forRemoval = false)
  public Translation2d enforceHardStopOnFuel(
      SpatialDyn dyn,
      Translation2d p,
      double rCore,
      double rSnap,
      double rCentroid,
      double minMass) {
    return enforceHardStopOnCollectResource(dyn, p, rCore, rSnap, rCentroid, minMass);
  }

  /**
   * Computes the pickup robust penalty value for the current Repulsor planning state. Call this
   * from periodic planning or tests when a fresh decision is required; inputs should already be
   * expressed in the coordinate frame expected by the parameter names.
   *
   * @param dyn value used by this operation.
   * @param p value used by this operation.
   * @param rCore value used by this operation.
   * @param jitterR value used by this operation.
   * @return value produced by this operation.
   */
  public double pickupRobustPenalty(SpatialDyn dyn, Translation2d p, double rCore, double jitterR) {
    return PredictiveCollectPlacementRuntime.pickupRobustPenalty(this, dyn, p, rCore, jitterR);
  }

  /**
   * Returns the eval collect point value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param cap value used by this operation.
   * @param p value used by this operation.
   * @param goal value used by this operation.
   * @param cellM value used by this operation.
   * @param dyn value used by this operation.
   * @param enemyIntent value used by this operation.
   * @param allyIntent value used by this operation.
   * @return collect eval result for eval collect point.
   */
  public CollectEval evalCollectPoint(
      Translation2d ourPos,
      double cap,
      Translation2d p,
      int goal,
      double cellM,
      SpatialDyn dyn,
      IntentAggCont enemyIntent,
      IntentAggCont allyIntent) {
    return PredictiveCollectScoringRuntime.evalCollectPoint(
        this, ourPos, cap, p, goal, cellM, dyn, enemyIntent, allyIntent);
  }

  /**
   * Returns the normalize value value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param value value used by this operation.
   * @return value produced by this operation.
   */
  double normalizeValue(SpatialDyn dyn, double value) {
    return PredictiveCollectScoringRuntime.normalizeValue(this, dyn, value);
  }

  /**
   * Returns the reservation overlap penalty value maintained by this Repulsor component.
   *
   * @param allies value used by this operation.
   * @param p value used by this operation.
   * @param ourEtaS value used by this operation.
   * @return value produced by this operation.
   */
  double reservationOverlapPenalty(
      HashMap<Integer, Track> allies, Translation2d p, double ourEtaS) {
    return PredictiveCollectScoringRuntime.reservationOverlapPenalty(this, allies, p, ourEtaS);
  }

  /**
   * Returns the should escape current collect value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param dyn value used by this operation.
   * @param totalEv value used by this operation.
   * @param minUnits value used by this operation.
   * @param minCount value used by this operation.
   * @param cellM value used by this operation.
   * @return value produced by this operation.
   */
  public boolean shouldEscapeCurrentCollect(
      Translation2d ourPos,
      SpatialDyn dyn,
      double totalEv,
      double minUnits,
      int minCount,
      double cellM) {
    return PredictiveCollectScoringRuntime.shouldEscapeCurrentCollect(
        this, ourPos, dyn, totalEv, minUnits, minCount, cellM);
  }

  /**
   * Returns the collect commit window value maintained by this Repulsor component.
   *
   * @param etaCurrent value used by this operation.
   * @return value produced by this operation.
   */
  public double collectCommitWindow(double etaCurrent) {
    return PredictiveCollectScoringRuntime.collectCommitWindow(this, etaCurrent);
  }

  /**
   * Returns the min evidence value maintained by this Repulsor component.
   *
   * @param totalEvidence value used by this operation.
   * @return value produced by this operation.
   */
  public double minEvidence(double totalEvidence) {
    return PredictiveCollectScoringRuntime.minEvidence(this, totalEvidence);
  }

  /**
   * Returns the dynamic min units value maintained by this Repulsor component.
   *
   * @param totalEvidence value used by this operation.
   * @return value produced by this operation.
   */
  public double dynamicMinUnits(double totalEvidence) {
    return PredictiveCollectScoringRuntime.dynamicMinUnits(this, totalEvidence);
  }

  /**
   * Returns the dynamic min count value maintained by this Repulsor component.
   *
   * @param totalEvidence value used by this operation.
   * @return value produced by this operation.
   */
  public int dynamicMinCount(double totalEvidence) {
    return PredictiveCollectScoringRuntime.dynamicMinCount(this, totalEvidence);
  }

  /**
   * Updates record region attempt state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param dyn value used by this operation.
   * @param p value used by this operation.
   * @param now value used by this operation.
   * @param success value used by this operation.
   */
  public void recordRegionAttempt(SpatialDyn dyn, Translation2d p, double now, boolean success) {
    PredictiveCollectScoringRuntime.recordRegionAttempt(this, dyn, p, now, success);
  }

  /**
   * Returns the region bandit bonus value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param p value used by this operation.
   * @param now value used by this operation.
   * @return value produced by this operation.
   */
  public double regionBanditBonus(SpatialDyn dyn, Translation2d p, double now) {
    return PredictiveCollectScoringRuntime.regionBanditBonus(this, dyn, p, now);
  }

  /**
   * Returns the lerp value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param t value used by this operation.
   * @return value produced by this operation.
   */
  static double lerp(double a, double b, double t) {
    return PredictiveCollectScoringRuntime.lerp(null, a, b, t);
  }

  /**
   * Returns the clamp01 value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  static double clamp01(double x) {
    return PredictiveCollectScoringRuntime.clamp01(null, x);
  }

  /**
   * Returns the wall distance value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return value produced by this operation.
   */
  static double wallDistance(Translation2d p) {
    return PredictiveCollectScoringRuntime.wallDistance(null, p);
  }

  /**
   * Computes distance to the nearest field wall using the configured predictive field geometry.
   *
   * @param p field-relative point in meters
   * @return nearest wall distance in meters, or zero for a null point
   */
  public double wallDistanceForCollection(Translation2d p) {
    return PredictiveCollectScoringRuntime.wallDistance(this, p);
  }

  /**
   * Returns the is invalid fuel band value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return value produced by this operation.
   */
  static boolean isInvalidFuelBand(Translation2d p) {
    return PredictiveCollectScoringRuntime.isInvalidFuelBand(null, p);
  }

  /**
   * Reports whether a point is excluded by the active collection profile.
   *
   * @param p field-relative point in meters
   * @return true when the point is inside one of the profile's excluded regions
   */
  public boolean isExcludedCollectResourceRegion(Translation2d p) {
    if (p == null) return false;
    for (Predicate<Translation2d> region : collectionProfile.excludedRegions()) {
      if (region != null && region.test(p)) return true;
    }
    return false;
  }

  /**
   * Returns the default collect resource position filter value maintained by this Repulsor
   * component.
   *
   * @param p value used by this operation.
   * @return value produced by this operation.
   */
  public static boolean defaultCollectResourcePositionFilter(Translation2d p) {
    return PredictiveCollectScoringRuntime.defaultCollectResourcePositionFilter(null, p);
  }
}
