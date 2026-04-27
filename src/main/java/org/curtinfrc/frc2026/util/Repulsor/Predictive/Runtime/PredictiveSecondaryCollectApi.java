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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Runtime;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.ResourceRegions;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveCollectSecondaryRankers;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.SpatialDyn;

/**
 * Provides predictive secondary collect api functionality for the Repulsor runtime helper layer
 * shared by behaviours and planners. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class PredictiveSecondaryCollectApi implements PredictiveCollectSecondaryRankers.Api {
  private final PredictiveFieldStateOps ops;

  /**
   * Returns the predictive secondary collect api value maintained by this Repulsor component.
   *
   * @param ops value used by this operation.
   */
  public PredictiveSecondaryCollectApi(PredictiveFieldStateOps ops) {
    this.ops = ops;
  }

  /**
   * Returns the cached dyn value maintained by this Repulsor component.
   *
   * @return spatial dyn result for cached dyn.
   */
  @Override
  public SpatialDyn cachedDyn() {
    return ops.cachedDyn();
  }

  /**
   * Updates set collect context state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param goalUnits value used by this operation.
   * @param cellM value used by this operation.
   */
  @Override
  public void setCollectContext(
      Translation2d ourPos, double ourSpeedCap, int goalUnits, double cellM) {
    ops.lastOurPosForCollect = ourPos;
    ops.lastOurCapForCollect =
        ourSpeedCap > 0.0 ? ourSpeedCap : PredictiveFieldStateOps.DEFAULT_OUR_SPEED;
    ops.lastGoalUnitsForCollect = Math.max(1, goalUnits);
    ops.lastCellMForCollect = Math.max(0.10, cellM);
  }

  /** Runs sweep depleted marks in the Repulsor runtime. */
  @Override
  public void sweepDepletedMarks() {
    ops.sweepDepletedMarks();
  }

  /**
   * Returns the build collect candidates value maintained by this Repulsor component.
   *
   * @param gridPoints value used by this operation.
   * @param dyn value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public Translation2d[] buildCollectCandidates(Translation2d[] gridPoints, SpatialDyn dyn) {
    return ops.buildCollectCandidates(gridPoints, dyn);
  }

  /**
   * Returns the dynamic min units value maintained by this Repulsor component.
   *
   * @param totalEvidence value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public double dynamicMinUnits(double totalEvidence) {
    return ops.dynamicMinUnits(totalEvidence);
  }

  /**
   * Returns the dynamic min count value maintained by this Repulsor component.
   *
   * @param totalEvidence value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public int dynamicMinCount(double totalEvidence) {
    return ops.dynamicMinCount(totalEvidence);
  }

  /**
   * Returns the min evidence value maintained by this Repulsor component.
   *
   * @param totalEvidence value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public double minEvidence(double totalEvidence) {
    return ops.minEvidence(totalEvidence);
  }

  /**
   * Returns the build resource regions value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param maxRegions value used by this operation.
   * @return resource regions result for build resource regions.
   */
  @Override
  public ResourceRegions buildResourceRegions(SpatialDyn dyn, int maxRegions) {
    return ops.buildResourceRegions(dyn, maxRegions);
  }

  /**
   * Returns the enemy intent to regions value maintained by this Repulsor component.
   *
   * @param regs value used by this operation.
   * @return intent agg cont result for enemy intent to regions.
   */
  @Override
  public IntentAggCont enemyIntentToRegions(ResourceRegions regs) {
    return ops.enemyIntentToRegions(ops.enemyMap, regs);
  }

  /**
   * Returns the ally intent to regions value maintained by this Repulsor component.
   *
   * @param regs value used by this operation.
   * @return intent agg cont result for ally intent to regions.
   */
  @Override
  public IntentAggCont allyIntentToRegions(ResourceRegions regs) {
    return ops.allyIntentToRegions(ops.allyMap, regs);
  }

  /**
   * Returns the estimate travel time value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param speed value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public double estimateTravelTime(Translation2d a, Translation2d b, double speed) {
    return ops.estimateTravelTime(a, b, speed);
  }

  /**
   * Returns the eval collect point value maintained by this Repulsor component.
   *
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param p value used by this operation.
   * @param goalUnits value used by this operation.
   * @param cellM value used by this operation.
   * @param dyn value used by this operation.
   * @param enemyIntent value used by this operation.
   * @param allyIntent value used by this operation.
   * @return collect eval result for eval collect point.
   */
  @Override
  public CollectEval evalCollectPoint(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d p,
      int goalUnits,
      double cellM,
      SpatialDyn dyn,
      IntentAggCont enemyIntent,
      IntentAggCont allyIntent) {
    return ops.evalCollectPoint(
        ourPos, ourSpeedCap, p, goalUnits, cellM, dyn, enemyIntent, allyIntent);
  }

  /**
   * Returns the ally radial density value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param sigma value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public double allyRadialDensity(Translation2d p, double sigma) {
    return PredictiveFieldStateOps.radialDensity(ops.allyMap, p, sigma);
  }

  /**
   * Returns the enemy radial density value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param sigma value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public double enemyRadialDensity(Translation2d p, double sigma) {
    return PredictiveFieldStateOps.radialDensity(ops.enemyMap, p, sigma);
  }

  /**
   * Returns the region bandit bonus value maintained by this Repulsor component.
   *
   * @param dyn value used by this operation.
   * @param p value used by this operation.
   * @param now value used by this operation.
   * @return value produced by this operation.
   */
  @Override
  public double regionBanditBonus(SpatialDyn dyn, Translation2d p, double now) {
    return ops.regionBanditBonus(dyn, p, now);
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
  @Override
  public void addDepletedMark(
      Translation2d p, double radiusM, double strength, double ttlS, boolean merge) {
    ops.addDepletedMark(p, radiusM, strength, ttlS, merge);
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
  @Override
  public void addDepletedRing(Translation2d p, double r0, double r1, double strength, double ttlS) {
    ops.addDepletedRing(p, r0, r1, strength, ttlS);
  }

  /**
   * Updates set last returned collect state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param p value used by this operation.
   * @param nowS value used by this operation.
   */
  @Override
  public void setLastReturnedCollect(Translation2d p, double nowS) {
    ops.lastReturnedCollect = p;
    ops.lastReturnedCollectTs = nowS;
  }
}
