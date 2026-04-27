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

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.HashMap;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.DepletedMark;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.RegionStat;

/**
 * Provides predictive collect penalty tracker functionality for the Repulsor predictive field-state
 * and collection-planning layer. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class PredictiveCollectPenaltyTracker {
  private static final double DEPLETED_TTL_S = 3.25;
  private static final double DEPLETED_DECAY = 1.15;
  private static final int DEPLETED_MARKS_MAX = 128;
  private static final int REGION_STATS_MAX = 512;

  private final ArrayList<DepletedMark> depletedMarks = new ArrayList<>(256);
  private final HashMap<Long, RegionStat> regionStats = new HashMap<>(512);

  /**
   * Runs sweep depleted marks in the Repulsor runtime.
   *
   * @param errorMode value used by this operation.
   */
  void sweepDepletedMarks(boolean errorMode) {
    if (errorMode) return;
    if (depletedMarks.isEmpty()) return;
    double now = PredictiveClock.nowSeconds();
    depletedMarks.removeIf(m -> now - m.t > m.ttl);
    if (depletedMarks.size() > DEPLETED_MARKS_MAX) {
      while (depletedMarks.size() > DEPLETED_MARKS_MAX) depletedMarks.remove(0);
    }
  }

  /**
   * Runs add depleted mark in the Repulsor runtime.
   *
   * @param p value used by this operation.
   * @param radiusM value used by this operation.
   * @param strength value used by this operation.
   * @param ttlS value used by this operation.
   * @param merge value used by this operation.
   * @param errorMode value used by this operation.
   */
  void addDepletedMark(
      Translation2d p,
      double radiusM,
      double strength,
      double ttlS,
      boolean merge,
      boolean errorMode) {
    if (errorMode) return;
    if (p == null) return;

    double now = PredictiveClock.nowSeconds();
    double r = Math.max(0.10, radiusM);
    double s = Math.max(0.0, strength);
    double ttl = Math.max(0.1, ttlS);

    if (merge) {
      for (int i = depletedMarks.size() - 1; i >= 0; i--) {
        DepletedMark m = depletedMarks.get(i);
        if (m == null || m.ring) continue;
        if (m.p.getDistance(p) <= 0.30) {
          m.t = now;
          m.s = Math.max(m.s, s);
          m.r = Math.max(m.r, r);
          m.ttl = Math.max(m.ttl, ttl);
          return;
        }
      }
    }

    depletedMarks.add(new DepletedMark(p, now, s, r, ttl));
    if (depletedMarks.size() > DEPLETED_MARKS_MAX) depletedMarks.remove(0);
  }

  /**
   * Runs add depleted ring in the Repulsor runtime.
   *
   * @param p value used by this operation.
   * @param r0 value used by this operation.
   * @param r1 value used by this operation.
   * @param strength value used by this operation.
   * @param ttlS value used by this operation.
   * @param errorMode value used by this operation.
   */
  void addDepletedRing(
      Translation2d p, double r0, double r1, double strength, double ttlS, boolean errorMode) {
    if (errorMode) return;
    if (p == null) return;
    double now = PredictiveClock.nowSeconds();
    depletedMarks.add(
        new DepletedMark(p, now, Math.max(0.0, strength), r0, r1, Math.max(0.1, ttlS)));
    if (depletedMarks.size() > DEPLETED_MARKS_MAX) depletedMarks.remove(0);
  }

  /**
   * Returns the depleted penalty soft value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param errorMode value used by this operation.
   * @return value produced by this operation.
   */
  double depletedPenaltySoft(Translation2d p, boolean errorMode) {
    if (errorMode) return 0.0;
    if (p == null || depletedMarks.isEmpty()) return 0.0;

    double now = PredictiveClock.nowSeconds();
    double sum = 0.0;

    for (int i = 0; i < depletedMarks.size(); i++) {
      DepletedMark m = depletedMarks.get(i);
      if (m == null) continue;
      double age = Math.max(0.0, now - m.t);
      if (age > m.ttl) continue;

      double w = m.s * Math.exp(-DEPLETED_DECAY * age);
      double d = m.p.getDistance(p);

      if (!m.ring) {
        double sig2 = Math.max(1e-6, m.r * m.r);
        sum += w * Math.exp(-0.5 * (d * d) / sig2);
      } else {
        double mid = 0.5 * (m.ringR0 + m.ringR1);
        double width = Math.max(0.08, 0.33 * (m.ringR1 - m.ringR0));
        double x = d - mid;
        double sig2 = Math.max(1e-6, width * width);
        sum += w * Math.exp(-0.5 * (x * x) / sig2);
      }
    }

    return Math.min(2.25, sum);
  }

  /**
   * Updates record region attempt state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param p value used by this operation.
   * @param now value used by this operation.
   * @param success value used by this operation.
   */
  public void recordRegionAttempt(Translation2d p, double now, boolean success) {
    if (p == null) return;
    long k = regionKey(p, 0.30);
    RegionStat st = regionStats.get(k);
    if (st == null) {
      if (regionStats.size() > REGION_STATS_MAX) regionStats.clear();
      st = new RegionStat();
      regionStats.put(k, st);
    }
    st.attempts++;
    if (success) st.successes++;
    st.lastAttemptTs = now;
  }

  /**
   * Returns the region bandit bonus value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param now value used by this operation.
   * @return value produced by this operation.
   */
  public double regionBanditBonus(Translation2d p, double now) {
    if (p == null) return 0.0;
    if (regionStats.isEmpty()) return 0.0;

    long k = regionKey(p, 0.30);
    RegionStat st = regionStats.get(k);
    if (st == null) return 0.0;

    int a = Math.max(1, st.attempts);
    double mean = (double) st.successes / a;

    double totalA = 0.0;
    for (RegionStat rs : regionStats.values()) totalA += Math.max(1, rs.attempts);
    totalA = Math.max(1.0, totalA);

    double ucb = mean + 0.65 * Math.sqrt(Math.log(totalA + 1.0) / a);

    double rec = now - st.lastAttemptTs;
    double recW = rec >= 0.0 ? Math.exp(-0.25 * rec) : 1.0;

    return 0.22 * ucb * recW;
  }

  /**
   * Returns the region key value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @param binM value used by this operation.
   * @return value produced by this operation.
   */
  static long regionKey(Translation2d p, double binM) {
    double inv = 1.0 / Math.max(1e-6, binM);
    int cx = (int) Math.floor(p.getX() * inv);
    int cy = (int) Math.floor(p.getY() * inv);
    return SpatialDyn.key(cx, cy);
  }

  /**
   * Returns the get default depleted ttl s value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  double getDefaultDepletedTtlS() {
    return DEPLETED_TTL_S;
  }
}
