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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.IntentAgg;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.Track;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveClock;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.SpatialDyn;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.GameSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.ResourceRegionSummary;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;

/**
 * Provides predictive field state tracking runtime functionality for the Repulsor runtime helper
 * layer shared by behaviours and planners. Use this type from robot code, field profiles, or tests
 * when integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
 * method documents robot-relative motion.
 */
public final class PredictiveFieldStateTrackingRuntime {
  private PredictiveFieldStateTrackingRuntime() {}

  /**
   * Updates update ally state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param ops value used by this operation.
   * @param id value used by this operation.
   * @param pos value used by this operation.
   * @param velHint value used by this operation.
   * @param speedCap value used by this operation.
   */
  public static void updateAlly(
      PredictiveFieldStateOps ops,
      int id,
      Translation2d pos,
      Translation2d velHint,
      Double speedCap) {
    if (pos == null) return;

    double now = PredictiveClock.nowSeconds();
    Track t =
        ops.allyMap.getOrDefault(
            id,
            new Track(
                pos,
                new Translation2d(),
                speedCap != null ? speedCap : PredictiveFieldStateOps.DEFAULT_ALLY_SPEED,
                now));

    double dtRaw = now - t.lastTs;
    double dt = Math.max(PredictiveFieldStateOps.MIN_DT, dtRaw);
    double dtMeas =
        Math.min(
            PredictiveFieldStateOps.MAX_MEAS_DT, Math.max(PredictiveFieldStateOps.MIN_DT, dtRaw));

    Translation2d vMeas = velHint != null ? velHint : pos.minus(t.pos).div(dtMeas);

    double cap =
        speedCap != null ? Math.max(0.1, speedCap) : PredictiveFieldStateOps.DEFAULT_ALLY_SPEED;
    double vMag = Math.min(vMeas.getNorm(), cap);
    Translation2d vClamped =
        vMeas.getNorm() > 1e-6 ? vMeas.div(vMeas.getNorm()).times(vMag) : new Translation2d();

    double aVel = PredictiveFieldStateOps.emaAlpha(PredictiveFieldStateOps.VEL_EMA, dt);
    double aPos = PredictiveFieldStateOps.emaAlpha(PredictiveFieldStateOps.POS_EMA, dt);

    Translation2d vEma = PredictiveFieldStateOps.lerpVec(t.vel, vClamped, aVel);
    t.vel = PredictiveFieldStateOps.clampDeltaV(t.vel, vEma, PredictiveFieldStateOps.ACC_LIMIT, dt);

    t.pos = PredictiveFieldStateOps.lerpVec(t.pos, pos, aPos);
    t.speedCap = cap;
    t.lastTs = now;

    ops.allyMap.put(id, t);
  }

  /**
   * Updates update enemy state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param ops value used by this operation.
   * @param id value used by this operation.
   * @param pos value used by this operation.
   * @param velHint value used by this operation.
   * @param speedCap value used by this operation.
   */
  public static void updateEnemy(
      PredictiveFieldStateOps ops,
      int id,
      Translation2d pos,
      Translation2d velHint,
      Double speedCap) {
    if (pos == null) return;

    double now = PredictiveClock.nowSeconds();
    Track t =
        ops.enemyMap.getOrDefault(
            id,
            new Track(
                pos,
                new Translation2d(),
                speedCap != null ? speedCap : PredictiveFieldStateOps.DEFAULT_ENEMY_SPEED,
                now));

    double dtRaw = now - t.lastTs;
    double dt = Math.max(PredictiveFieldStateOps.MIN_DT, dtRaw);
    double dtMeas =
        Math.min(
            PredictiveFieldStateOps.MAX_MEAS_DT, Math.max(PredictiveFieldStateOps.MIN_DT, dtRaw));

    Translation2d vMeas = velHint != null ? velHint : pos.minus(t.pos).div(dtMeas);

    double cap =
        speedCap != null ? Math.max(0.1, speedCap) : PredictiveFieldStateOps.DEFAULT_ENEMY_SPEED;
    double vMag = Math.min(vMeas.getNorm(), cap);
    Translation2d vClamped =
        vMeas.getNorm() > 1e-6 ? vMeas.div(vMeas.getNorm()).times(vMag) : new Translation2d();

    double aVel = PredictiveFieldStateOps.emaAlpha(PredictiveFieldStateOps.VEL_EMA, dt);
    double aPos = PredictiveFieldStateOps.emaAlpha(PredictiveFieldStateOps.POS_EMA, dt);

    Translation2d vEma = PredictiveFieldStateOps.lerpVec(t.vel, vClamped, aVel);
    t.vel = PredictiveFieldStateOps.clampDeltaV(t.vel, vEma, PredictiveFieldStateOps.ACC_LIMIT, dt);

    t.pos = PredictiveFieldStateOps.lerpVec(t.pos, pos, aPos);
    t.speedCap = cap;
    t.lastTs = now;

    ops.enemyMap.put(id, t);
  }

  /**
   * Updates clear stale state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param ops value used by this operation.
   * @param maxAgeS value used by this operation.
   */
  public static void clearStale(PredictiveFieldStateOps ops, double maxAgeS) {
    double now = PredictiveClock.nowSeconds();
    ops.allyMap.entrySet().removeIf(e -> now - e.getValue().lastTs > maxAgeS);
    ops.enemyMap.entrySet().removeIf(e -> now - e.getValue().lastTs > maxAgeS);
  }

  /**
   * Returns the rank value maintained by this Repulsor component.
   *
   * @param ops value used by this operation.
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param cat value used by this operation.
   * @param limit value used by this operation.
   * @return value produced by this operation.
   */
  public static List<Candidate> rank(
      PredictiveFieldStateOps ops,
      Translation2d ourPos,
      double ourSpeedCap,
      CategorySpec cat,
      int limit) {
    Objects.requireNonNull(ourPos);
    List<RankTarget> targets = new ArrayList<>();
    for (GameElement e : ops.worldElements) {
      RankTarget target = rankTargetFor(e, ops.ourAlliance, cat);
      if (target != null) {
        targets.add(target);
      }
    }
    if (targets.isEmpty()) return List.of();

    List<Translation2d> targetPoints = new ArrayList<>();
    for (RankTarget target : targets) {
      targetPoints.add(target.point());
    }

    IntentAgg allyAgg = ops.softIntentAgg(ops.allyMap, targetPoints);
    IntentAgg enemyAgg = ops.softIntentAgg(ops.enemyMap, targetPoints);

    double now = PredictiveClock.nowSeconds();
    List<Candidate> out = new ArrayList<>();
    double cap = ourSpeedCap > 0 ? ourSpeedCap : PredictiveFieldStateOps.DEFAULT_OUR_SPEED;

    for (int i = 0; i < targets.size(); i++) {
      RankTarget target = targets.get(i);
      Translation2d t = target.point();
      RepulsorSetpoint sp = target.setpoint();

      double ourEta = ops.estimateTravelTime(ourPos, t, cap);
      double enemyEta = PredictiveFieldStateOps.minEtaToTarget(ops.enemyMap, t);
      double allyEta = PredictiveFieldStateOps.minEtaToTarget(ops.allyMap, t);

      double pressure =
          ops.radialPressure(ops.enemyMap, t, ourEta, enemyAgg.intent[i], enemyAgg.count);
      double congestion =
          ops.radialCongestion(ops.allyMap, t, ourEta, allyAgg.intent[i], allyAgg.count);
      double distBias = ourPos.getDistance(t) * PredictiveFieldStateOps.DIST_COST;

      double capacityFrac = 0.0;
      GameElement e = target.element();
      if (e.getMaxContained() > 0) {
        capacityFrac =
            1.0
                - Math.min(
                    1.0, (double) e.getContainedCount() / Math.max(1.0, e.getMaxContained()));
      }

      double heading = ops.headingAffinity(ourPos, t, ops.allyMap, ops.enemyMap);

      double score =
          (enemyEta - ourEta) * PredictiveFieldStateOps.ADV_GAIN
              - congestion * PredictiveFieldStateOps.CONGEST_COST
              - pressure * PredictiveFieldStateOps.PRESSURE_GAIN
              - distBias
              + capacityFrac * PredictiveFieldStateOps.CAPACITY_GAIN
              + heading * PredictiveFieldStateOps.HEADING_GAIN;

      if (ops.lastChosen != null
          && sp.equals(ops.lastChosen)
          && now - ops.lastChosenTs < PredictiveFieldStateOps.HYST_PERSIST_S) {
        score += PredictiveFieldStateOps.HYST_BONUS;
      }

      out.add(new Candidate(sp, t, ourEta, enemyEta, allyEta, congestion, pressure, score));
    }

    out.sort(Comparator.comparingDouble((Candidate c) -> -c.score));
    if (!out.isEmpty()) {
      ops.lastChosen = out.get(0).setpoint;
      ops.lastChosenTs = now;
    }
    if (limit > 0 && out.size() > limit) return new ArrayList<>(out.subList(0, limit));
    return out;
  }

  private static RankTarget rankTargetFor(
      GameElement e,
      org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance alliance,
      CategorySpec cat) {
    if (e == null || e.isAtCapacity()) return null;
    if (e.getAlliance() != alliance) return null;
    if (cat != null && e.getCategory() != cat) return null;
    if (e.getModel() == null || e.getModel().getPosition() == null) return null;

    var position = e.getModel().getPosition();
    var rotation = position.getRotation();
    double headingRadians = rotation == null ? 0.0 : rotation.getZ();
    Pose2d pose =
        new Pose2d(position.getX(), position.getY(), Rotation2d.fromRadians(headingRadians));
    Translation2d point = pose.getTranslation();
    RepulsorSetpoint setpoint = e.getRelatedPoint().orElseGet(() -> fallbackSetpoint(e, pose));
    return new RankTarget(e, point, setpoint);
  }

  private static RepulsorSetpoint fallbackSetpoint(GameElement e, Pose2d pose) {
    String category = categoryLevelId(e.getCategory());
    String name = String.format("field-%s-%.2f-%.2f", category, pose.getX(), pose.getY());
    GameSetpoint point =
        new GameSetpoint(name, SetpointType.kOther, false) {
          @Override
          public Pose2d bluePose(SetpointContext ctx) {
            return pose;
          }
        };
    return new RepulsorSetpoint(point, category, HeightSetpoint.NONE);
  }

  private static String categoryLevelId(CategorySpec category) {
    if (category == null) return "objective";
    String raw = category.name().trim().toLowerCase();
    return raw.startsWith("k") && raw.length() > 1 ? raw.substring(1) : raw;
  }

  private record RankTarget(GameElement element, Translation2d point, RepulsorSetpoint setpoint) {}

  /**
   * Runs sort idx by key in the Repulsor runtime.
   *
   * @param key distance or field-coordinate value in meters.
   * @param idx distance or field-coordinate value in meters.
   */
  public static void sortIdxByKey(double[] key, int[] idx) {
    quickSortIdx(key, idx, 0, idx.length - 1);
  }

  /**
   * Runs quick sort idx in the Repulsor runtime.
   *
   * @param key distance or field-coordinate value in meters.
   * @param idx distance or field-coordinate value in meters.
   * @param lo value used by this operation.
   * @param hi value used by this operation.
   */
  public static void quickSortIdx(double[] key, int[] idx, int lo, int hi) {
    while (lo < hi) {
      int i = lo;
      int j = hi;
      double pivot = key[idx[(lo + hi) >>> 1]];

      while (i <= j) {
        while (key[idx[i]] < pivot) i++;
        while (key[idx[j]] > pivot) j--;
        if (i <= j) {
          int tmp = idx[i];
          idx[i] = idx[j];
          idx[j] = tmp;
          i++;
          j--;
        }
      }

      if (j - lo < hi - i) {
        if (lo < j) quickSortIdx(key, idx, lo, j);
        lo = i;
      } else {
        if (i < hi) quickSortIdx(key, idx, i, hi);
        hi = j;
      }
    }
  }

  /**
   * Returns the rank setpoints value maintained by this Repulsor component.
   *
   * @param ops value used by this operation.
   * @param ourPos value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param cat value used by this operation.
   * @param limit value used by this operation.
   * @return list of repulsor setpoint values produced by this operation.
   */
  public static List<RepulsorSetpoint> rankSetpoints(
      PredictiveFieldStateOps ops,
      Translation2d ourPos,
      double ourSpeedCap,
      CategorySpec cat,
      int limit) {
    List<Candidate> c = rank(ops, ourPos, ourSpeedCap, cat, limit);
    List<RepulsorSetpoint> out = new ArrayList<>();
    for (Candidate k : c) out.add(k.setpoint);
    return out;
  }

  /**
   * Returns the resource observation count value maintained by this Repulsor component.
   *
   * @param ops value used by this operation.
   * @return value produced by this operation.
   */
  public static int resourceObservationCount(PredictiveFieldStateOps ops) {
    SpatialDyn d = ops.cachedDyn();
    if (d == null) return 0;
    return d.resources.size();
  }

  /**
   * Returns the snap to collect centroid value maintained by this Repulsor component.
   *
   * @param ops value used by this operation.
   * @param seed value used by this operation.
   * @param r value used by this operation.
   * @param minMass value used by this operation.
   * @return value produced by this operation.
   */
  public static Translation2d snapToCollectCentroid(
      PredictiveFieldStateOps ops, Translation2d seed, double r, double minMass) {
    if (seed == null) return null;
    SpatialDyn dyn = ops.cachedDyn();
    if (dyn == null) return seed;

    Translation2d c = dyn.centroidResourcesWithin(seed, Math.max(0.05, r), Math.max(0.0, minMass));
    if (c == null) return seed;

    Translation2d ourPos = ops.lastOurPosForCollect != null ? ops.lastOurPosForCollect : seed;
    double cap =
        ops.lastOurCapForCollect > 0.0
            ? ops.lastOurCapForCollect
            : PredictiveFieldStateOps.DEFAULT_OUR_SPEED;
    int goal = Math.max(1, ops.lastGoalUnitsForCollect);
    double cellM = Math.max(0.10, ops.lastCellMForCollect);

    var a = ops.evalCollectPoint(ourPos, cap, seed, goal, cellM, dyn, null, null);
    var b = ops.evalCollectPoint(ourPos, cap, c, goal, cellM, dyn, null, null);

    if (b.score > a.score + 1e-9) return c;
    return seed;
  }

  /**
   * Computes the nearest collect resource value for the current Repulsor planning state.
   *
   * @param ops value used by this operation.
   * @param p value used by this operation.
   * @param maxDist value used by this operation.
   * @return value produced by this operation.
   */
  public static Translation2d nearestCollectResource(
      PredictiveFieldStateOps ops, Translation2d p, double maxDist) {
    if (p == null) return null;
    SpatialDyn d = ops.cachedDyn();
    if (d == null) return null;
    return d.nearestResourceTo(p, Math.max(0.01, maxDist));
  }

  /**
   * Summarizes collectable resource evidence inside a field region for strategy evaluation.
   *
   * @param ops predictive state operations containing resource specs and traffic tracks
   * @param id stable region identifier
   * @param regionFilter field-relative predicate selecting resources in the region
   * @param robotPos current robot position in field-relative meters
   * @param maxDistanceMeters maximum actionable robot-to-resource distance
   * @return normalized resource and risk summary for the requested region
   */
  public static ResourceRegionSummary summarizeResourceRegion(
      PredictiveFieldStateOps ops,
      String id,
      Predicate<Translation2d> regionFilter,
      Translation2d robotPos,
      double maxDistanceMeters) {
    if (ops == null || robotPos == null) return ResourceRegionSummary.empty(id);
    SpatialDyn dyn = ops.cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return ResourceRegionSummary.empty(id);

    Predicate<Translation2d> accepts = regionFilter != null ? regionFilter : point -> true;
    double maxDist =
        Double.isFinite(maxDistanceMeters) && maxDistanceMeters > 0.0
            ? maxDistanceMeters
            : ops.getFieldGeometry().diagonalMeters();
    double maxDist2 = maxDist * maxDist;

    double units = 0.0;
    double value = 0.0;
    Translation2d nearest = null;
    double nearestDist2 = Double.POSITIVE_INFINITY;

    for (DynamicObject object : dyn.resources) {
      if (object == null || object.pos == null || object.type == null) continue;
      if (!accepts.test(object.pos)) continue;

      double dx = object.pos.getX() - robotPos.getX();
      double dy = object.pos.getY() - robotPos.getY();
      double d2 = dx * dx + dy * dy;
      if (d2 > maxDist2) continue;

      double objectValue = dyn.resourceEvidence(object);
      if (objectValue <= 1e-9) continue;
      units += objectValue;
      value += objectValue;

      if (d2 < nearestDist2) {
        nearestDist2 = d2;
        nearest = object.pos;
      }
    }

    if (nearest == null || units <= 1e-9 || value <= 1e-9) {
      return ResourceRegionSummary.empty(id);
    }

    double nearestDistance = Math.sqrt(Math.max(0.0, nearestDist2));
    double travelEta =
        ops.estimateTravelTime(
            robotPos,
            nearest,
            ops.lastOurCapForCollect > 0.0
                ? ops.lastOurCapForCollect
                : PredictiveFieldStateOps.DEFAULT_OUR_SPEED);
    double enemyPressure = ops.radialPressure(ops.enemyMap, nearest, travelEta, 0.0, 0);
    double allyCongestion = ops.radialCongestion(ops.allyMap, nearest, travelEta, 0.0, 0);
    double nearbyTraffic =
        PredictiveFieldStateOps.radialDensity(
                ops.enemyMap, nearest, PredictiveFieldStateOps.COLLECT_ACTIVITY_SIGMA)
            + 0.65
                * PredictiveFieldStateOps.radialDensity(
                    ops.allyMap, nearest, PredictiveFieldStateOps.COLLECT_ACTIVITY_SIGMA);
    double trafficRisk = Math.min(3.0, enemyPressure + allyCongestion + nearbyTraffic);
    double obstacleRisk =
        Math.min(
            3.0,
            dyn.localAvoidPenalty(nearest, PredictiveFieldStateOps.COLLECT_LOCAL_AVOID_R)
                + ops.depletedPenaltySoft(nearest));

    return new ResourceRegionSummary(
        id, units, value, nearest, nearestDistance, trafficRisk, obstacleRisk);
  }
}
