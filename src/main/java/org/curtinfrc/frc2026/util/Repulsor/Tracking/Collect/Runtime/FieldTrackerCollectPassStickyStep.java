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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime;

import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.dot;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.holdSForDist;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.nowSFromNs;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.stickySame;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.stickySwitched;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.switchMarginForDist;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.unit;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.unitOrDefault;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.ToDoubleBiFunction;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.CollectObjectiveSelectionConfig;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveLoop;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath;

/**
 * Provides field tracker collect pass sticky step functionality for the Repulsor runtime helper
 * layer shared by behaviours and planners. Use this type from robot code, field profiles, or tests
 * when integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
 * method documents robot-relative motion.
 */
public final class FieldTrackerCollectPassStickyStep {
  private FieldTrackerCollectPassStickyStep() {}

  /**
   * Returns the should block far switch value maintained by this Repulsor component.
   *
   * @param distToCurrent value used by this operation.
   * @param currentScore value used by this operation.
   * @param bestScore value used by this operation.
   * @param margin value used by this operation.
   * @param currentIsTrap value used by this operation.
   * @param bestIsTrap value used by this operation.
   * @param currentValid value used by this operation.
   * @param bestValid value used by this operation.
   * @param stillSec time value in seconds.
   * @param movedSinceLastSwitchM value used by this operation.
   * @param sinceLastSwitchS value used by this operation.
   * @return value produced by this operation.
   */
  static boolean shouldBlockFarSwitch(
      double distToCurrent,
      double currentScore,
      double bestScore,
      double margin,
      boolean currentIsTrap,
      boolean bestIsTrap,
      boolean currentValid,
      boolean bestValid,
      double stillSec,
      double movedSinceLastSwitchM,
      double sinceLastSwitchS) {
    return shouldBlockFarSwitch(
        distToCurrent,
        currentScore,
        bestScore,
        margin,
        currentIsTrap,
        bestIsTrap,
        currentValid,
        bestValid,
        stillSec,
        movedSinceLastSwitchM,
        sinceLastSwitchS,
        CollectObjectiveSelectionConfig.defaults());
  }

  static boolean shouldBlockFarSwitch(
      double distToCurrent,
      double currentScore,
      double bestScore,
      double margin,
      boolean currentIsTrap,
      boolean bestIsTrap,
      boolean currentValid,
      boolean bestValid,
      double stillSec,
      double movedSinceLastSwitchM,
      double sinceLastSwitchS,
      CollectObjectiveSelectionConfig selection) {
    CollectObjectiveSelectionConfig config = normalized(selection);
    if (!currentValid || !bestValid) return false;
    if (!Double.isFinite(distToCurrent) || distToCurrent < config.farSwitchLockDistanceMeters()) {
      return false;
    }
    if (currentIsTrap && !bestIsTrap) return false;
    if (stillSec >= 0.30) return false;
    if (sinceLastSwitchS >= 0.55
        && Double.isFinite(movedSinceLastSwitchM)
        && movedSinceLastSwitchM < FieldTrackerCollectObjectiveLoop.COLLECT_SWITCH_MIN_MOVE_M) {
      return false;
    }
    return bestScore <= currentScore + (margin * config.farSwitchForceMultiplier());
  }

  /**
   * Returns the adapt switch margin for distance value maintained by this Repulsor component.
   *
   * @param margin value used by this operation.
   * @param distToCurrent value used by this operation.
   * @return value produced by this operation.
   */
  static double adaptSwitchMarginForDistance(double margin, double distToCurrent) {
    return adaptSwitchMarginForDistance(
        margin, distToCurrent, CollectObjectiveSelectionConfig.defaults());
  }

  static double adaptSwitchMarginForDistance(
      double margin, double distToCurrent, CollectObjectiveSelectionConfig selection) {
    CollectObjectiveSelectionConfig config = normalized(selection);
    if (!Double.isFinite(distToCurrent)) return margin;
    if (distToCurrent <= config.closeSwitchEasyDistanceMeters()) {
      return margin * config.closeSwitchMarginScale();
    }
    return margin;
  }

  /**
   * Returns the should hold previous for too soon value maintained by this Repulsor component.
   *
   * @param tooSoon value used by this operation.
   * @param stillSec time value in seconds.
   * @param opposite value used by this operation.
   * @param distToCurrent value used by this operation.
   * @return value produced by this operation.
   */
  static boolean shouldHoldPreviousForTooSoon(
      boolean tooSoon, double stillSec, boolean opposite, double distToCurrent) {
    return shouldHoldPreviousForTooSoon(
        tooSoon, stillSec, opposite, distToCurrent, CollectObjectiveSelectionConfig.defaults());
  }

  static boolean shouldHoldPreviousForTooSoon(
      boolean tooSoon,
      double stillSec,
      boolean opposite,
      double distToCurrent,
      CollectObjectiveSelectionConfig selection) {
    CollectObjectiveSelectionConfig config = normalized(selection);
    if (!tooSoon) return false;
    if (opposite) return false;
    if (Double.isFinite(distToCurrent) && distToCurrent <= config.closeSwitchEasyDistanceMeters()) {
      return false;
    }
    return stillSec < 0.10;
  }

  /**
   * Returns the prefer ranked candidate for sticky value maintained by this Repulsor component.
   *
   * @param bestCandidate value used by this operation.
   * @param cand value used by this operation.
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  static Translation2d preferRankedCandidateForSticky(
      Translation2d bestCandidate,
      FieldTrackerCollectPassCandidateResult cand,
      FieldTrackerCollectPassContext ctx) {
    return preferRankedCandidateForSticky(
        bestCandidate, cand, ctx, CollectObjectiveSelectionConfig.defaults());
  }

  static Translation2d preferRankedCandidateForSticky(
      Translation2d bestCandidate,
      FieldTrackerCollectPassCandidateResult cand,
      FieldTrackerCollectPassContext ctx,
      CollectObjectiveSelectionConfig selection) {
    if (cand == null) return bestCandidate;
    CollectObjectiveSelectionConfig config = normalized(selection);
    if (cand.best() == null || cand.best().point == null) return bestCandidate;
    Translation2d ranked = cand.best().point;
    if (!cand.collectValid().test(ranked)) return bestCandidate;
    if (bestCandidate == null) return ranked;
    if (!cand.collectValid().test(bestCandidate)) return ranked;

    boolean rankedTrap =
        FieldTrackerCollectObjectiveMath.isHubFrontTrapPoint(
            ranked, ctx.leftBandX1(), ctx.rightBandX0());
    boolean bestTrap =
        FieldTrackerCollectObjectiveMath.isHubFrontTrapPoint(
            bestCandidate, ctx.leftBandX1(), ctx.rightBandX0());
    if (rankedTrap && !bestTrap) return bestCandidate;
    if (!rankedTrap && bestTrap) return ranked;

    double rankedScore = cand.scoreResource().apply(ranked);
    double currentScore = cand.scoreResource().apply(bestCandidate);
    if (rankedScore > currentScore + config.stickyPreferRankedScoreMargin()) return ranked;
    return bestCandidate;
  }

  private static CollectObjectiveSelectionConfig normalized(
      CollectObjectiveSelectionConfig config) {
    return config == null ? CollectObjectiveSelectionConfig.defaults() : config;
  }

  /**
   * Computes the select and prime value for the current Repulsor planning state. Call this from
   * periodic planning or tests when a fresh decision is required; inputs should already be
   * expressed in the coordinate frame expected by the parameter names.
   *
   * @param loop value used by this operation.
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @param cand value used by this operation.
   * @param pass value used by this operation.
   * @return field tracker collect pass sticky result result for select and prime.
   */
  public static FieldTrackerCollectPassStickyResult selectAndPrime(
      FieldTrackerCollectObjectiveLoop loop,
      FieldTrackerCollectPassContext ctx,
      FieldTrackerCollectPassCandidateResult cand,
      int pass) {
    CollectObjectiveSelectionConfig selection = loop.collectPlannerTuning().selection();
    Translation2d bestCandidate =
        preferRankedCandidateForSticky(cand.bestCandidate(), cand, ctx, selection);
    Translation2d prevSticky = loop.collectStickyPoint;

    double distToCand = ctx.robotPos().getDistance(bestCandidate);
    double distToCur =
        loop.collectStickyPoint != null
            ? ctx.robotPos().getDistance(loop.collectStickyPoint)
            : Double.NaN;
    double holdS = holdSForDist(distToCand);
    double keepMargin = switchMarginForDist(distToCand);
    double immediateDelta = Math.max(keepMargin * 1.85, keepMargin + 0.28);

    if (Double.isFinite(distToCur) && distToCur - distToCand > 0.35) {
      holdS = Math.min(holdS, 0.35);
      keepMargin *= 0.70;
      immediateDelta *= 0.70;
    }

    if (ctx.robotInCenterBand()) {
      holdS = Math.max(holdS, 0.85);
      keepMargin *= 1.05;
      immediateDelta *= 1.03;
    }

    final double keepMarginF = keepMargin;

    ToDoubleBiFunction<Translation2d, Translation2d> transitionExtra =
        (from, to) -> {
          if (from == null || to == null) return 0.0;
          double extra = 0.0;
          if (from.getDistance(to) <= FieldTrackerCollectObjectiveLoop.COLLECT_SWITCH_CLOSE_M)
            extra += keepMarginF * 1.35 + 0.14;

          return extra;
        };

    Translation2d selectedResource;
    boolean forcedFuelReplacement = false;

    if (pass == 0) {
      Translation2d cur = loop.collectStickyPoint;
      boolean curValid = cur != null && cand.collectValid().test(cur);
      boolean bestValid = bestCandidate != null && cand.collectValid().test(bestCandidate);

      selectedResource = curValid ? cur : (bestValid ? bestCandidate : prevSticky);

      if (bestValid && curValid && !stickySame(cur, bestCandidate)) {
        boolean curTrap =
            FieldTrackerCollectObjectiveMath.isHubFrontTrapPoint(
                cur, ctx.leftBandX1(), ctx.rightBandX0());
        boolean bestTrap =
            FieldTrackerCollectObjectiveMath.isHubFrontTrapPoint(
                bestCandidate, ctx.leftBandX1(), ctx.rightBandX0());

        if (curTrap && !bestTrap) {
          selectedResource = bestCandidate;
          forcedFuelReplacement = true;
        } else {
          double curScore = cand.scoreResource().apply(cur);
          double bestScore = cand.scoreResource().apply(bestCandidate);
          double margin = switchMarginForDist(ctx.robotPos().getDistance(bestCandidate));
          margin = adaptSwitchMarginForDistance(margin, distToCur, selection);
          double sinceLastSwitchS = nowSFromNs(ctx.nowNs() - loop.collectStickyLastSwitchNs);
          double movedSinceLastSwitchM =
              loop.collectStickyLastSwitchRobotPos != null
                  ? ctx.robotPos().getDistance(loop.collectStickyLastSwitchRobotPos)
                  : Double.POSITIVE_INFINITY;
          if (sinceLastSwitchS < 0.35) margin *= 1.45;
          if (ctx.robotInCenterBand()) margin *= 1.10;
          if (!shouldBlockFarSwitch(
              distToCur,
              curScore,
              bestScore,
              margin,
              curTrap,
              bestTrap,
              curValid,
              bestValid,
              loop.collectStickyStillSec,
              movedSinceLastSwitchM,
              sinceLastSwitchS,
              selection)) {
            if (bestScore > curScore + margin) selectedResource = bestCandidate;
          }
        }
      } else if (!curValid && bestValid) {
        selectedResource = bestCandidate;
        forcedFuelReplacement = true;
      }
    } else {
      Translation2d cur = loop.collectStickyPoint;
      selectedResource = (cur != null && cand.collectValid().test(cur)) ? cur : bestCandidate;
    }

    if (pass == 0
        && !forcedFuelReplacement
        && prevSticky != null
        && selectedResource != null
        && !stickySame(prevSticky, selectedResource)) {
      Translation2d toPrev = prevSticky.minus(ctx.robotPos());
      Translation2d toNext = selectedResource.minus(ctx.robotPos());

      Translation2d prevHat = unitOrDefault(toPrev, new Translation2d(1.0, 0.0));
      Translation2d nextHat = unitOrDefault(toNext, prevHat);

      boolean opposite = dot(prevHat, nextHat) < 0.15;
      boolean tooSoon = nowSFromNs(ctx.nowNs() - loop.collectStickyLastSwitchNs) < 0.25;

      if (shouldHoldPreviousForTooSoon(
          tooSoon, loop.collectStickyStillSec, opposite, distToCur, selection)) {
        selectedResource = prevSticky;
        loop.collectStickySelector.force(prevSticky);
      }
    }

    boolean switched = stickySwitched(prevSticky, selectedResource);

    if (switched && !forcedFuelReplacement && loop.collectStickyLastSwitchNs != 0L) {
      double sinceLastSwitchS = nowSFromNs(ctx.nowNs() - loop.collectStickyLastSwitchNs);
      double movedSinceLastSwitch =
          loop.collectStickyLastSwitchRobotPos != null
              ? ctx.robotPos().getDistance(loop.collectStickyLastSwitchRobotPos)
              : Double.POSITIVE_INFINITY;
      if (sinceLastSwitchS < FieldTrackerCollectObjectiveLoop.COLLECT_SWITCH_COOLDOWN_S
          && movedSinceLastSwitch < FieldTrackerCollectObjectiveLoop.COLLECT_SWITCH_MIN_MOVE_M) {
        if (prevSticky != null) {
          selectedResource = prevSticky;
          loop.collectStickySelector.force(prevSticky);
          switched = false;
        }
      }
    }

    loop.collectStickyPoint = selectedResource;
    loop.collectStickyScore = cand.scoreResource().apply(selectedResource);
    loop.collectStickyTsNs = ctx.nowNs();

    if (switched) {
      loop.collectStickyReachedTsNs = 0L;
      loop.collectStickyLastSwitchNs = ctx.nowNs();
      loop.collectStickyLastSwitchRobotPos = ctx.robotPos();
      loop.collectStickyNoProgressSinceNs = ctx.nowNs();
      loop.collectStickyLastDistM = Double.POSITIVE_INFINITY;
      loop.collectSnapActive = false;

      Translation2d dir = selectedResource.minus(ctx.robotPos());
      Translation2d hat = unitOrDefault(dir, new Translation2d(1.0, 0.0));

      double dist = ctx.robotPos().getDistance(selectedResource);
      double pushT = FieldTrackerCollectObjectiveMath.clamp01((dist - 0.35) / (2.50 - 0.35));
      double pushM =
          Math.min(
              FieldTrackerCollectObjectiveMath.lerp(0.42, 0.18, pushT),
              FieldTrackerCollectObjectiveLoop.COLLECT_MAX_DRIVE_OFFSET_FROM_FUEL_M);

      loop.collectStickyApproachHat = hat;
      loop.collectStickyPushM = pushM;
      loop.collectStickySide = 0;

      Translation2d frozen =
          loop.computeFrozenDriveTarget(
              selectedResource,
              loop.collectStickyApproachHat,
              loop.collectStickyPushM,
              ctx.clampToFieldRobotSafe(),
              ctx.inForbidden(),
              ctx.violatesWall(),
              ctx.nudgeOutOfForbidden());
      loop.collectStickyDriveTarget = frozen;
      loop.collectForcedDriveCand = null;
      loop.collectForcedDriveSinceNs = 0L;
    }

    Translation2d desiredCollectPoint = loop.collectStickyPoint;

    if (desiredCollectPoint == null) {
      loop.clearCollectSticky();
      return new FieldTrackerCollectPassStickyResult(
          null, null, loop.fallbackCollectPose(ctx.robotPoseBlue()));
    }
    Translation2d desiredDriveTarget = loop.collectStickyDriveTarget;

    if (!cand.collectValid().test(desiredCollectPoint)) {
      if (!cand.footprintHasFuel().test(desiredCollectPoint)) {
        loop.predictor.markCollectDepleted(
            desiredCollectPoint, FieldTrackerCollectObjectiveLoop.COLLECT_CELL_M, 1.0);
        loop.clearCollectSticky();
        loop.collectStickySelector.forceInvalidate();
        Translation2d fallbackTarget =
            (bestCandidate != null && cand.collectValid().test(bestCandidate))
                ? bestCandidate
                : null;
        if (fallbackTarget != null) {
          loop.collectStickyPoint = fallbackTarget;
          loop.collectStickyScore = cand.scoreResource().apply(fallbackTarget);
          loop.collectStickyTsNs = ctx.nowNs();
          loop.collectStickyInvalidSec = 0.0;
          loop.collectStickySelector.force(fallbackTarget);
          desiredCollectPoint = fallbackTarget;
        } else {
          return new FieldTrackerCollectPassStickyResult(
              null,
              null,
              ctx.holdPose()
                  .apply(
                      desiredDriveTarget != null
                          ? desiredDriveTarget
                          : ctx.robotPoseBlue().getTranslation()));
        }
      }
      loop.collectStickyInvalidSec += ctx.dt();

      if (loop.collectStickyInvalidSec
              >= FieldTrackerCollectObjectiveLoop.COLLECT_STICKY_INVALID_DROP_SEC
          && pass == 0) {
        loop.predictor.markCollectDepleted(
            desiredCollectPoint, FieldTrackerCollectObjectiveLoop.COLLECT_CELL_M, 1.0);
        loop.clearCollectSticky();
      }

      return new FieldTrackerCollectPassStickyResult(
          null,
          null,
          ctx.holdPose()
              .apply(
                  desiredDriveTarget != null
                      ? desiredDriveTarget
                      : ctx.robotPoseBlue().getTranslation()));
    } else {
      loop.collectStickyInvalidSec = 0.0;
    }

    if (desiredDriveTarget == null) {
      desiredDriveTarget =
          loop.computeFrozenDriveTarget(
              desiredCollectPoint,
              loop.collectStickyApproachHat != null
                  ? loop.collectStickyApproachHat
                  : unit(desiredCollectPoint.minus(ctx.robotPos())),
              loop.collectStickyPushM > 1e-6 ? loop.collectStickyPushM : 0.18,
              ctx.clampToFieldRobotSafe(),
              ctx.inForbidden(),
              ctx.violatesWall(),
              ctx.nudgeOutOfForbidden());
      loop.collectStickyDriveTarget = desiredDriveTarget;
    }

    desiredDriveTarget = ctx.clampToFieldRobotSafe().apply(desiredDriveTarget);
    if (ctx.inForbidden().test(desiredDriveTarget))
      desiredDriveTarget = ctx.nudgeOutOfForbidden().apply(desiredDriveTarget);
    desiredDriveTarget = ctx.clampToFieldRobotSafe().apply(desiredDriveTarget);

    if (ctx.inForbidden().test(desiredDriveTarget) || ctx.violatesWall().test(desiredDriveTarget)) {
      desiredDriveTarget =
          loop.computeFrozenDriveTarget(
              desiredCollectPoint,
              loop.collectStickyApproachHat != null
                  ? loop.collectStickyApproachHat
                  : unit(desiredCollectPoint.minus(ctx.robotPos())),
              loop.collectStickyPushM > 1e-6 ? loop.collectStickyPushM : 0.18,
              ctx.clampToFieldRobotSafe(),
              ctx.inForbidden(),
              ctx.violatesWall(),
              ctx.nudgeOutOfForbidden());
      loop.collectStickyDriveTarget = desiredDriveTarget;
    }

    return new FieldTrackerCollectPassStickyResult(desiredCollectPoint, desiredDriveTarget, null);
  }
}
