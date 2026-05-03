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
import java.util.List;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathing;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

/**
 * Provides reactive bypass scorer functionality for the Repulsor runtime helper layer shared by
 * behaviours and planners. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
final class ReactiveBypassScorer {
  private ReactiveBypassScorer() {}

  /**
   * Computes the score candidate value for the current Repulsor planning state.
   *
   * @param cfg value used by this operation.
   * @param preferredSide value used by this operation.
   * @param timeSinceSideSwitchS value used by this operation.
   * @param stuckNow value used by this operation.
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @param waypoint value used by this operation.
   * @param goal value used by this operation.
   * @param headingTowardGoal value used by this operation.
   * @param robotX distance or field-coordinate value in meters.
   * @param robotY distance or field-coordinate value in meters.
   * @param dynamicObstacles obstacle set used for safety checks, costs, or replanning.
   * @param intersectsDynamicOnly distance or field-coordinate value in meters.
   * @return reactive bypass score result for score candidate.
   */
  static ReactiveBypassScore scoreCandidate(
      ReactiveBypassConfig cfg,
      int preferredSide,
      double timeSinceSideSwitchS,
      boolean stuckNow,
      Pose2d pose,
      Pose2d waypoint,
      Pose2d goal,
      Rotation2d headingTowardGoal,
      double robotX,
      double robotY,
      List<? extends Obstacle> dynamicObstacles,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {

    boolean ok1 =
        ExtraPathing.isClearPath(
            "Repulsor/Bypass/Leg1",
            pose.getTranslation(),
            waypoint.getTranslation(),
            dynamicObstacles,
            robotX,
            robotY,
            true);

    boolean ok2 =
        ok1
            && ExtraPathing.isClearPath(
                "Repulsor/Bypass/Leg2",
                waypoint.getTranslation(),
                goal.getTranslation(),
                dynamicObstacles,
                robotX,
                robotY,
                true);

    double len =
        pose.getTranslation().getDistance(waypoint.getTranslation())
            + waypoint.getTranslation().getDistance(goal.getTranslation());

    double angErr =
        Math.abs(
            ReactiveBypassMath.radDiff(
                headingTowardGoal.getRadians(),
                waypoint.getTranslation().minus(pose.getTranslation()).getAngle().getRadians()));
    double ang =
        (angErr <= Math.toRadians(cfg.headingDeadbandDeg))
            ? 0.0
            : cfg.angleCostWeight * (angErr - Math.toRadians(cfg.headingDeadbandDeg));

    double leg1 = waypoint.getTranslation().minus(pose.getTranslation()).getAngle().getRadians();
    double leg2 = goal.getTranslation().minus(waypoint.getTranslation()).getAngle().getRadians();
    double curv = cfg.curvatureWeight * Math.abs(ReactiveBypassMath.radDiff(leg1, leg2));

    double wall =
        cfg.wallPenaltyGain
            * ReactiveBypassWaypointPlanner.wallPenalty(cfg, waypoint.getTranslation());

    double rawLegOcc1 =
        ReactiveBypassProbing.legOcc(
            cfg,
            pose.getTranslation(),
            waypoint.getTranslation(),
            robotX,
            robotY,
            intersectsDynamicOnly);
    double rawLegOcc2 =
        ReactiveBypassProbing.legOcc(
            cfg,
            waypoint.getTranslation(),
            goal.getTranslation(),
            robotX,
            robotY,
            intersectsDynamicOnly);
    double occLeg = cfg.occCostGain * (rawLegOcc1 + rawLegOcc2);

    double rawLocalOcc =
        ReactiveBypassProbing.localOccAt(
            cfg,
            waypoint.getTranslation(),
            headingTowardGoal,
            robotX,
            robotY,
            intersectsDynamicOnly);
    double localOcc = cfg.occCostGain * 0.5 * rawLocalOcc;
    boolean sampledClear = rawLegOcc1 <= 0.0 && rawLegOcc2 <= 0.0 && rawLocalOcc <= 0.0;

    int side = ReactiveBypassMath.sideOf(pose, headingTowardGoal, waypoint.getTranslation());
    double sw = sideSwitchPenalty(cfg, preferredSide, timeSinceSideSwitchS, side, stuckNow);

    double yawDeltaDeg =
        Math.abs(Math.toDegrees(ReactiveBypassMath.radDiff(headingTowardGoal.getRadians(), leg1)));
    double zzz = yawDeltaDeg > cfg.zzzMaxYawDeltaDeg ? cfg.zzzPenalty : 0.0;

    Translation2d delta = waypoint.getTranslation().minus(pose.getTranslation());
    double forwardProgress =
        delta.getX() * Math.cos(headingTowardGoal.getRadians())
            + delta.getY() * Math.sin(headingTowardGoal.getRadians());
    double requiredForward =
        cfg.minForwardProgressMeters
            * (pose.getTranslation().getDistance(goal.getTranslation()) > cfg.nearGoalDistMeters
                ? 1.0
                : 0.5);
    double progressDeficit = Math.max(0.0, requiredForward - forwardProgress);
    double prog = cfg.progressCostWeight * progressDeficit;

    double wallHere = ReactiveBypassWaypointPlanner.wallPenalty(cfg, pose.getTranslation());
    double wallWp = ReactiveBypassWaypointPlanner.wallPenalty(cfg, waypoint.getTranslation());
    double cornerReward = 0.0;
    if (wallHere > cfg.cornerWallThresh) {
      double improve = wallHere - wallWp;
      if (improve > 0.0) cornerReward = -cfg.cornerRewardGain * improve;
    }

    double total =
        (ok1 && ok2 && sampledClear ? len : 1e9)
            + ang
            + curv
            + wall
            + occLeg
            + localOcc
            + sw
            + zzz
            + prog
            + cornerReward;

    return new ReactiveBypassScore(
        waypoint,
        ok1 && sampledClear,
        ok2 && sampledClear,
        len,
        ang,
        curv,
        wall,
        occLeg,
        sw,
        zzz,
        prog,
        localOcc,
        cornerReward,
        total);
  }

  private static double sideSwitchPenalty(
      ReactiveBypassConfig cfg,
      int preferredSide,
      double timeSinceSideSwitchS,
      int candidateSide,
      boolean stuckNow) {
    if (preferredSide == 0) return 0.0;
    if (candidateSide == preferredSide) return 0.0;
    if (stuckNow) {
      return cfg.sideSwitchPenalty * cfg.sideSwitchStuckPenaltyScale;
    }
    if (timeSinceSideSwitchS < cfg.sideStickSeconds) return 1e9;
    return cfg.sideSwitchPenalty;
  }
}
