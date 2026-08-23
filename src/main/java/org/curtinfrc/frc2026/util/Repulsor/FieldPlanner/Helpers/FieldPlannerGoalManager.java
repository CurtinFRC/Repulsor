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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.littletonrobotics.junction.Logger;

/**
 * Provides field planner goal manager functionality for the Repulsor repulsor-field planner that
 * combines goals, obstacles, and force samples. Use this type from robot code, field profiles, or
 * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public final class FieldPlannerGoalManager {
  private static final double STAGED_CENTER_BAND_M = 3.648981;
  private static final double STAGED_RESTAGE_DIST_M = 1.5;
  private static final double STAGED_SAME_GOAL_POS_M = 0.05;
  private static final double STAGED_SAME_GOAL_ROT_DEG = 5.0;

  private static final int STAGED_ENTRY_MIN_TICKS_BEFORE_PASS = 3;
  private static final double STAGED_ENTRY_PASS_MUST_BE_WITHIN_M = 0.85;

  private static final double STAGED_REACH_EXIT_M = 0.55;
  private static final double STAGED_ENTRY_REACH_ENTER_M = 0.50;
  private static final double STAGED_ENTRY_PASSED_PROJ_M = 0.06;
  private static final double STAGED_EXIT_REACH_ENTER_M = 0.45;
  private static final double STAGED_EXIT_PASSED_PROJ_M = 0.10;
  private static final double STAGED_GATE_CLEAR_EXIT_M = 0.40;
  private static final int STAGED_ENTRY_REACH_TICKS = 1;
  private static final int STAGED_EXIT_REACH_TICKS = 1;
  private static final int STAGED_GATE_CLEAR_TICKS = 2;

  private static final int STAGED_MAX_TICKS = 40;

  private static final double STAGED_LANE_WEIGHT = 2.0;
  private static final double STAGED_PREF_GATE_PENALTY = 2.0;
  private static final double STAGED_LANE_LOCK_WEIGHT = 2.5;
  private static final double STAGED_LANE_LOCK_MAX_DELTA_M = 3.0;

  private static final double STAGED_GATE_PAD_M = 0.25;
  private static final double STAGED_PASSED_X_HYST_M = 0.35;
  private static final double STAGED_GOAL_SIDE_PROJ_M = 0.05;
  private static final double STAGED_LEAD_THROUGH_SCALE = 0.28;
  private static final double STAGED_LEAD_THROUGH_MIN_M = 0.45;
  private static final double STAGED_LEAD_THROUGH_MAX_M = 1.05;
  private static final double STAGED_DEEP_CENTER_BAND_M = 1.40;
  private static final double STAGED_CENTER_RETURN_STAGE_TRIGGER_M = 3.0;
  private static final double STAGED_CENTER_RETURN_INTERSECTION_TRIGGER_M = 4.2;
  private static final double STAGED_CENTER_RETURN_EXIT_MIN_M = 0.70;
  private static final double STAGED_CENTER_RETURN_EXIT_MAX_M = 2.40;
  private static final double STAGED_CENTER_RETURN_GATE_MIN_OFFSET_M = 2.0;
  private static final double STAGED_FIELD_EDGE_MARGIN_M = 0.35;
  private static final FieldGeometry COMPATIBILITY_FIELD_GEOMETRY =
      new FieldGeometry(16.540988, 8.211236);

  private FieldPlannerWaypointConfig waypointConfig;
  private FieldPlannerWaypointStrategy waypointStrategy;

  private Pose2d goal = Pose2d.kZero;
  private Pose2d requestedGoal = Pose2d.kZero;
  private Translation2d stagedAttractor = null;
  private Translation2d lastStagedPoint = null;
  private boolean stagedComplete = false;
  private GatedAttractorObstacle stagedGate = null;
  private int stagedReachTicks = 0;
  private int stagedModeTicks = 0;
  private boolean stagedGatePassed = false;
  private int stagedGateClearTicks = 0;
  private Translation2d stagedLatchedPull = null;
  private boolean stagedUsingBypass = false;
  private Double stagedLaneY = null;
  private boolean stagedCenterReturn = false;
  private boolean stagedExitPhase = false;
  private Translation2d stagedExitPoint = null;
  private FieldPlannerWaypointDecision lastStrategyDecision =
      FieldPlannerWaypointDecision.useDefault();
  private FieldPlannerWaypointObjectiveRole lastObjectiveRole =
      FieldPlannerWaypointObjectiveRole.ANY;
  private String waypointTransitionReason = "initialized";

  private final List<GatedAttractorObstacle> gatedAttractors;
  private final double fieldLengthMeters;
  private final double fieldWidthMeters;

  /**
   * Returns the field planner goal manager value maintained by this Repulsor component.
   *
   * @param gatedAttractors value used by this operation.
   */
  public FieldPlannerGoalManager(List<GatedAttractorObstacle> gatedAttractors) {
    this(gatedAttractors, COMPATIBILITY_FIELD_GEOMETRY);
  }

  /**
   * Returns the field planner goal manager value maintained by this Repulsor component.
   *
   * @param gatedAttractors value used by this operation.
   * @param fieldGeometry distance or field-coordinate value in meters.
   */
  public FieldPlannerGoalManager(
      List<GatedAttractorObstacle> gatedAttractors, FieldGeometry fieldGeometry) {
    this(gatedAttractors, fieldGeometry, FieldPlannerWaypointConfig.defaults());
  }

  public FieldPlannerGoalManager(
      List<GatedAttractorObstacle> gatedAttractors,
      FieldGeometry fieldGeometry,
      FieldPlannerWaypointConfig waypointConfig) {
    this(
        gatedAttractors, fieldGeometry.lengthMeters(), fieldGeometry.widthMeters(), waypointConfig);
  }

  public FieldPlannerGoalManager(
      List<GatedAttractorObstacle> gatedAttractors,
      FieldGeometry fieldGeometry,
      FieldPlannerWaypointConfig waypointConfig,
      FieldPlannerWaypointStrategy waypointStrategy) {
    this(
        gatedAttractors,
        fieldGeometry.lengthMeters(),
        fieldGeometry.widthMeters(),
        waypointConfig,
        waypointStrategy);
  }

  /**
   * Returns the field planner goal manager value maintained by this Repulsor component.
   *
   * @param gatedAttractors value used by this operation.
   * @param fieldLengthMeters distance or field-coordinate value in meters.
   * @param fieldWidthMeters distance or field-coordinate value in meters.
   */
  public FieldPlannerGoalManager(
      List<GatedAttractorObstacle> gatedAttractors,
      double fieldLengthMeters,
      double fieldWidthMeters) {
    this(
        gatedAttractors,
        fieldLengthMeters,
        fieldWidthMeters,
        FieldPlannerWaypointConfig.defaults());
  }

  public FieldPlannerGoalManager(
      List<GatedAttractorObstacle> gatedAttractors,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldPlannerWaypointConfig waypointConfig) {
    this(gatedAttractors, fieldLengthMeters, fieldWidthMeters, waypointConfig, null);
  }

  public FieldPlannerGoalManager(
      List<GatedAttractorObstacle> gatedAttractors,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldPlannerWaypointConfig waypointConfig,
      FieldPlannerWaypointStrategy waypointStrategy) {
    this.gatedAttractors = gatedAttractors;
    this.fieldLengthMeters = fieldLengthMeters;
    this.fieldWidthMeters = fieldWidthMeters;
    this.waypointConfig =
        waypointConfig == null ? FieldPlannerWaypointConfig.defaults() : waypointConfig;
    this.waypointStrategy =
        waypointStrategy == null ? FieldPlannerWaypointStrategy.defaults() : waypointStrategy;
    Logger.recordOutput(
        "GoalManagerGatedAttractors",
        this.gatedAttractors.stream()
            .map(g -> new Pose2d(g.center, new Rotation2d()))
            .toArray(Pose2d[]::new));
  }

  /**
   * Returns the get goal pose value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Pose2d getGoalPose() {
    return goal;
  }

  /**
   * Returns the get requested goal pose value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Pose2d getRequestedGoalPose() {
    return requestedGoal;
  }

  /**
   * Returns the get goal translation value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Translation2d getGoalTranslation() {
    return goal.getTranslation();
  }

  public FieldPlannerWaypointConfig getWaypointConfig() {
    return waypointConfig;
  }

  public FieldPlannerWaypointStatus getWaypointStatus() {
    return new FieldPlannerWaypointStatus(
        requestedGoal,
        goal,
        lastStrategyDecision,
        lastObjectiveRole,
        waypointTransitionReason,
        stagedAttractor != null,
        stagedAttractor,
        stagedExitPoint,
        stagedGate == null ? null : stagedGate.center,
        stagedExitPhase,
        stagedComplete,
        stagedUsingBypass,
        stagedCenterReturn,
        stagedModeTicks);
  }

  public void setWaypointStrategy(FieldPlannerWaypointStrategy waypointStrategy) {
    this.waypointStrategy =
        waypointStrategy == null ? FieldPlannerWaypointStrategy.defaults() : waypointStrategy;
  }

  public void setWaypointPolicyProfile(FieldPlannerWaypointPolicyProfile profile) {
    if (profile == null) {
      this.waypointConfig = FieldPlannerWaypointConfig.defaults();
      setWaypointStrategy(FieldPlannerWaypointStrategy.defaults());
      waypointTransitionReason = "waypoint_profile_reset";
      return;
    }
    this.waypointConfig =
        profile.config() == null ? FieldPlannerWaypointConfig.defaults() : profile.config();
    setWaypointStrategy(profile.strategy());
    clearStagedState(false);
    waypointTransitionReason = "waypoint_profile_applied";
  }

  /**
   * Updates set requested goal state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param requested value used by this operation.
   */
  public void setRequestedGoal(Pose2d requested) {
    Logger.recordOutput("RequestedGoal", requested);
    boolean same = isPoseNear(this.requestedGoal, requested);
    this.requestedGoal = requested;

    if (!same) {
      this.stagedAttractor = null;
      this.lastStagedPoint = null;
      this.stagedComplete = false;
      this.stagedLaneY = null;
      this.stagedCenterReturn = false;
      this.stagedExitPhase = false;
      this.stagedExitPoint = null;
      this.waypointTransitionReason = "requested_goal_changed";
    }
  }

  /**
   * Updates set active goal state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param active value used by this operation.
   */
  public void setActiveGoal(Pose2d active) {
    this.goal = active;
    Logger.recordOutput("ActiveGoal", this.goal);
  }

  /**
   * Updates update staged goal state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param curPos value used by this operation.
   * @param obstacles obstacle set used for safety checks, costs, or replanning.
   * @return value produced by this operation.
   */
  public boolean updateStagedGoal(Translation2d curPos, List<? extends Obstacle> obstacles) {
    return updateStagedGoal(curPos, obstacles, FieldPlannerWaypointObjectiveRole.ANY);
  }

  public boolean updateStagedGoal(
      Translation2d curPos,
      List<? extends Obstacle> obstacles,
      FieldPlannerWaypointObjectiveRole objectiveRole) {
    if (objectiveRole == null) objectiveRole = FieldPlannerWaypointObjectiveRole.ANY;
    lastObjectiveRole = objectiveRole;
    Translation2d reqT = requestedGoal.getTranslation();
    var context =
        new FieldPlannerWaypointContext(
            curPos,
            requestedGoal,
            List.copyOf(gatedAttractors),
            obstacles == null ? List.of() : obstacles,
            fieldLengthMeters,
            fieldWidthMeters,
            objectiveRole,
            waypointConfig,
            stagedComplete,
            lastStagedPoint,
            stagedAttractor != null,
            stagedAttractor,
            stagedExitPhase);
    FieldPlannerWaypointDecision decision = waypointStrategy.decide(context);
    if (decision == null) decision = FieldPlannerWaypointDecision.useDefault();
    lastStrategyDecision = decision;
    if (decision.goesDirectlyToRequestedGoal()) {
      clearStagedState(false);
      goal = requestedGoal;
      waypointTransitionReason = "strategy_direct";
      return true;
    }
    if (decision.stages()) {
      if (shouldApplyWaypointPlan(decision.plan()) && applyWaypointPlan(decision.plan(), curPos)) {
        return false;
      }
      if (stagedAttractor != null) {
        // Strategy intentionally owns the active stage, but the requested waypoint is unchanged.
        // Continue the normal stage progression/release logic below.
        waypointTransitionReason = "strategy_stage_existing";
      } else {
        clearStagedState(false);
        goal = requestedGoal;
        waypointTransitionReason = "strategy_stage_suppressed";
        return true;
      }
    }
    if (!decision.usesDefaultPolicy() && stagedAttractor == null) {
      clearStagedState(false);
      goal = requestedGoal;
      waypointTransitionReason = "strategy_no_stage";
      return true;
    }

    if (gatedAttractors.isEmpty() && stagedAttractor == null) {
      goal = requestedGoal;
      clearStagedState(false);
      waypointTransitionReason = "no_gates_direct";
      return true;
    }

    GatedAttractorObstacle firstBlock = firstOccludingGateAlongSegment(curPos, reqT);

    Logger.recordOutput(
        "FirstOccludingGate",
        firstBlock != null ? new Pose2d(firstBlock.center, new Rotation2d()) : null);

    boolean stageForOccludingGate =
        waypointConfig.occludingGateStagingEnabled()
            && firstBlock != null
            && !shouldDeferCenterReturnStage(curPos, reqT, firstBlock);
    boolean stageByBand = shouldStageThroughAttractor(curPos, reqT);
    boolean shouldStage = stageByBand || stageForOccludingGate;
    boolean allowImmediateCenterExitRestage = stageByBand && isCenterReturnTransition(curPos, reqT);
    if (stagedComplete && lastStagedPoint != null) {
      double d = curPos.getDistance(lastStagedPoint);
      if (d < waypointConfig.restageDistanceMeters()
          && !stageForOccludingGate
          && !allowImmediateCenterExitRestage) {
        shouldStage = false;
        waypointTransitionReason = "restage_suppressed";
      } else {
        stagedComplete = false;
        lastStagedPoint = null;
      }
    }

    if (shouldStage && stagedAttractor == null) {
      GatedAttractorObstacle gateToUse =
          (firstBlock != null) ? firstBlock : chooseBestGateByScore(curPos, reqT, null);

      if (gateToUse != null) {
        stagedGate = gateToUse;
        stagedLatchedPull = null;
        if (stagedGate.center != null) stagedLaneY = stagedGate.center.getY();
        stagedCenterReturn =
            isCenterReturnTransition(curPos, reqT) && isCorridorSideGate(stagedGate);
        stagedExitPhase = false;
        Translation2d computedExit = stagingExitPoint(stagedGate, curPos, reqT);
        if (stagedCenterReturn) {
          Translation2d centerReturnExit = computeCenterReturnExitPoint(stagedGate, reqT);
          if (centerReturnExit != null) computedExit = centerReturnExit;
        }
        stagedExitPoint = computedExit;

        stagedUsingBypass =
            (stagedGate.gatePoly != null && stagedGate.bypassPoint != null)
                && FieldPlannerGeometry.segmentIntersectsPolygonOuter(
                    curPos,
                    reqT,
                    expandPoly(stagedGate.gatePoly, waypointConfig.gatePaddingMeters()));

        Translation2d pick = stagingEntryPoint(stagedGate, curPos, reqT);
        if (pick == null) pick = stagingPullPoint(stagedGate, curPos, reqT);

        boolean mustStageForOccludingGate = stageForOccludingGate;
        if (pick != null
            && (mustStageForOccludingGate
                || shouldStage
                || curPos.getDistance(pick) > STAGED_REACH_EXIT_M)) {
          stagedAttractor = pick;
          lastStagedPoint = pick;
          stagedReachTicks = 0;
          stagedModeTicks = 0;
          stagedGatePassed = false;
          stagedGateClearTicks = 0;
          stagedComplete = false;
          waypointTransitionReason = stageForOccludingGate ? "occluding_gate_stage" : "band_stage";

          Pose2d staged = new Pose2d(pick, requestedGoal.getRotation());
          setActiveGoal(staged);
          return false;
        } else {
          stagedAttractor = null;
          stagedGate = null;
          stagedUsingBypass = false;
          stagedGatePassed = false;
          stagedLatchedPull = null;
          stagedReachTicks = 0;
          stagedModeTicks = 0;
          stagedGateClearTicks = 0;
          stagedCenterReturn = false;
          stagedExitPhase = false;
          stagedExitPoint = null;
          waypointTransitionReason = "stage_candidate_unavailable";
        }
      }
    }

    if (stagedAttractor != null) {
      stagedModeTicks++;

      Translation2d liveTarget = stagedAttractor;
      double passedProjMeters =
          stagedExitPhase ? STAGED_EXIT_PASSED_PROJ_M : STAGED_ENTRY_PASSED_PROJ_M;
      boolean passedLiveTargetTowardGoal =
          hasPassedPointTowardGoal(curPos, liveTarget, reqT, passedProjMeters);

      double d = curPos.getDistance(liveTarget);

      double reachEnter = stagedExitPhase ? STAGED_EXIT_REACH_ENTER_M : STAGED_ENTRY_REACH_ENTER_M;
      int reachTicksRequired = stagedExitPhase ? STAGED_EXIT_REACH_TICKS : STAGED_ENTRY_REACH_TICKS;

      if (d <= reachEnter) stagedReachTicks++;
      else if (d >= STAGED_REACH_EXIT_M) stagedReachTicks = 0;

      boolean reached = stagedReachTicks >= reachTicksRequired;

      if (!reached && !stagedExitPhase && passedLiveTargetTowardGoal) {
        boolean committed =
            (d <= STAGED_ENTRY_PASS_MUST_BE_WITHIN_M)
                || (stagedModeTicks >= STAGED_ENTRY_MIN_TICKS_BEFORE_PASS);
        if (committed) reached = true;
      }

      if (stagedGate != null) {
        boolean gateOccludingNow =
            stagedGate.gatePoly != null
                && FieldPlannerGeometry.segmentIntersectsPolygonOuter(
                    curPos,
                    reqT,
                    expandPoly(stagedGate.gatePoly, waypointConfig.gatePaddingMeters()));
        stagedGatePassed =
            stagedGatePassed
                || gateIsBehind(curPos, reqT, stagedGate)
                || gateOnGoalSide(curPos, reqT, stagedGate)
                || !gateOccludingNow
                || passedLiveTargetTowardGoal;
      }

      boolean gateCleared = (stagedGate == null) || stagedGatePassed;
      if (gateCleared) stagedGateClearTicks++;
      else stagedGateClearTicks = 0;

      if (!reached
          && stagedGate != null
          && stagedGateClearTicks >= STAGED_GATE_CLEAR_TICKS
          && (passedLiveTargetTowardGoal || d <= STAGED_GATE_CLEAR_EXIT_M)) {
        reached = true;
      }
      if (!reached && stagedGate == null && passedLiveTargetTowardGoal) reached = true;

      if (stagedModeTicks >= STAGED_MAX_TICKS) {
        GatedAttractorObstacle nowFirst = firstOccludingGateAlongSegment(curPos, reqT);
        if (nowFirst == null || stagedGatePassed) {
          reached = true;
          gateCleared = true;
        } else if (stagedGate != null && nowFirst != stagedGate) {
          stagedGate = nowFirst;
          stagedLatchedPull = null;

          stagedUsingBypass =
              (stagedGate.gatePoly != null && stagedGate.bypassPoint != null)
                  && FieldPlannerGeometry.segmentIntersectsPolygonOuter(
                      curPos,
                      reqT,
                      expandPoly(stagedGate.gatePoly, waypointConfig.gatePaddingMeters()));

          Translation2d repick = stagingEntryPoint(stagedGate, curPos, reqT);
          if (repick == null) repick = stagingPullPoint(stagedGate, curPos, reqT);
          if (repick != null) {
            Translation2d nextExit = stagingExitPoint(stagedGate, curPos, reqT);
            if (stagedCenterReturn) {
              Translation2d centerReturnExit = computeCenterReturnExitPoint(stagedGate, reqT);
              if (centerReturnExit != null) nextExit = centerReturnExit;
            }
            stagedExitPoint = nextExit;
            stagedExitPhase = false;
            stagedAttractor = repick;
            lastStagedPoint = repick;
            stagedReachTicks = 0;
            stagedModeTicks = 0;
            stagedGatePassed = false;
            stagedGateClearTicks = 0;
            goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
            waypointTransitionReason = "stage_repicked";
            return false;
          }
        }
      }

      if (reached
          && !stagedExitPhase
          && stagedExitPoint != null
          && (stagedAttractor == null
              || stagedAttractor.getDistance(stagedExitPoint) > STAGED_REACH_EXIT_M)) {
        stagedExitPhase = true;
        stagedAttractor = stagedExitPoint;
        lastStagedPoint = stagedExitPoint;
        stagedGate = null;
        stagedUsingBypass = false;
        stagedGatePassed = true;
        stagedLatchedPull = null;
        stagedReachTicks = 0;
        stagedModeTicks = 0;
        stagedGateClearTicks = 0;
        goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
        waypointTransitionReason = "stage_exit_phase";
        return false;
      }
      if (reached && gateCleared) {
        if (stagedCenterReturn && !stagedExitPhase && stagedExitPoint != null) {
          stagedExitPhase = true;
          stagedAttractor = stagedExitPoint;
          lastStagedPoint = stagedExitPoint;
          stagedGate = null;
          stagedUsingBypass = false;
          stagedGatePassed = true;
          stagedLatchedPull = null;
          stagedReachTicks = 0;
          stagedModeTicks = 0;
          stagedGateClearTicks = 0;
          goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
          waypointTransitionReason = "center_return_exit_phase";
          return false;
        }

        lastStagedPoint = liveTarget != null ? liveTarget : stagedAttractor;
        stagedAttractor = null;
        stagedGate = null;
        stagedUsingBypass = false;
        stagedGatePassed = false;
        stagedLatchedPull = null;
        stagedReachTicks = 0;
        stagedModeTicks = 0;
        stagedGateClearTicks = 0;
        stagedComplete = true;
        stagedCenterReturn = false;
        stagedExitPhase = false;
        stagedExitPoint = null;

        goal = requestedGoal;
        waypointTransitionReason = "stage_complete";
        return true;
      }

      goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());

      waypointTransitionReason = stagedExitPhase ? "stage_exit_holding" : "stage_entry_holding";
      return false;
    }

    goal = requestedGoal;
    waypointTransitionReason = "default_direct";
    return true;
  }

  private void clearStagedState(boolean complete) {
    stagedAttractor = null;
    stagedGate = null;
    stagedUsingBypass = false;
    stagedGatePassed = false;
    stagedLatchedPull = null;
    if (!complete) lastStagedPoint = null;
    stagedComplete = complete;
    stagedReachTicks = 0;
    stagedModeTicks = 0;
    stagedGateClearTicks = 0;
    stagedLaneY = null;
    stagedCenterReturn = false;
    stagedExitPhase = false;
    stagedExitPoint = null;
  }

  private boolean shouldApplyWaypointPlan(FieldPlannerWaypointPlan plan) {
    if (plan == null || plan.entryPoint() == null) return false;
    if (stagedAttractor == null) return true;
    if (stagedAttractor.getDistance(plan.entryPoint()) > 0.02) return true;
    if (stagedExitPoint == null) return plan.exitPoint() != null;
    if (plan.exitPoint() == null) return true;
    return stagedExitPoint.getDistance(plan.exitPoint()) > 0.02;
  }

  private boolean applyWaypointPlan(FieldPlannerWaypointPlan plan, Translation2d curPos) {
    if (plan == null || plan.entryPoint() == null) {
      waypointTransitionReason = "strategy_stage_invalid";
      return false;
    }
    Translation2d pick = clampToField(plan.entryPoint());
    if (pick == null) {
      waypointTransitionReason = "strategy_stage_out_of_field";
      return false;
    }

    if (!plan.forceStage() && curPos != null && curPos.getDistance(pick) <= STAGED_REACH_EXIT_M) {
      waypointTransitionReason = "strategy_stage_suppressed_near_entry";
      return false;
    }

    stagedGate = plan.gate();
    stagedLatchedPull = null;
    if (stagedGate != null && stagedGate.center != null) stagedLaneY = stagedGate.center.getY();
    stagedCenterReturn = plan.centerReturn();
    stagedExitPhase = false;
    stagedExitPoint = plan.exitPoint() == null ? null : clampToField(plan.exitPoint());
    stagedUsingBypass = plan.usingBypass();
    stagedAttractor = pick;
    lastStagedPoint = pick;
    stagedReachTicks = 0;
    stagedModeTicks = 0;
    stagedGatePassed = stagedGate == null;
    stagedGateClearTicks = 0;
    stagedComplete = false;

    setActiveGoal(new Pose2d(pick, requestedGoal.getRotation()));
    Logger.recordOutput("CustomWaypointStage", new Pose2d(pick, requestedGoal.getRotation()));
    waypointTransitionReason =
        plan.centerReturn() ? "strategy_center_return_stage" : "strategy_stage";
    return true;
  }

  private static Translation2d polyCentroid(Translation2d[] poly) {
    if (poly == null || poly.length == 0) return null;
    double sx = 0.0, sy = 0.0;
    for (Translation2d p : poly) {
      sx += p.getX();
      sy += p.getY();
    }
    return new Translation2d(sx / poly.length, sy / poly.length);
  }

  private static Translation2d[] expandPoly(Translation2d[] poly, double pad) {
    if (poly == null || poly.length == 0) return poly;
    if (pad <= 1e-9) return poly;

    Translation2d c = polyCentroid(poly);
    if (c == null) return poly;

    Translation2d[] out = new Translation2d[poly.length];
    for (int i = 0; i < poly.length; i++) {
      Translation2d v = poly[i].minus(c);
      double n = v.getNorm();
      if (n < 1e-9) out[i] = poly[i];
      else out[i] = c.plus(v.div(n).times(n + pad));
    }
    return out;
  }

  private static double firstIntersectionT(Translation2d a, Translation2d b, Translation2d[] poly) {
    if (poly == null || poly.length < 3) return Double.POSITIVE_INFINITY;

    if (FieldPlannerGeometry.isPointInPolygon(a, poly)) return 0.0;

    double bestT = Double.POSITIVE_INFINITY;
    for (int i = 0; i < poly.length; i++) {
      Translation2d c = poly[i];
      Translation2d d = poly[(i + 1) % poly.length];
      Double t = segmentIntersectionParam(a, b, c, d);
      if (t != null && t >= 0.0 && t <= 1.0 && t < bestT) bestT = t;
    }
    return bestT;
  }

  private static Double segmentIntersectionParam(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {

    double ax = a.getX(), ay = a.getY();
    double bx = b.getX(), by = b.getY();
    double cx = c.getX(), cy = c.getY();
    double dx = d.getX(), dy = d.getY();

    double rpx = bx - ax;
    double rpy = by - ay;
    double spx = dx - cx;
    double spy = dy - cy;

    double rxs = rpx * spy - rpy * spx;
    double qpx = cx - ax;
    double qpy = cy - ay;

    double qpxr = qpx * rpy - qpy * rpx;

    double eps = 1e-9;
    if (Math.abs(rxs) < eps) {
      return null;
    }

    double t = (qpx * spy - qpy * spx) / rxs;
    double u = qpxr / rxs;

    if (t >= -eps && t <= 1.0 + eps && u >= -eps && u <= 1.0 + eps) {
      if (t < 0.0) t = 0.0;
      if (t > 1.0) t = 1.0;
      return t;
    }

    return null;
  }

  private static boolean gateIsBehind(
      Translation2d pos, Translation2d goal, GatedAttractorObstacle gate) {
    if (pos == null || goal == null || gate == null || gate.center == null) return false;
    Translation2d toGoal = goal.minus(pos);
    double n = toGoal.getNorm();
    if (n <= 1e-6) return false;
    Translation2d dir = toGoal.div(n);
    Translation2d toGate = gate.center.minus(pos);
    double proj = toGate.getX() * dir.getX() + toGate.getY() * dir.getY();
    return proj < -STAGED_PASSED_X_HYST_M;
  }

  private static boolean gateOnGoalSide(
      Translation2d pos, Translation2d goal, GatedAttractorObstacle gate) {
    if (pos == null || goal == null || gate == null || gate.center == null) return false;
    Translation2d gateToGoal = goal.minus(gate.center);
    double n = gateToGoal.getNorm();
    if (n <= 1e-6) return false;
    Translation2d gateToPos = pos.minus(gate.center);
    double proj = (gateToPos.getX() * gateToGoal.getX() + gateToPos.getY() * gateToGoal.getY()) / n;
    return proj >= STAGED_GOAL_SIDE_PROJ_M;
  }

  private GatedAttractorObstacle firstOccludingGateAlongSegment(
      Translation2d pos, Translation2d target) {
    if (gatedAttractors.isEmpty() || pos == null || target == null) return null;

    GatedAttractorObstacle best = null;
    double bestT = Double.POSITIVE_INFINITY;
    double bestLaneMetric = Double.POSITIVE_INFINITY;
    double laneY = 0.5 * (pos.getY() + target.getY());
    final double tieEps = 1e-6;

    for (GatedAttractorObstacle gate : gatedAttractors) {
      if (gate == null || gate.gatePoly == null) continue;
      if (gateIsBehind(pos, target, gate)) continue;

      Translation2d[] poly = expandPoly(gate.gatePoly, waypointConfig.gatePaddingMeters());

      if (!FieldPlannerGeometry.segmentIntersectsPolygonOuter(pos, target, poly)) continue;
      double t = firstIntersectionT(pos, target, poly);
      if (t < bestT - tieEps) {
        bestT = t;
        bestLaneMetric = laneDistanceMetric(gate, laneY);
        best = gate;
      } else if (Math.abs(t - bestT) <= tieEps) {
        double laneMetric = laneDistanceMetric(gate, laneY);
        if (laneMetric < bestLaneMetric) {
          bestLaneMetric = laneMetric;
          best = gate;
        }
      }
    }

    return best;
  }

  private int sideSignXBand(double x, double band) {
    double mid = fieldLengthMeters * 0.5;
    double b = Math.max(0.0, band);
    if (x < mid - b) return -1;
    if (x > mid + b) return 1;
    return 0;
  }

  private static boolean isPoseNear(Pose2d a, Pose2d b) {
    if (a == null || b == null) return false;
    if (a.getTranslation().getDistance(b.getTranslation()) > STAGED_SAME_GOAL_POS_M) return false;
    double rotDeg =
        Math.abs(
            MathUtil.angleModulus(a.getRotation().getRadians() - b.getRotation().getRadians()));
    return rotDeg <= Math.toRadians(STAGED_SAME_GOAL_ROT_DEG);
  }

  private boolean shouldStageThroughAttractor(Translation2d pos, Translation2d target) {
    if (!waypointConfig.bandTransitionStagingEnabled()) return false;
    int goalSide = sideSignXBand(target.getX(), waypointConfig.centerBandMeters());
    int robotSide = sideSignXBand(pos.getX(), waypointConfig.centerBandMeters());
    if (goalSide == 0 && robotSide != 0) return true;
    // When exiting deep center toward an alliance side, avoid forced staging.
    // This reduces stop/slow behavior in open corridor return paths.
    if (goalSide != 0 && robotSide == 0) {
      double mid = fieldLengthMeters * 0.5;
      boolean deepCenter = Math.abs(pos.getX() - mid) <= waypointConfig.deepCenterBandMeters();
      return !deepCenter;
    }
    return goalSide != 0 && robotSide != 0 && goalSide != robotSide;
  }

  private boolean shouldDeferCenterReturnStage(
      Translation2d pos, Translation2d target, GatedAttractorObstacle gate) {
    if (pos == null || target == null || gate == null || gate.center == null) return false;

    if (!waypointConfig.centerReturnStagingEnabled()) return false;
    int goalSide = sideSignXBand(target.getX(), waypointConfig.centerBandMeters());
    int robotSide = sideSignXBand(pos.getX(), waypointConfig.centerBandMeters());
    if (!(goalSide != 0 && robotSide == 0)) return false;

    double mid = fieldLengthMeters * 0.5;
    boolean deepCenter = Math.abs(pos.getX() - mid) <= waypointConfig.deepCenterBandMeters();
    if (!deepCenter) return false;

    Translation2d[] poly = expandPoly(gate.gatePoly, waypointConfig.gatePaddingMeters());
    double t = firstIntersectionT(pos, target, poly);
    if (Double.isFinite(t) && t >= 0.0 && t <= 1.0) {
      double segDist = pos.getDistance(target);
      double hitDist = segDist * t;
      return hitDist > waypointConfig.centerReturnIntersectionTriggerMeters();
    }

    return pos.getDistance(gate.center) > waypointConfig.centerReturnStageTriggerMeters();
  }

  private boolean isCenterReturnTransition(Translation2d pos, Translation2d target) {
    if (!waypointConfig.centerReturnStagingEnabled()) return false;
    int goalSide = sideSignXBand(target.getX(), waypointConfig.centerBandMeters());
    int robotSide = sideSignXBand(pos.getX(), waypointConfig.centerBandMeters());
    return goalSide != 0 && robotSide == 0;
  }

  private boolean isCorridorSideGate(GatedAttractorObstacle gate) {
    if (gate == null || gate.center == null) return false;
    double mid = fieldLengthMeters * 0.5;
    return Math.abs(gate.center.getX() - mid) >= waypointConfig.centerReturnGateMinOffsetMeters();
  }

  private Translation2d computeCenterReturnExitPoint(
      GatedAttractorObstacle gate, Translation2d target) {
    if (gate == null || gate.center == null || target == null) return null;

    double dx = target.getX() - gate.center.getX();
    double sign = Math.signum(dx);
    if (sign == 0.0) {
      double mid = fieldLengthMeters * 0.5;
      sign = gate.center.getX() >= mid ? 1.0 : -1.0;
    }

    double advance =
        MathUtil.clamp(
            Math.abs(dx) * 0.45,
            waypointConfig.centerReturnExitMinMeters(),
            waypointConfig.centerReturnExitMaxMeters());

    double x =
        MathUtil.clamp(
            gate.center.getX() + sign * advance,
            waypointConfig.fieldEdgeMarginMeters(),
            fieldLengthMeters - waypointConfig.fieldEdgeMarginMeters());
    double yBase = stagedLaneY != null ? stagedLaneY.doubleValue() : gate.center.getY();
    double y =
        MathUtil.clamp(
            yBase,
            waypointConfig.fieldEdgeMarginMeters(),
            fieldWidthMeters - waypointConfig.fieldEdgeMarginMeters());

    return new Translation2d(x, y);
  }

  private static boolean isRightOfGate(
      Translation2d point, Translation2d gateCenter, Translation2d fallbackPoint) {
    if (gateCenter == null) return true;
    double dx = 0.0;
    if (point != null) dx = point.getX() - gateCenter.getX();
    if (Math.abs(dx) <= 1e-6 && fallbackPoint != null)
      dx = fallbackPoint.getX() - gateCenter.getX();
    if (Math.abs(dx) <= 1e-6) dx = 1.0;
    return dx >= 0.0;
  }

  private Translation2d clampToField(Translation2d p) {
    if (p == null) return null;
    return new Translation2d(
        MathUtil.clamp(
            p.getX(),
            waypointConfig.fieldEdgeMarginMeters(),
            fieldLengthMeters - waypointConfig.fieldEdgeMarginMeters()),
        MathUtil.clamp(
            p.getY(),
            waypointConfig.fieldEdgeMarginMeters(),
            fieldWidthMeters - waypointConfig.fieldEdgeMarginMeters()));
  }

  private static Translation2d gateSidePoint(GatedAttractorObstacle gate, boolean rightSide) {
    if (gate == null || gate.center == null) return null;
    if (gate.bypassPoint == null) return gate.center;

    Translation2d center = gate.center;
    Translation2d inside = gate.bypassPoint;
    Translation2d outside = new Translation2d(2.0 * center.getX() - inside.getX(), inside.getY());

    boolean insideOnRight = inside.getX() >= center.getX();
    Translation2d rightPoint = insideOnRight ? inside : outside;
    Translation2d leftPoint = insideOnRight ? outside : inside;
    return rightSide ? rightPoint : leftPoint;
  }

  private Translation2d stagingEntryPoint(
      GatedAttractorObstacle gate, Translation2d pos, Translation2d target) {
    if (gate == null || gate.center == null) return null;
    if (gate.gatePoly == null || gate.bypassPoint == null) return gate.center;

    boolean robotOnRight = isRightOfGate(pos, gate.center, target);
    Translation2d pick = gateSidePoint(gate, robotOnRight);
    if (pick == null) pick = gate.center;
    return clampToField(pick);
  }

  private Translation2d stagingExitPoint(
      GatedAttractorObstacle gate, Translation2d pos, Translation2d target) {
    if (gate == null || gate.center == null) return null;

    Translation2d pick;
    if (gate.gatePoly == null || gate.bypassPoint == null) {
      pick = gate.center;
    } else {
      boolean targetOnRight = isRightOfGate(target, gate.center, pos);
      pick = gateSidePoint(gate, targetOnRight);
    }
    if (pick == null) return null;

    pick = extendPullPointThroughGate(pick, gate.center, target);
    return clampToField(pick);
  }

  private Translation2d stagingPullPoint(
      GatedAttractorObstacle gate, Translation2d pos, Translation2d target) {
    if (gate == null) return null;
    Translation2d center = gate.center;
    if (center == null) return null;

    if (gate.gatePoly == null || gate.bypassPoint == null) return center;
    if (pos == null || target == null) return center;

    if (gate == stagedGate && stagedLatchedPull != null) return stagedLatchedPull;

    Translation2d[] poly = expandPoly(gate.gatePoly, waypointConfig.gatePaddingMeters());
    boolean hit = FieldPlannerGeometry.segmentIntersectsPolygonOuter(pos, target, poly);

    Translation2d inside = gate.bypassPoint;
    Translation2d outside = new Translation2d(2.0 * center.getX() - inside.getX(), inside.getY());

    boolean targetOnRight = target.getX() > center.getX();
    boolean insideOnRight = inside.getX() > center.getX();

    Translation2d pick =
        hit
            ? (targetOnRight
                ? (insideOnRight ? inside : outside)
                : (insideOnRight ? outside : inside))
            : center;

    pick = extendPullPointThroughGate(pick, center, target);

    if (gate == stagedGate) stagedLatchedPull = pick;
    return pick;
  }

  private static boolean hasPassedPointTowardGoal(
      Translation2d pos, Translation2d point, Translation2d goal, double projMeters) {
    if (pos == null || point == null || goal == null) return false;

    Translation2d toGoal = goal.minus(point);
    double n = toGoal.getNorm();
    if (n <= 1e-6) return pos.getDistance(point) <= Math.max(0.0, projMeters);

    Translation2d dir = toGoal.div(n);
    Translation2d toPos = pos.minus(point);
    double proj = toPos.getX() * dir.getX() + toPos.getY() * dir.getY();
    return proj >= Math.max(0.0, projMeters);
  }

  private Translation2d extendPullPointThroughGate(
      Translation2d pullPoint, Translation2d gateCenter, Translation2d target) {
    if (pullPoint == null || gateCenter == null || target == null) return pullPoint;

    double dx = target.getX() - gateCenter.getX();
    double sign = Math.signum(dx);
    if (Math.abs(sign) < 1e-9) return pullPoint;

    double lead =
        MathUtil.clamp(
            Math.abs(dx) * waypointConfig.leadThroughScale(),
            waypointConfig.leadThroughMinMeters(),
            waypointConfig.leadThroughMaxMeters());

    double x =
        MathUtil.clamp(
            pullPoint.getX() + sign * lead,
            waypointConfig.fieldEdgeMarginMeters(),
            fieldLengthMeters - waypointConfig.fieldEdgeMarginMeters());
    double y =
        MathUtil.clamp(
            pullPoint.getY(),
            waypointConfig.fieldEdgeMarginMeters(),
            fieldWidthMeters - waypointConfig.fieldEdgeMarginMeters());
    return new Translation2d(x, y);
  }

  private GatedAttractorObstacle chooseBestGateByScore(
      Translation2d pos, Translation2d target, GatedAttractorObstacle preferredGate) {

    GatedAttractorObstacle best = null;
    double bestScore = Double.POSITIVE_INFINITY;

    double laneY = 0.5 * (pos.getY() + target.getY());

    for (GatedAttractorObstacle gate : gatedAttractors) {
      if (gate == null) continue;
      if (gateIsBehind(pos, target, gate)) continue;

      Translation2d pullTo = stagingPullPoint(gate, pos, target);
      if (pullTo == null) continue;

      double prefPenalty =
          (preferredGate != null && gate != preferredGate) ? STAGED_PREF_GATE_PENALTY : 0.0;
      double lanePenalty = STAGED_LANE_WEIGHT * Math.abs(gate.center.getY() - laneY);
      double laneLockPenalty = laneLockPenalty(gate);

      double score =
          pos.getDistance(pullTo)
              + pullTo.getDistance(target)
              + prefPenalty
              + lanePenalty
              + laneLockPenalty;

      if (score < bestScore) {
        bestScore = score;
        best = gate;
      }
    }

    Logger.recordOutput("ChosenStagedGate", best == null ? null : best.center);
    return best;
  }

  private double laneDistanceMetric(GatedAttractorObstacle gate, double laneY) {
    if (gate == null || gate.center == null) return Double.POSITIVE_INFINITY;
    return Math.abs(gate.center.getY() - laneY) + laneLockPenalty(gate);
  }

  private double laneLockPenalty(GatedAttractorObstacle gate) {
    if (stagedLaneY == null || gate == null || gate.center == null) return 0.0;
    double dy = Math.abs(gate.center.getY() - stagedLaneY.doubleValue());
    if (dy <= 1e-6) return 0.0;
    double scaled = Math.min(dy, STAGED_LANE_LOCK_MAX_DELTA_M) / STAGED_LANE_LOCK_MAX_DELTA_M;
    return STAGED_LANE_LOCK_WEIGHT * scaled;
  }
}
