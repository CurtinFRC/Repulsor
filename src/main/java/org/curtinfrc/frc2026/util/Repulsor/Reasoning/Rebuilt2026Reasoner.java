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

package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.EnumSet;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourContext;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourFlag;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;
import org.curtinfrc.frc2026.util.Repulsor.State.GameState;
import org.curtinfrc.frc2026.util.Repulsor.State.StateManager;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Decision;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Inputs;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Intent;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Tuning;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.ResourceRegionSummary;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.StrategyDirective;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;

/**
 * Provides rebuilt2026 reasoner functionality for the Repulsor rule-based strategy and signal
 * reasoning layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class Rebuilt2026Reasoner
    implements Reasoner<BehaviourFlag, BehaviourContext>, AutoCloseable {
  private static final SignalKey<Boolean> WANT_DEFENSE = ReasoningKeys.boolKey("want_defense");
  private static final SignalKey<Boolean> WANT_AUTOPATH = ReasoningKeys.boolKey("want_autopath");
  private static final SignalKey<Boolean> WANT_SHUTTLE = ReasoningKeys.boolKey("want_shuttle");
  private static final SignalKey<Boolean> WANT_SHUTTLE_RECOVERY =
      ReasoningKeys.boolKey("want_shuttle_recovery");
  private static final SignalKey<Boolean> TESTING = ReasoningKeys.boolKey("testing");
  private static final SignalKey<Boolean> AUTO_WANT_SHUTTLE =
      ReasoningKeys.boolKey("auto_want_shuttle");
  private static final SignalKey<Boolean> AUTO_WANT_SHUTTLE_RECOVERY =
      ReasoningKeys.boolKey("auto_want_shuttle_recovery");
  private static final SignalKey<Boolean> TRANSFER_ACTION_AVAILABLE =
      ReasoningKeys.boolKey("transfer_action_available");
  private static final SignalKey<Boolean> SCORE_ACTION_AVAILABLE =
      ReasoningKeys.boolKey("score_action_available");
  private static final SignalKey<Boolean> HUB_ACTIVE = ReasoningKeys.boolKey("hub_active");
  private static final SignalKey<Boolean> HAS_PIECE = ReasoningKeys.boolKey("has_piece");
  private static final SignalKey<Long> PIECE_COUNT = ReasoningKeys.longKey("piece_count");
  private static final SignalKey<Double> REMAINING_SHIFT_TIME =
      ReasoningKeys.doubleKey("remaining_shift_time");
  private static final SignalKey<String> SELECTED_MODE = ReasoningKeys.stringKey("selected_mode");
  private static final SignalKey<String> CYCLE_INTENT = ReasoningKeys.stringKey("cycle_intent");
  private static final SignalKey<Double> SCORE_OPTION_SCORE =
      ReasoningKeys.doubleKey("score_option_score");
  private static final SignalKey<Double> TRANSFER_OPTION_SCORE =
      ReasoningKeys.doubleKey("transfer_option_score");
  private static final SignalKey<Double> ALLIANCE_SIDE_FUEL_UNITS =
      ReasoningKeys.doubleKey("alliance_side_fuel_units");
  private static final SignalKey<Double> CENTER_FUEL_UNITS =
      ReasoningKeys.doubleKey("center_fuel_units");

  private static final int PH_SHUTTLE = 0;
  private static final int PH_SHUTTLE_RECOVERY = 1;
  private static final int PH_AUTOPATH = 2;
  private static final int PH_DEFENSE = 3;

  private static final double ALLIANCE_ZONE_X_FRACTION = 0.42;
  private static final double CENTER_ZONE_HALF_WIDTH_FRACTION = 0.16;

  private final NetworkTablesSignals nt;
  private final NetworkTablesValue<Long> pieceCount;
  private final SequenceReasoner<BehaviourFlag, BehaviourContext> seq;
  private Intent currentCycleIntent = Intent.FALLBACK;

  /** Returns the rebuilt2026 reasoner value maintained by this Repulsor component. */
  public Rebuilt2026Reasoner() {
    this(NetworkTableInstance.getDefault(), "/Repulsor/Reasoning");
  }

  /**
   * Returns the rebuilt2026 reasoner value maintained by this Repulsor component.
   *
   * @param inst value used by this operation.
   * @param basePath value used by this operation.
   */
  public Rebuilt2026Reasoner(NetworkTableInstance inst, String basePath) {
    NetworkTablesSignals nts = new NetworkTablesSignals(inst, basePath);
    nts.register(ReasoningKeys.ENABLED, false);
    nts.register(ReasoningKeys.AUTO, false);
    nts.register(ReasoningKeys.TELEOP, false);
    nts.register(ReasoningKeys.ENDGAME, false);
    nts.register(WANT_DEFENSE, false);
    nts.register(WANT_AUTOPATH, false);
    nts.register(WANT_SHUTTLE, false);
    nts.register(WANT_SHUTTLE_RECOVERY, false);
    nts.register(TESTING, false);
    nts.register(AUTO_WANT_SHUTTLE, false);
    nts.register(AUTO_WANT_SHUTTLE_RECOVERY, false);
    nts.register(TRANSFER_ACTION_AVAILABLE, false);
    nts.register(SCORE_ACTION_AVAILABLE, false);
    nts.register(HUB_ACTIVE, false);
    nts.register(HAS_PIECE, false);
    nts.register(PIECE_COUNT, 0L);
    nts.register(REMAINING_SHIFT_TIME, 0.0);
    nts.register(SELECTED_MODE, "");
    nts.register(CYCLE_INTENT, Intent.FALLBACK.name());
    nts.register(SCORE_OPTION_SCORE, 0.0);
    nts.register(TRANSFER_OPTION_SCORE, 0.0);
    nts.register(ALLIANCE_SIDE_FUEL_UNITS, 0.0);
    nts.register(CENTER_FUEL_UNITS, 0.0);
    this.nt = nts;
    this.pieceCount = NetworkTablesValue.ofInteger(inst, "/PieceCount", 0L);

    Clock clock = new WpiClock();

    SequenceReasoner.Builder<BehaviourFlag, BehaviourContext> b =
        new SequenceReasoner.Builder<BehaviourFlag, BehaviourContext>(
                BehaviourFlag.class, clock, nt)
            .startAt(PH_SHUTTLE);

    b.addPhase(
        "transfer_to_score_zone", EnumSet.of(BehaviourFlag.SHUTTLE_MODE), 0.0, 1e9, PH_SHUTTLE);
    b.addPhase(
        "recover_and_score_transfers",
        EnumSet.of(BehaviourFlag.SHUTTLE_RECOVERY_MODE),
        0.0,
        1e9,
        PH_SHUTTLE_RECOVERY);
    b.addPhase("autopath", EnumSet.of(BehaviourFlag.AUTOPATH_MODE), 0.0, 1e9, PH_AUTOPATH);
    b.addPhase("defense", EnumSet.of(BehaviourFlag.DEFENCE_MODE), 0.0, 1e9, PH_DEFENSE);

    addModeTransitions(b, PH_SHUTTLE, "transfer");
    addModeTransitions(b, PH_SHUTTLE_RECOVERY, "recovery");
    addModeTransitions(b, PH_AUTOPATH, "autopath");
    addModeTransitions(b, PH_DEFENSE, "defense");

    this.seq = b.build();
  }

  /**
   * Returns the signals value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Signals signals() {
    return seq.signals();
  }

  /**
   * Updates set want defense state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param v value used by this operation.
   */
  public void setWantDefense(boolean v) {
    seq.signals().put(WANT_DEFENSE, v);
    seq.signals().flush();
  }

  /**
   * Updates set want autopath state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param v value used by this operation.
   */
  public void setWantAutopath(boolean v) {
    seq.signals().put(WANT_AUTOPATH, v);
    seq.signals().flush();
  }

  /**
   * Updates set want shuttle state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param v value used by this operation.
   */
  public void setWantShuttle(boolean v) {
    seq.signals().put(WANT_SHUTTLE, v);
    seq.signals().flush();
  }

  /**
   * Updates set want shuttle recovery state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param v value used by this operation.
   */
  public void setWantShuttleRecovery(boolean v) {
    seq.signals().put(WANT_SHUTTLE_RECOVERY, v);
    seq.signals().flush();
  }

  /**
   * Updates set testing state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param v value used by this operation.
   */
  public void setTesting(boolean v) {
    seq.signals().put(TESTING, v);
    seq.signals().flush();
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return enum set of behaviour flag result for update.
   */
  @Override
  public EnumSet<BehaviourFlag> update(BehaviourContext ctx) {
    Signals signals = seq.signals();
    signals.put(ReasoningKeys.ENABLED, DriverStation.isEnabled());
    signals.put(ReasoningKeys.AUTO, DriverStation.isAutonomous());
    signals.put(ReasoningKeys.TELEOP, DriverStation.isTeleop());

    boolean transferAvailable =
        ctx != null
            && ctx.repulsor != null
            && ctx.repulsor
                .getFieldDefinition()
                .actionProfile()
                .transferProjectileShot()
                .isPresent();
    boolean scoringAvailable =
        ctx != null
            && ctx.repulsor != null
            && ctx.repulsor.getFieldDefinition().actionProfile().scoreProjectileShot().isPresent();

    GameState gameState = StateManager.getState(GameState.class);
    boolean hubActive = gameState != null && gameState.isHubActive();
    double remainingShiftTime = gameState != null ? gameState.getRemainingShiftTime() : 0.0;
    boolean endgame =
        DriverStation.isTeleopEnabled() && isEndgameTime(DriverStation.getMatchTime());
    long currentPieceCount = safePieceCount();
    boolean hasPiece =
        (ctx != null && ctx.repulsor != null && ctx.repulsor.hasPiece()) || currentPieceCount > 0L;
    Decision cycleDecision =
        evaluateCycle(ctx, hubActive, remainingShiftTime, transferAvailable, scoringAvailable);
    currentCycleIntent = cycleDecision.intent();
    StrategyDirective directive =
        directiveFor(ctx, cycleDecision, hubActive ? Math.max(0.0, remainingShiftTime) : 0.0);
    if (ctx != null && ctx.repulsor != null) {
      ctx.repulsor.setStrategyDirective(directive);
    }

    signals.put(TRANSFER_ACTION_AVAILABLE, transferAvailable);
    signals.put(SCORE_ACTION_AVAILABLE, scoringAvailable);
    signals.put(HUB_ACTIVE, hubActive);
    signals.put(HAS_PIECE, hasPiece);
    signals.put(PIECE_COUNT, currentPieceCount);
    signals.put(REMAINING_SHIFT_TIME, remainingShiftTime);
    signals.put(ReasoningKeys.ENDGAME, endgame);
    signals.put(CYCLE_INTENT, cycleDecision.intent().name());
    signals.put(SCORE_OPTION_SCORE, cycleDecision.scoringOption().score());
    signals.put(TRANSFER_OPTION_SCORE, cycleDecision.transferOption().score());
    signals.put(
        ALLIANCE_SIDE_FUEL_UNITS, cycleDecision.scoringOption().resources().resourceUnits());
    signals.put(CENTER_FUEL_UNITS, cycleDecision.transferOption().resources().resourceUnits());

    boolean autoWantsRecovery =
        scoringAvailable
            && hubActive
            && (hasPiece || cycleDecision.intent() == Intent.SCORE_AVAILABLE_RESOURCES);
    boolean autoWantsShuttle =
        transferAvailable
            && (!hubActive || cycleDecision.intent() == Intent.TRANSFER_FOR_LATER_SCORE);
    signals.put(AUTO_WANT_SHUTTLE, autoWantsShuttle);
    signals.put(AUTO_WANT_SHUTTLE_RECOVERY, autoWantsRecovery);

    if (signals.getOr(TESTING, false)) {
      EnumSet<BehaviourFlag> out = EnumSet.of(BehaviourFlag.AUTOPATH_MODE);
      signals.put(SELECTED_MODE, "testing_autopath");
      signals.flush();
      return out;
    }

    EnumSet<BehaviourFlag> out = seq.update(ctx);
    signals.put(SELECTED_MODE, seq.phaseName());
    signals.flush();
    return out;
  }

  private static void addModeTransitions(
      SequenceReasoner.Builder<BehaviourFlag, BehaviourContext> b, int from, String fromName) {
    b.addTransition(
        from,
        fromName + "_to_defense",
        120,
        0.0,
        (ctx, signals) -> wantsDefense(signals),
        PH_DEFENSE);
    b.addTransition(
        from,
        fromName + "_to_autopath",
        110,
        0.0,
        (ctx, signals) -> wantsAutopath(signals),
        PH_AUTOPATH);
    b.addTransition(
        from,
        fromName + "_to_recovery",
        90,
        0.0,
        (ctx, signals) -> wantsShuttleRecovery(signals),
        PH_SHUTTLE_RECOVERY);
    b.addTransition(
        from,
        fromName + "_to_transfer",
        80,
        0.0,
        (ctx, signals) -> wantsShuttle(signals),
        PH_SHUTTLE);
  }

  private static boolean wantsDefense(Signals signals) {
    return signals.getOr(WANT_DEFENSE, false)
        || (signals.getOr(ReasoningKeys.ENDGAME, false) && !signals.getOr(WANT_AUTOPATH, false));
  }

  private static boolean wantsAutopath(Signals signals) {
    return signals.getOr(WANT_AUTOPATH, false)
        || signals.getOr(TESTING, false)
        || (!signals.getOr(TRANSFER_ACTION_AVAILABLE, false)
            && !signals.getOr(SCORE_ACTION_AVAILABLE, false));
  }

  private static boolean wantsShuttle(Signals signals) {
    return signals.getOr(TRANSFER_ACTION_AVAILABLE, false)
        && (signals.getOr(WANT_SHUTTLE, false) || signals.getOr(AUTO_WANT_SHUTTLE, false));
  }

  private static boolean wantsShuttleRecovery(Signals signals) {
    return signals.getOr(SCORE_ACTION_AVAILABLE, false)
        && (signals.getOr(WANT_SHUTTLE_RECOVERY, false)
            || signals.getOr(AUTO_WANT_SHUTTLE_RECOVERY, false));
  }

  private Decision evaluateCycle(
      BehaviourContext ctx,
      boolean hubActive,
      double remainingShiftTime,
      boolean transferAvailable,
      boolean scoringAvailable) {
    if (ctx == null || ctx.repulsor == null || ctx.robotPose == null) {
      return CycleStrategyEvaluator.decide(null, Tuning.defaults());
    }

    Pose2d robotPose = ctx.robotPose.get();
    FieldGeometry geometry = ctx.repulsor.getFieldDefinition().geometry();
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    FieldTrackerCore tracker = FieldTrackerCore.getInstance();

    ResourceRegionSummary scoringSide =
        scoringAvailable
            ? tracker.summarizeCollectResources(
                "alliance_side", robotPose, p -> isAllianceSide(p, alliance, geometry))
            : ResourceRegionSummary.empty("alliance_side");
    ResourceRegionSummary center =
        transferAvailable
            ? tracker.summarizeCollectResources("center", robotPose, p -> isCenterZone(p, geometry))
            : ResourceRegionSummary.empty("center");

    Inputs inputs =
        new Inputs(
            hubActive,
            hubActive ? Math.max(0.0, remainingShiftTime) : 0.0,
            5.2,
            scoringSide,
            center,
            distanceTo(robotPose, scoringSide, allianceSideFallback(alliance, geometry)),
            distanceTo(robotPose, center, geometry.center()),
            centerReturnMeters(center, alliance, geometry),
            currentCycleIntent);
    return CycleStrategyEvaluator.decide(inputs, Tuning.defaults());
  }

  private static StrategyDirective directiveFor(
      BehaviourContext ctx, Decision decision, double deadlineSeconds) {
    if (ctx == null || ctx.repulsor == null || decision == null || decision.bestOption() == null) {
      return StrategyDirective.none();
    }
    var option = decision.bestOption();
    var actionProfile = ctx.repulsor.getFieldDefinition().actionProfile();
    String actionId =
        switch (option.intent()) {
          case TRANSFER_FOR_LATER_SCORE ->
              actionProfile.transferProjectileShot().map(a -> a.id()).orElse("none");
          case SCORE_AVAILABLE_RESOURCES ->
              actionProfile.scoreProjectileShot().map(a -> a.id()).orElse("none");
          case FALLBACK -> "none";
        };
    String actionRole =
        switch (option.intent()) {
          case TRANSFER_FOR_LATER_SCORE -> "TRANSFER_TO_SCORE";
          case SCORE_AVAILABLE_RESOURCES -> "SCORE";
          case FALLBACK -> "none";
        };
    return new StrategyDirective(
        "rebuilt2026.cycle",
        option.intent(),
        option.resources().id(),
        option.resources().nearestResource(),
        actionRole,
        actionId,
        option.expectedUnits(),
        option.cycleSeconds(),
        deadlineSeconds,
        option.score());
  }

  private static boolean isAllianceSide(
      Translation2d point, DriverStation.Alliance alliance, FieldGeometry geometry) {
    if (point == null || geometry == null) return false;
    double boundary = geometry.lengthMeters() * ALLIANCE_ZONE_X_FRACTION;
    if (alliance == DriverStation.Alliance.Red) {
      return point.getX() >= geometry.lengthMeters() - boundary;
    }
    return point.getX() <= boundary;
  }

  private static boolean isCenterZone(Translation2d point, FieldGeometry geometry) {
    if (point == null || geometry == null) return false;
    double halfWidth = geometry.lengthMeters() * CENTER_ZONE_HALF_WIDTH_FRACTION;
    return Math.abs(point.getX() - geometry.lengthMeters() * 0.5) <= halfWidth;
  }

  private static Translation2d allianceSideFallback(
      DriverStation.Alliance alliance, FieldGeometry geometry) {
    double x =
        alliance == DriverStation.Alliance.Red
            ? geometry.lengthMeters() * 0.75
            : geometry.lengthMeters() * 0.25;
    return new Translation2d(x, geometry.widthMeters() * 0.5);
  }

  private static double distanceTo(
      Pose2d robotPose, ResourceRegionSummary summary, Translation2d fallback) {
    Translation2d target =
        summary != null && summary.nearestResource() != null ? summary.nearestResource() : fallback;
    return robotPose == null || target == null
        ? 0.0
        : robotPose.getTranslation().getDistance(target);
  }

  private static double centerReturnMeters(
      ResourceRegionSummary center, DriverStation.Alliance alliance, FieldGeometry geometry) {
    Translation2d source =
        center != null && center.nearestResource() != null
            ? center.nearestResource()
            : geometry.center();
    return source.getDistance(allianceSideFallback(alliance, geometry));
  }

  private static boolean isEndgameTime(double matchTimeSecondsRemaining) {
    return matchTimeSecondsRemaining >= 0.0 && matchTimeSecondsRemaining <= 20.0;
  }

  private long safePieceCount() {
    try {
      Long v = pieceCount.get();
      return v == null ? 0L : Math.max(0L, v);
    } catch (RuntimeException ex) {
      return 0L;
    }
  }

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  @Override
  public void reset() {
    seq.reset();
    currentCycleIntent = Intent.FALLBACK;
    seq.signals().flush();
  }

  /** Runs close in the Repulsor runtime. */
  @Override
  public void close() {
    pieceCount.close();
    nt.close();
  }
}
