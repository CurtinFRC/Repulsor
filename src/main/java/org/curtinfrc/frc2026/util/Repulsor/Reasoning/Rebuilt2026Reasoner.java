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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.EnumSet;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourContext;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourFlag;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;
import org.curtinfrc.frc2026.util.Repulsor.State.GameState;
import org.curtinfrc.frc2026.util.Repulsor.State.StateManager;

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
  private static final SignalKey<Boolean> HUB_ACTIVE = ReasoningKeys.boolKey("hub_active");
  private static final SignalKey<Boolean> HAS_PIECE = ReasoningKeys.boolKey("has_piece");
  private static final SignalKey<Long> PIECE_COUNT = ReasoningKeys.longKey("piece_count");
  private static final SignalKey<Double> REMAINING_SHIFT_TIME =
      ReasoningKeys.doubleKey("remaining_shift_time");
  private static final SignalKey<String> SELECTED_MODE = ReasoningKeys.stringKey("selected_mode");

  private static final int PH_SHUTTLE = 0;
  private static final int PH_SHUTTLE_RECOVERY = 1;
  private static final int PH_AUTOPATH = 2;
  private static final int PH_DEFENSE = 3;

  private final NetworkTablesSignals nt;
  private final NetworkTablesValue<Long> pieceCount;
  private final SequenceReasoner<BehaviourFlag, BehaviourContext> seq;

  public Rebuilt2026Reasoner() {
    this(NetworkTableInstance.getDefault(), "/Repulsor/Reasoning");
  }

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
    nts.register(HUB_ACTIVE, false);
    nts.register(HAS_PIECE, false);
    nts.register(PIECE_COUNT, 0L);
    nts.register(REMAINING_SHIFT_TIME, 0.0);
    nts.register(SELECTED_MODE, "");
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

  public Signals signals() {
    return seq.signals();
  }

  public void setWantDefense(boolean v) {
    seq.signals().put(WANT_DEFENSE, v);
    seq.signals().flush();
  }

  public void setWantAutopath(boolean v) {
    seq.signals().put(WANT_AUTOPATH, v);
    seq.signals().flush();
  }

  public void setWantShuttle(boolean v) {
    seq.signals().put(WANT_SHUTTLE, v);
    seq.signals().flush();
  }

  public void setWantShuttleRecovery(boolean v) {
    seq.signals().put(WANT_SHUTTLE_RECOVERY, v);
    seq.signals().flush();
  }

  public void setTesting(boolean v) {
    seq.signals().put(TESTING, v);
    seq.signals().flush();
  }

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

    GameState gameState = StateManager.getState(GameState.class);
    boolean hubActive = gameState != null && gameState.isHubActive();
    double remainingShiftTime = gameState != null ? gameState.getRemainingShiftTime() : 0.0;
    boolean endgame =
        DriverStation.isTeleopEnabled() && isEndgameTime(DriverStation.getMatchTime());
    long currentPieceCount = safePieceCount();
    boolean hasPiece =
        (ctx != null && ctx.repulsor != null && ctx.repulsor.hasPiece()) || currentPieceCount > 0L;

    signals.put(TRANSFER_ACTION_AVAILABLE, transferAvailable);
    signals.put(HUB_ACTIVE, hubActive);
    signals.put(HAS_PIECE, hasPiece);
    signals.put(PIECE_COUNT, currentPieceCount);
    signals.put(REMAINING_SHIFT_TIME, remainingShiftTime);
    signals.put(ReasoningKeys.ENDGAME, endgame);

    boolean autoWantsRecovery = transferAvailable && hubActive;
    boolean autoWantsShuttle = transferAvailable && !hubActive;
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
        || !signals.getOr(TRANSFER_ACTION_AVAILABLE, false);
  }

  private static boolean wantsShuttle(Signals signals) {
    return signals.getOr(TRANSFER_ACTION_AVAILABLE, false)
        && (signals.getOr(WANT_SHUTTLE, false) || signals.getOr(AUTO_WANT_SHUTTLE, false));
  }

  private static boolean wantsShuttleRecovery(Signals signals) {
    return signals.getOr(TRANSFER_ACTION_AVAILABLE, false)
        && (signals.getOr(WANT_SHUTTLE_RECOVERY, false)
            || signals.getOr(AUTO_WANT_SHUTTLE_RECOVERY, false));
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

  @Override
  public void reset() {
    seq.reset();
    seq.signals().flush();
  }

  @Override
  public void close() {
    pieceCount.close();
    nt.close();
  }
}
