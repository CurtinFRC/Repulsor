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
package org.curtinfrc.frc2026.util.Repulsor.State;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/**
 * Provides game state functionality for the Repulsor match-state storage and simulation driver
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public class GameState extends StaticState {
  /**
   * Default teleop game length in seconds. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  public static final double DEFAULT_TELEOP_GAME_LENGTH = 140.0;

  /**
   * Default autonomous period length in seconds. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public static final double DEFAULT_AUTONOMOUS_PERIOD_LENGTH = 20.0;

  /**
   * Default transition period length in seconds. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public static final double DEFAULT_TRANSITION_PERIOD_LENGTH = 10.0;

  /**
   * Default match shift length in seconds. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  public static final double DEFAULT_MATCH_SHIFT_LENGTH = 25.0;

  private final double teleopGameLength;
  private final double autonomousPeriodLength;
  private final double transitionPeriodLength;
  private final double matchShiftLength;

  private Optional<DriverStation.Alliance> alliance = Optional.empty();
  private Optional<DriverStation.Alliance> inactiveFirst = Optional.empty();

  private Alert noAllianceAlert = new Alert("No Alliance Read", AlertType.kWarning);
  private Alert noGameDataAlert = new Alert("No Game Data Read", AlertType.kWarning);

  /** Creates a game state using the default FRC match format timings. */
  public GameState() {
    this(
        DEFAULT_TELEOP_GAME_LENGTH,
        DEFAULT_AUTONOMOUS_PERIOD_LENGTH,
        DEFAULT_TRANSITION_PERIOD_LENGTH,
        DEFAULT_MATCH_SHIFT_LENGTH);
  }

  /**
   * Creates a game state with the given match-format timings.
   *
   * @param teleopGameLength total teleop length in seconds.
   * @param autonomousPeriodLength autonomous period length in seconds.
   * @param transitionPeriodLength transition period length in seconds.
   * @param matchShiftLength length of one hub shift in seconds.
   */
  public GameState(
      double teleopGameLength,
      double autonomousPeriodLength,
      double transitionPeriodLength,
      double matchShiftLength) {
    if (!(teleopGameLength > 0.0)
        || !(autonomousPeriodLength > 0.0)
        || !(transitionPeriodLength > 0.0)
        || !(matchShiftLength > 0.0)) {
      throw new IllegalArgumentException("match format lengths must be positive");
    }
    this.teleopGameLength = teleopGameLength;
    this.autonomousPeriodLength = autonomousPeriodLength;
    this.transitionPeriodLength = transitionPeriodLength;
    this.matchShiftLength = matchShiftLength;
  }

  private void updateAlliance() {
    Optional<DriverStation.Alliance> readAlliance = DriverStation.getAlliance();
    noAllianceAlert.set(readAlliance.isEmpty());
    if (alliance.isEmpty() && readAlliance.isPresent()) {
      alliance = readAlliance;
    }
  }

  private void updateGameData() {
    String gameData = DriverStation.getGameSpecificMessage();
    noGameDataAlert.set(true);
    if (gameData.length() > 0) {
      if (gameData.charAt(0) == 'B' || gameData.charAt(0) == 'R') {
        noGameDataAlert.set(false);
      }
    }

    if (inactiveFirst.isEmpty() && !noGameDataAlert.get()) {
      inactiveFirst =
          Optional.of(
              (gameData.charAt(0) == 'B')
                  ? DriverStation.Alliance.Blue
                  : DriverStation.Alliance.Red);
    }
  }

  private double getMatchTime() {
    double gameTime = DriverStation.getMatchTime();
    if (DriverStation.isFMSAttached()) {
      gameTime = teleopGameLength - gameTime;
    }
    return gameTime;
  }

  private int getGamePeriodNumber() {
    double gameTime = getMatchTime();

    int gamePeriodNumber;
    if (!DriverStation.isTeleopEnabled()) {
      gamePeriodNumber = -1;
    } else {
      if (gameTime <= transitionPeriodLength) {
        gamePeriodNumber = 0;
      } else {
        gamePeriodNumber = (int) Math.ceil((gameTime - transitionPeriodLength) / matchShiftLength);
      }
    }

    return gamePeriodNumber;
  }

  /**
   * Returns the is hub active value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isHubActive() {
    if (!(inactiveFirst.isPresent() && alliance.isPresent())) {
      return false;
    }

    int shiftDiscriminant = (inactiveFirst.get() == alliance.get()) ? 1 : 0;
    int gamePeriodNumber = getGamePeriodNumber();
    boolean isActive = false;
    if (gamePeriodNumber == 0) {
      isActive = true;
    } else {
      isActive = !(gamePeriodNumber % 2 == shiftDiscriminant);
    }
    return isActive;
  }

  /**
   * Returns the is hub inactive value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isHubInactive() {
    return !isHubActive();
  }

  /**
   * Returns the get remaining shift time value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getRemainingShiftTime() {
    double shiftEndTime;
    double gameTime = getMatchTime();
    double shiftNumber = getGamePeriodNumber();
    if (shiftNumber == -1) {
      shiftEndTime = autonomousPeriodLength;
    } else if (shiftNumber == 0) {
      shiftEndTime = transitionPeriodLength;
    } else {
      shiftEndTime = shiftNumber * matchShiftLength + transitionPeriodLength;
      shiftEndTime = (shiftNumber < 5) ? shiftEndTime : shiftEndTime + 5;
    }

    return shiftEndTime - gameTime;
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param dt value used by this operation.
   */
  @Override
  public void update(double dt) {
    updateGameData();
    updateAlliance();
  }
}
