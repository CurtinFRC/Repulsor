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

package org.curtinfrc.frc2026.util.Repulsor.Predictive.Model;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

/**
 * Provides candidate functionality for the Repulsor typed model layer for field objects and
 * prediction inputs. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class Candidate {
  /**
   * Configuration value for setpoint. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final RepulsorSetpoint setpoint;

  /**
   * Configuration value for target xy. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Translation2d targetXY;

  /**
   * Configuration value for our eta s. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double ourEtaS;

  /**
   * Configuration value for enemy eta s. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public final double enemyEtaS;

  /**
   * Configuration value for ally eta s. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double allyEtaS;

  /**
   * Configuration value for congestion. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double congestion;

  /**
   * Configuration value for pressure. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double pressure;

  /**
   * Configuration value for score. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double score;

  /**
   * Returns the candidate value maintained by this Repulsor component.
   *
   * @param sp value used by this operation.
   * @param xy distance or field-coordinate value in meters.
   * @param ourEtaS value used by this operation.
   * @param enemyEtaS value used by this operation.
   * @param allyEtaS value used by this operation.
   * @param congestion value used by this operation.
   * @param pressure value used by this operation.
   * @param score value used by this operation.
   */
  public Candidate(
      RepulsorSetpoint sp,
      Translation2d xy,
      double ourEtaS,
      double enemyEtaS,
      double allyEtaS,
      double congestion,
      double pressure,
      double score) {
    this.setpoint = sp;
    this.targetXY = xy;
    this.ourEtaS = ourEtaS;
    this.enemyEtaS = enemyEtaS;
    this.allyEtaS = allyEtaS;
    this.congestion = congestion;
    this.pressure = pressure;
    this.score = score;
  }
}
