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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides point candidate functionality for the Repulsor typed model layer for field objects and
 * prediction inputs. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class PointCandidate {
  /**
   * Configuration value for point. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Translation2d point;

  /**
   * Configuration value for our eta s. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double ourEtaS;

  /**
   * Configuration value for value. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double value;

  /**
   * Configuration value for enemy pressure. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public final double enemyPressure;

  /**
   * Configuration value for ally congestion. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public final double allyCongestion;

  /**
   * Configuration value for enemy intent. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public final double enemyIntent;

  /**
   * Configuration value for ally intent. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public final double allyIntent;

  /**
   * Configuration value for score. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double score;

  /**
   * Configuration value for rotation. Angles use WPILib rotation conventions; names ending in
   * degrees are degrees, otherwise radians are assumed by the API.
   */
  public final Rotation2d rotation;

  /**
   * Returns the point candidate value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @param rot value used by this operation.
   * @param ourEtaS value used by this operation.
   * @param value value used by this operation.
   * @param enemyPressure value used by this operation.
   * @param allyCongestion value used by this operation.
   * @param enemyIntent value used by this operation.
   * @param allyIntent value used by this operation.
   * @param score value used by this operation.
   */
  public PointCandidate(
      Translation2d point,
      Rotation2d rot,
      double ourEtaS,
      double value,
      double enemyPressure,
      double allyCongestion,
      double enemyIntent,
      double allyIntent,
      double score) {
    this.point = point;
    this.ourEtaS = ourEtaS;
    this.value = value;
    this.enemyPressure = enemyPressure;
    this.allyCongestion = allyCongestion;
    this.enemyIntent = enemyIntent;
    this.allyIntent = allyIntent;
    this.score = score;
    this.rotation = rot;
  }
}
