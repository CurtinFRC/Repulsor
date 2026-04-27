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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides collect eval functionality for the Repulsor package-internal data structures used by the
 * surrounding Repulsor subsystem. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class CollectEval {
  /**
   * Configuration value for p. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public Translation2d p;

  /**
   * Configuration value for eta. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double eta;

  /**
   * Configuration value for units. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double units;

  /**
   * Configuration value for count. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public int count;

  /**
   * Configuration value for evidence. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double evidence;

  /**
   * Configuration value for value. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double value;

  /**
   * Configuration value for enemy pressure. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double enemyPressure;

  /**
   * Configuration value for ally congestion. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double allyCongestion;

  /**
   * Configuration value for enemy intent. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double enemyIntent;

  /**
   * Configuration value for ally intent. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double allyIntent;

  /**
   * Configuration value for local avoid. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double localAvoid;

  /**
   * Configuration value for activity. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double activity;

  /**
   * Configuration value for depleted. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double depleted;

  /**
   * Configuration value for overlap. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double overlap;

  /**
   * Configuration value for core count. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public int coreCount;

  /**
   * Configuration value for core dist. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double coreDist;

  /**
   * Configuration value for robust penalty. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double robustPenalty;

  /**
   * Configuration value for score. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double score;

  /**
   * Configuration value for region units. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double regionUnits;

  /**
   * Configuration value for bandit bonus. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double banditBonus;
}
