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

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides reactive bypass sample functionality for the Repulsor runtime helper layer shared by
 * behaviours and planners. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
final class ReactiveBypassSample {
  /**
   * Configuration value for pos. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final Translation2d pos;

  /**
   * Configuration value for s para. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final double sPara;

  /**
   * Configuration value for s perp. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final double sPerp;

  /**
   * Configuration value for sign para. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final int signPara;

  /**
   * Configuration value for sign perp. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final int signPerp;

  /**
   * Configuration value for dt. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  final double dt;

  /**
   * Creates a reactive bypass sample instance with the dependencies and tuning values used by this
   * Repulsor component.
   *
   * @param pos value used by this operation.
   * @param sPara value used by this operation.
   * @param sPerp value used by this operation.
   * @param signPara value used by this operation.
   * @param signPerp value used by this operation.
   * @param dt value used by this operation.
   */
  ReactiveBypassSample(
      Translation2d pos, double sPara, double sPerp, int signPara, int signPerp, double dt) {
    this.pos = pos;
    this.sPara = sPara;
    this.sPerp = sPerp;
    this.signPara = signPara;
    this.signPerp = signPerp;
    this.dt = dt;
  }
}
