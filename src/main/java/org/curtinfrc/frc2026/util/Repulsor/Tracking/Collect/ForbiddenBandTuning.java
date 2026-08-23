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

package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

/**
 * Season-specific forbidden-band geometry consumed by the generic collect pass setup. Values are
 * populated from the active field profile so the runtime carries no game-specific constants.
 *
 * @param forbidMarginMeters extra clearance added outside each band half width
 * @param trenchSquareCenterXMeters blue-side x center of the square resource region
 * @param bumpRectCenterOffsetMeters offset from field mid-length to the rectangular bump region
 *     center
 * @param bandHalfWidthMeters half width of each forbidden band before margin is applied
 */
public record ForbiddenBandTuning(
    double forbidMarginMeters,
    double trenchSquareCenterXMeters,
    double bumpRectCenterOffsetMeters,
    double bandHalfWidthMeters) {

  /**
   * Returns carrier defaults matching the legacy hardcoded 2026 trench/bump band values.
   *
   * @return forbidden band tuning equal to the previous inline constants
   */
  public static ForbiddenBandTuning defaults() {
    return new ForbiddenBandTuning(0.6, 4.625594, 3.63982, 1.1938 * 0.5);
  }

  /**
   * Returns total band half width including the forbid margin.
   *
   * @return effective half width used for band containment checks
   */
  public double effectiveHalfWidthMeters() {
    return bandHalfWidthMeters + forbidMarginMeters;
  }
}
