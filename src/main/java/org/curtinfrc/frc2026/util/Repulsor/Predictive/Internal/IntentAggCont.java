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
 * Provides intent agg cont functionality for the Repulsor package-internal data structures used by
 * the surrounding Repulsor subsystem. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class IntentAggCont {
  /**
   * Configuration value for regions. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Translation2d[] regions;

  /**
   * Configuration value for intent mass. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public final double[] intentMass;

  /**
   * Configuration value for count. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final int count;

  private final double sigma;

  /**
   * Returns the intent agg cont value maintained by this Repulsor component.
   *
   * @param regions value used by this operation.
   * @param intentMass value used by this operation.
   * @param count value used by this operation.
   * @param sigma value used by this operation.
   */
  public IntentAggCont(Translation2d[] regions, double[] intentMass, int count, double sigma) {
    this.regions = regions != null ? regions : new Translation2d[0];
    this.intentMass = intentMass != null ? intentMass : new double[this.regions.length];
    this.count = count;
    this.sigma = Math.max(1e-6, sigma);
  }

  /**
   * Returns the intent at value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return value produced by this operation.
   */
  public double intentAt(Translation2d p) {
    if (p == null || regions.length == 0) return 0.0;
    double sum = 0.0;
    double s2 = sigma * sigma;
    for (int i = 0; i < regions.length; i++) {
      Translation2d c = regions[i];
      if (c == null) continue;
      double d = c.getDistance(p);
      double k = Math.exp(-0.5 * (d * d) / Math.max(1e-6, s2));
      sum += intentMass[i] * k;
    }
    return sum;
  }
}
