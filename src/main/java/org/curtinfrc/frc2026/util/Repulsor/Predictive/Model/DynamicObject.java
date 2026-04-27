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

/**
 * Provides dynamic object functionality for the Repulsor typed model layer for field objects and
 * prediction inputs. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class DynamicObject {
  /**
   * Configuration value for id. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final String id;

  /**
   * Configuration value for type. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final String type;

  /**
   * Configuration value for pos. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Translation2d pos;

  /**
   * Configuration value for vel. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final Translation2d vel;

  /**
   * Configuration value for age s. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public final double ageS;

  /**
   * Returns the dynamic object value maintained by this Repulsor component.
   *
   * @param id value used by this operation.
   * @param type value used by this operation.
   * @param pos value used by this operation.
   * @param vel value used by this operation.
   * @param ageS value used by this operation.
   */
  public DynamicObject(String id, String type, Translation2d pos, Translation2d vel, double ageS) {
    this.id = id;
    this.type = type != null ? type : "unknown";
    this.pos = pos != null ? pos : new Translation2d();
    this.vel = vel != null ? vel : new Translation2d();
    this.ageS = Math.max(0.0, ageS);
  }
}
