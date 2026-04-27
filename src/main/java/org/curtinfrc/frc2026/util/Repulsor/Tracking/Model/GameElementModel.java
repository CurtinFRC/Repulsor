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

package org.curtinfrc.frc2026.util.Repulsor.Tracking.Model;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Provides game element model functionality for the Repulsor typed model layer for field objects
 * and prediction inputs. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class GameElementModel {
  private PrimitiveObject[] composition = new PrimitiveObject[0];
  private Pose3d position;

  /**
   * Returns the game element model value maintained by this Repulsor component.
   *
   * @param position value used by this operation.
   */
  public GameElementModel(Pose3d position) {
    this.position = position;
  }

  /**
   * Returns the game element model value maintained by this Repulsor component.
   *
   * @param position value used by this operation.
   * @param composition value used by this operation.
   */
  public GameElementModel(Pose3d position, PrimitiveObject[] composition) {
    this.position = position;
    this.composition = composition != null ? composition : new PrimitiveObject[0];
  }

  /**
   * Returns the get position value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Pose3d getPosition() {
    return position;
  }

  /**
   * Updates set position state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param position value used by this operation.
   */
  public void setPosition(Pose3d position) {
    this.position = position;
  }

  /**
   * Returns the get composition value maintained by this Repulsor component.
   *
   * @return primitive object[] result for get composition.
   */
  public PrimitiveObject[] getComposition() {
    return composition;
  }

  /**
   * Updates set composition state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param composition value used by this operation.
   */
  public void setComposition(PrimitiveObject[] composition) {
    this.composition = composition != null ? composition : new PrimitiveObject[0];
  }
}
