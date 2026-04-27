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
 * Provides game object functionality for the Repulsor typed model layer for field objects and
 * prediction inputs. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class GameObject {
  private final String id;
  private final String type;
  private final Pose3d position;

  /**
   * Returns the game object value maintained by this Repulsor component.
   *
   * @param id value used by this operation.
   * @param type value used by this operation.
   */
  public GameObject(String id, String type) {
    this(id, type, null);
  }

  /**
   * Returns the game object value maintained by this Repulsor component.
   *
   * @param id value used by this operation.
   * @param type value used by this operation.
   * @param position value used by this operation.
   */
  public GameObject(String id, String type, Pose3d position) {
    if (id == null || id.isEmpty()) throw new IllegalArgumentException("id cannot be null/empty");
    if (type == null || type.isEmpty())
      throw new IllegalArgumentException("type cannot be null/empty");
    this.id = id;
    this.type = type;
    this.position = position;
  }

  /**
   * Returns the get id value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String getId() {
    return id;
  }

  /**
   * Returns the get type value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String getType() {
    return type;
  }

  /**
   * Returns the get position value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Pose3d getPosition() {
    return position;
  }
}
