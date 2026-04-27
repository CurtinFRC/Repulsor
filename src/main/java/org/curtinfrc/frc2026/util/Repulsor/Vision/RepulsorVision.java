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

package org.curtinfrc.frc2026.util.Repulsor.Vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;

// ROBOT VISION

/**
 * Contract for repulsor vision implementations used by the Repulsor vision integration and
 * simulation layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public interface RepulsorVision {
  /**
   * Provides obstacle type functionality for the Repulsor vision integration and simulation layer.
   * Use this type from robot code, field profiles, or tests when integrating the corresponding
   * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
   * motion.
   */
  public static class ObstacleType {
    private Pair<Double, Double> size;
    private Kind kind;

    /**
     * Returns the obstacle type value maintained by this Repulsor component.
     *
     * @param size_x distance or field-coordinate value in meters.
     * @param size_y distance or field-coordinate value in meters.
     * @param kind value used by this operation.
     */
    public ObstacleType(double size_x, double size_y, Kind kind) {
      size = new Pair<Double, Double>(size_x, size_y);
      this.kind = kind;
    }

    /**
     * Returns the get size value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public Pair<Double, Double> getSize() {
      return size;
    }

    /**
     * Returns the get kind value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public Kind getKind() {
      return kind;
    }
  }

  /**
   * Defines the kind values used by the Repulsor vision integration and simulation layer. Use this
   * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static enum Kind {
    kRobotRed,
    kRobotBlue,
    kGameElement,
    kUnknown
  }

  /**
   * Provides obstacle functionality for the Repulsor vision integration and simulation layer. Use
   * this type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static class Obstacle {
    private double m_x;
    private double m_y;
    private ObstacleType m_type;

    /**
     * Returns the obstacle value maintained by this Repulsor component.
     *
     * @param x distance or field-coordinate value in meters.
     * @param y distance or field-coordinate value in meters.
     * @param type value used by this operation.
     */
    public Obstacle(double x, double y, ObstacleType type) {
      m_x = x;
      m_y = y;
      m_type = type;
    }

    /**
     * Returns the obstacle value maintained by this Repulsor component.
     *
     * @param pose WPILib Pose2d in field-relative coordinates.
     * @param type value used by this operation.
     */
    public Obstacle(Pose2d pose, ObstacleType type) {
      m_x = pose.getX();
      m_y = pose.getY();
      m_type = type;
    }

    /**
     * Returns x for the current Repulsor state.
     *
     * @return value produced by this operation.
     */
    public double x() {
      return m_x;
    }

    /**
     * Returns y for the current Repulsor state.
     *
     * @return value produced by this operation.
     */
    public double y() {
      return m_y;
    }

    /**
     * Returns the type value maintained by this Repulsor component.
     *
     * @return obstacle type result for type.
     */
    public ObstacleType type() {
      return m_type;
    }
  }

  /**
   * Returns the get obstacles value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public default Obstacle[] getObstacles() {
    return new Obstacle[0];
  }

  /** Runs tick in the Repulsor runtime. */
  public default void tick() {}
}
