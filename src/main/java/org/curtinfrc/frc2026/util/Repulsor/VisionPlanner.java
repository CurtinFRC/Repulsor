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

package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.NtRepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.RepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision.Kind;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision.ObstacleType;

/**
 * Provides vision planner functionality for the Repulsor core Repulsor coordination layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public class VisionPlanner {
  /**
   * Provides vision obstacle functionality for the Repulsor core Repulsor coordination layer. Use
   * this type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static class VisionObstacle extends Obstacle {
    /**
     * Configuration value for loc. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public Translation2d loc;

    /**
     * Configuration value for size x. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public double sizeX;

    /**
     * Configuration value for size y. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public double sizeY;

    /**
     * Configuration value for kind. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public Kind kind;

    /**
     * Returns the vision obstacle value maintained by this Repulsor component.
     *
     * @param loc value used by this operation.
     * @param strength value used by this operation.
     * @param type value used by this operation.
     */
    public VisionObstacle(Translation2d loc, double strength, ObstacleType type) {
      super(strength, true);
      this.loc = loc;
      this.sizeX = type.getSize().getFirst();
      this.sizeY = type.getSize().getSecond();
      this.kind = type.getKind();
    }

    /**
     * Returns the get force at position value maintained by this Repulsor component.
     *
     * @param position value used by this operation.
     * @param target value used by this operation.
     * @return value produced by this operation.
     */
    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      double distance = loc.getDistance(position);
      if (distance > 3.0) return new Force();

      var dsBase = RepulsorDriverStation.getInstance();
      double clearanceScale = 1.0;
      if (dsBase instanceof NtRepulsorDriverStation ds) {
        clearanceScale = ds.getConfigDouble("clearance_scale");
      }

      double scaledRadius = Math.max(sizeX, sizeY) * 0.5 * clearanceScale;
      double radial = distance - scaledRadius;

      double mag = distToForceMag(radial);
      Translation2d delta = position.minus(loc);
      if (delta.getNorm() < 1e-9 || Math.abs(mag) < 1e-12) {
        return new Force();
      }

      double angleRad = Math.atan2(delta.getY(), delta.getX());

      return new Force(mag, new edu.wpi.first.math.geometry.Rotation2d(angleRad));
    }

    /**
     * Returns the intersects rectangle value maintained by this Repulsor component.
     *
     * @param rectCorners value used by this operation.
     * @return value produced by this operation.
     */
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;

      double rx = sizeX / 2;
      double ry = sizeY / 2;
      for (Translation2d corner : rectCorners) {
        double dx = corner.getX() - loc.getX();
        double dy = corner.getY() - loc.getY();
        if ((dx * dx) / (rx * rx) + (dy * dy) / (ry * ry) <= 1) {
          return true;
        }
      }

      double boundingRadius = Math.max(rx, ry);
      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        Translation2d b = rectCorners[(i + 1) % rectCorners.length];
        if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < boundingRadius) return true;
      }

      return false;
    }
  }

  private List<RepulsorVision> m_vision = new ArrayList<RepulsorVision>();

  /** Returns the vision planner value maintained by this Repulsor component. */
  public VisionPlanner() {}

  /**
   * Returns the with vision value maintained by this Repulsor component.
   *
   * @param vision value used by this operation.
   * @return vision planner result for with vision.
   */
  public VisionPlanner withVision(RepulsorVision vision) {
    m_vision.add(vision);
    return this;
  }

  /**
   * Runs add vision in the Repulsor runtime.
   *
   * @param vision value used by this operation.
   */
  public void addVision(RepulsorVision vision) {
    m_vision.add(vision);
  }

  /**
   * Returns the get obstacles value maintained by this Repulsor component.
   *
   * @return list of vision obstacle values produced by this operation.
   */
  public List<VisionObstacle> getObstacles() {
    return m_vision.stream()
        .flatMap(v -> Arrays.stream(v.getObstacles()))
        .map(o -> new VisionObstacle(new Translation2d(o.x(), o.y()), 1.5, o.type()))
        .collect(Collectors.toList());
  }

  /** Runs tick in the Repulsor runtime. */
  public void tick() {
    for (RepulsorVision vision : m_vision) {
      vision.tick();
    }
  }
}
