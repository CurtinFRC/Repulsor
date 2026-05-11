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
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.NtRepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.RepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PredictedDynamicObstacleEnvelope;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision.Kind;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision.ObstacleType;

/**
 * Provides vision planner functionality for the Repulsor core Repulsor coordination layer. Use this
 * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public class VisionPlanner {
  private static final double DEFAULT_PREDICTION_STRENGTH = 1.5;
  private static final double DEFAULT_PREDICTION_HORIZON_WEIGHT = 0.70;
  private static final double DEFAULT_PREDICTION_UNCERTAINTY_METERS = 0.15;
  private static final double DEFAULT_MAX_ASSOCIATION_METERS = 2.0;

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
  private List<RepulsorVision.Obstacle> previousDetections = List.of();
  private List<Obstacle> cachedObstacles = List.of();
  private boolean cacheInitialized = false;

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
  public List<Obstacle> getObstacles() {
    if (!cacheInitialized) {
      refreshObstacleCache();
    }
    return cachedObstacles;
  }

  /** Runs tick in the Repulsor runtime. */
  public void tick() {
    for (RepulsorVision vision : m_vision) {
      vision.tick();
    }
    refreshObstacleCache();
  }

  private void refreshObstacleCache() {
    List<RepulsorVision.Obstacle> currentDetections = currentDetections();
    ArrayList<Obstacle> output = new ArrayList<>();
    for (RepulsorVision.Obstacle detection : currentDetections) {
      output.add(toVisionObstacle(detection));
    }
    if (predictionEnabled()) {
      output.addAll(predictedEnvelopes(currentDetections, previousDetections));
    }
    previousDetections = List.copyOf(currentDetections);
    cachedObstacles = List.copyOf(output);
    cacheInitialized = true;
  }

  private List<RepulsorVision.Obstacle> currentDetections() {
    ArrayList<RepulsorVision.Obstacle> detections = new ArrayList<>();
    for (RepulsorVision vision : m_vision) {
      detections.addAll(Arrays.asList(vision.getObstacles()));
    }
    return detections;
  }

  private VisionObstacle toVisionObstacle(RepulsorVision.Obstacle detection) {
    return new VisionObstacle(
        new Translation2d(detection.x(), detection.y()),
        DEFAULT_PREDICTION_STRENGTH,
        detection.type());
  }

  private List<PredictedDynamicObstacleEnvelope> predictedEnvelopes(
      List<RepulsorVision.Obstacle> currentDetections,
      List<RepulsorVision.Obstacle> previousDetections) {
    if (currentDetections.isEmpty() || previousDetections.isEmpty()) return List.of();
    boolean[] usedPrevious = new boolean[previousDetections.size()];
    ArrayList<PredictedDynamicObstacleEnvelope> predictions = new ArrayList<>();
    for (RepulsorVision.Obstacle current : currentDetections) {
      int previousIdx = nearestMatchingPrevious(current, previousDetections, usedPrevious);
      if (previousIdx < 0) continue;
      usedPrevious[previousIdx] = true;
      RepulsorVision.Obstacle previous = previousDetections.get(previousIdx);
      Translation2d displacement =
          new Translation2d(current.x() - previous.x(), current.y() - previous.y());
      if (displacement.getNorm() <= 1e-6) continue;
      ObstacleType type = current.type();
      double uncertainty = predictionUncertaintyMeters();
      double weight = predictionHorizonWeight();
      predictions.add(
          new PredictedDynamicObstacleEnvelope(
              new Translation2d(
                  current.x() + displacement.getX(), current.y() + displacement.getY()),
              type.getSize().getFirst() * 0.5 + uncertainty,
              type.getSize().getSecond() * 0.5 + uncertainty,
              DEFAULT_PREDICTION_STRENGTH,
              weight,
              1));
    }
    return predictions;
  }

  private int nearestMatchingPrevious(
      RepulsorVision.Obstacle current,
      List<RepulsorVision.Obstacle> previousDetections,
      boolean[] usedPrevious) {
    int best = -1;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < previousDetections.size(); i++) {
      if (usedPrevious[i]) continue;
      RepulsorVision.Obstacle previous = previousDetections.get(i);
      if (!sameType(current.type(), previous.type())) continue;
      double distance = Math.hypot(current.x() - previous.x(), current.y() - previous.y());
      if (distance < bestDistance && distance <= maxAssociationMeters()) {
        bestDistance = distance;
        best = i;
      }
    }
    return best;
  }

  private boolean sameType(ObstacleType a, ObstacleType b) {
    if (a == null || b == null) return false;
    return a.getKind() == b.getKind()
        && Math.abs(a.getSize().getFirst() - b.getSize().getFirst()) <= 1e-9
        && Math.abs(a.getSize().getSecond() - b.getSize().getSecond()) <= 1e-9;
  }

  private static boolean predictionEnabled() {
    return Boolean.parseBoolean(System.getProperty("repulsor.vision.prediction.enabled", "true"));
  }

  private static double predictionHorizonWeight() {
    return doubleProperty(
        "repulsor.vision.prediction.horizonWeight", DEFAULT_PREDICTION_HORIZON_WEIGHT);
  }

  private static double predictionUncertaintyMeters() {
    return doubleProperty(
        "repulsor.vision.prediction.uncertaintyMeters", DEFAULT_PREDICTION_UNCERTAINTY_METERS);
  }

  private static double maxAssociationMeters() {
    return doubleProperty(
        "repulsor.vision.prediction.maxAssociationMeters", DEFAULT_MAX_ASSOCIATION_METERS);
  }

  private static double doubleProperty(String key, double fallback) {
    try {
      return Double.parseDouble(System.getProperty(key, Double.toString(fallback)));
    } catch (NumberFormatException ex) {
      return fallback;
    }
  }
}
