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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Objects;

/**
 * Provides intake footprint functionality for the Repulsor core Repulsor coordination layer. Use
 * this type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public final class IntakeFootprint {
  private static IntakeFootprint instance = null;

  /**
   * Returns the get footprint value maintained by this Repulsor component.
   *
   * @return intake footprint result for get footprint.
   */
  public static IntakeFootprint getFootprint() {
    if (instance == null) {
      throw new IllegalStateException(
          "IntakeFootprint instance not initialized. Call one of the factory methods first.");
    }
    return instance;
  }

  /**
   * Updates set footprint state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param footprint value used by this operation.
   */
  public static void setFootprint(IntakeFootprint footprint) {
    if (instance != null) {
      throw new IllegalStateException("IntakeFootprint instance already set.");
    }
    instance = Objects.requireNonNull(footprint);
  }

  /**
   * Returns the robot square value maintained by this Repulsor component.
   *
   * @param robotSideMeters distance or field-coordinate value in meters.
   * @return intake footprint result for robot square.
   */
  public static IntakeFootprint robotSquare(double robotSideMeters) {
    double h = 0.5 * robotSideMeters;
    return new IntakeFootprint(new Rect(new Translation2d(0.0, 0.0), h, h));
  }

  /**
   * Returns the robot rect value maintained by this Repulsor component.
   *
   * @param robotLengthMeters distance or field-coordinate value in meters.
   * @param robotWidthMeters distance or field-coordinate value in meters.
   * @return intake footprint result for robot rect.
   */
  public static IntakeFootprint robotRect(double robotLengthMeters, double robotWidthMeters) {
    return new IntakeFootprint(
        new Rect(new Translation2d(0.0, 0.0), 0.5 * robotLengthMeters, 0.5 * robotWidthMeters));
  }

  /**
   * Returns the front rect value maintained by this Repulsor component.
   *
   * @param robotLengthMeters distance or field-coordinate value in meters.
   * @param intakeDepthMeters distance or field-coordinate value in meters.
   * @param intakeWidthMeters distance or field-coordinate value in meters.
   * @return intake footprint result for front rect.
   */
  public static IntakeFootprint frontRect(
      double robotLengthMeters, double intakeDepthMeters, double intakeWidthMeters) {
    double hx = 0.5 * intakeDepthMeters;
    double hy = 0.5 * intakeWidthMeters;
    double cx = 0.5 * robotLengthMeters + hx;
    return new IntakeFootprint(new Rect(new Translation2d(cx, 0.0), hx, hy));
  }

  private final Shape shape;

  private IntakeFootprint(Shape shape) {
    this.shape = Objects.requireNonNull(shape);
  }

  /**
   * Returns the contains point robot frame value maintained by this Repulsor component.
   *
   * @param pRobot value used by this operation.
   * @return value produced by this operation.
   */
  public boolean containsPointRobotFrame(Translation2d pRobot) {
    return shape.contains(pRobot);
  }

  /**
   * Returns the support point robot frame value maintained by this Repulsor component.
   *
   * @param dirRobot value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d supportPointRobotFrame(Translation2d dirRobot) {
    return shape.support(dirRobot);
  }

  /**
   * Returns the snap center so footprint touches point value maintained by this Repulsor component.
   *
   * @param desiredCenterField value used by this operation.
   * @param robotHeading value used by this operation.
   * @param pointField value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d snapCenterSoFootprintTouchesPoint(
      Translation2d desiredCenterField, Rotation2d robotHeading, Translation2d pointField) {

    Translation2d dirField = pointField.minus(desiredCenterField);
    Translation2d dirRobot = dirField.rotateBy(robotHeading.unaryMinus());

    double n2 = dirRobot.getX() * dirRobot.getX() + dirRobot.getY() * dirRobot.getY();
    if (n2 < 1e-12) {
      dirRobot = new Translation2d(1.0, 0.0);
    }

    Translation2d contactRobot = shape.support(dirRobot);
    Translation2d contactField = contactRobot.rotateBy(robotHeading);

    return pointField.minus(contactField);
  }

  /**
   * Returns the snap center so point is inside footprint value maintained by this Repulsor
   * component.
   *
   * @param desiredCenterField value used by this operation.
   * @param robotHeading value used by this operation.
   * @param pointField value used by this operation.
   * @return value produced by this operation.
   */
  public Translation2d snapCenterSoPointIsInsideFootprint(
      Translation2d desiredCenterField, Rotation2d robotHeading, Translation2d pointField) {

    Translation2d pRobot = pointField.minus(desiredCenterField).rotateBy(robotHeading.unaryMinus());
    Translation2d clampedRobot = shape.closestPointInside(pRobot);
    Translation2d deltaRobot = pRobot.minus(clampedRobot);

    return desiredCenterField.plus(deltaRobot.rotateBy(robotHeading));
  }

  private interface Shape {
    boolean contains(Translation2d p);

    /**
     * Returns the support value maintained by this Repulsor component.
     *
     * @param dir value used by this operation.
     * @return value produced by this operation.
     */
    Translation2d support(Translation2d dir);

    /**
     * Returns the closest point inside value maintained by this Repulsor component.
     *
     * @param p value used by this operation.
     * @return value produced by this operation.
     */
    Translation2d closestPointInside(Translation2d p);
  }

  private static final class Rect implements Shape {
    private final Translation2d c;
    private final double hx;
    private final double hy;

    Rect(Translation2d center, double halfX, double halfY) {
      this.c = Objects.requireNonNull(center);
      this.hx = Math.max(0.0, halfX);
      this.hy = Math.max(0.0, halfY);
    }

    /**
     * Returns the contains value maintained by this Repulsor component.
     *
     * @param p value used by this operation.
     * @return value produced by this operation.
     */
    @Override
    public boolean contains(Translation2d p) {
      double dx = p.getX() - c.getX();
      double dy = p.getY() - c.getY();
      return Math.abs(dx) <= hx + 1e-9 && Math.abs(dy) <= hy + 1e-9;
    }

    /**
     * Returns the support value maintained by this Repulsor component.
     *
     * @param dir value used by this operation.
     * @return value produced by this operation.
     */
    @Override
    public Translation2d support(Translation2d dir) {
      double sx = dir.getX() >= 0.0 ? hx : -hx;
      double sy = dir.getY() >= 0.0 ? hy : -hy;
      return new Translation2d(c.getX() + sx, c.getY() + sy);
    }

    /**
     * Returns the closest point inside value maintained by this Repulsor component.
     *
     * @param p value used by this operation.
     * @return value produced by this operation.
     */
    @Override
    public Translation2d closestPointInside(Translation2d p) {
      double x = clamp(p.getX(), c.getX() - hx, c.getX() + hx);
      double y = clamp(p.getY(), c.getY() - hy, c.getY() + hy);
      return new Translation2d(x, y);
    }

    private static double clamp(double v, double lo, double hi) {
      return Math.max(lo, Math.min(hi, v));
    }
  }
}
