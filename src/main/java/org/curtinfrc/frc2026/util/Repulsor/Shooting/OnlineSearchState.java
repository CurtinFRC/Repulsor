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

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

/**
 * Provides online search state functionality for the Repulsor projectile and shot-planning layer.
 * Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public final class OnlineSearchState {
  private Translation2d seed;
  private double stepMeters;
  private long lastUpdateNs;
  private ShotSolution lastSolution;
  private int lastRobotXmm;
  private int lastRobotYmm;
  private int lastTargetXmm;
  private int lastTargetYmm;
  private int lastObsHash;
  private long lastSolutionNs;

  /**
   * Returns the online search state value maintained by this Repulsor component.
   *
   * @param seed value used by this operation.
   * @param stepMeters distance or field-coordinate value in meters.
   */
  public OnlineSearchState(Translation2d seed, double stepMeters) {
    this.seed = seed;
    this.stepMeters = stepMeters;
    this.lastUpdateNs = System.nanoTime();
    Profiler.ensureInit();
  }

  /**
   * Returns the seed value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Translation2d seed() {
    return seed;
  }

  /**
   * Runs seed in the Repulsor runtime.
   *
   * @param seed value used by this operation.
   */
  public void seed(Translation2d seed) {
    this.seed = seed;
  }

  /**
   * Returns the step meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double stepMeters() {
    return stepMeters;
  }

  /**
   * Runs step meters in the Repulsor runtime.
   *
   * @param stepMeters distance or field-coordinate value in meters.
   */
  public void stepMeters(double stepMeters) {
    this.stepMeters = stepMeters;
  }

  /**
   * Returns the last update ns value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public long lastUpdateNs() {
    return lastUpdateNs;
  }

  /** Updates touch state or telemetry as part of the Repulsor runtime loop. */
  public void touch() {
    this.lastUpdateNs = System.nanoTime();
  }

  /**
   * Returns the last solution value maintained by this Repulsor component.
   *
   * @return shot solution result for last solution.
   */
  ShotSolution lastSolution() {
    return lastSolution;
  }

  /**
   * Returns the last robot xmm value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  int lastRobotXmm() {
    return lastRobotXmm;
  }

  /**
   * Returns the last robot ymm value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  int lastRobotYmm() {
    return lastRobotYmm;
  }

  /**
   * Returns the last target xmm value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  int lastTargetXmm() {
    return lastTargetXmm;
  }

  /**
   * Returns the last target ymm value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  int lastTargetYmm() {
    return lastTargetYmm;
  }

  /**
   * Returns the last obs hash value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  int lastObsHash() {
    return lastObsHash;
  }

  /**
   * Returns the last solution ns value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  long lastSolutionNs() {
    return lastSolutionNs;
  }

  /**
   * Updates set last solution state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param solution value used by this operation.
   * @param robotXmm value used by this operation.
   * @param robotYmm value used by this operation.
   * @param targetXmm value used by this operation.
   * @param targetYmm value used by this operation.
   * @param obsHash value used by this operation.
   */
  void setLastSolution(
      ShotSolution solution,
      int robotXmm,
      int robotYmm,
      int targetXmm,
      int targetYmm,
      int obsHash) {
    this.lastSolution = solution;
    this.lastRobotXmm = robotXmm;
    this.lastRobotYmm = robotYmm;
    this.lastTargetXmm = targetXmm;
    this.lastTargetYmm = targetYmm;
    this.lastObsHash = obsHash;
    this.lastSolutionNs = System.nanoTime();
  }
}
