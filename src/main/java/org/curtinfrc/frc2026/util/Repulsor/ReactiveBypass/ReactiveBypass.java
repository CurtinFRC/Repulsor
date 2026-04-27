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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassConfig;
import org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassRuntime;

/**
 * Provides reactive bypass functionality for the Repulsor reactive bypass planner for local
 * obstacle avoidance. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class ReactiveBypass {
  /**
   * Provides config functionality for the Repulsor reactive bypass planner for local obstacle
   * avoidance. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static class Config extends ReactiveBypassConfig {}

  private final Config cfg = new Config();
  private final ReactiveBypassRuntime runtime = new ReactiveBypassRuntime(cfg);

  /**
   * Updates set config state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param c value used by this operation.
   */
  public void setConfig(Consumer<Config> c) {
    c.accept(cfg);
  }

  /**
   * Runs enable logging in the Repulsor runtime.
   *
   * @param filePath value used by this operation.
   */
  public void enableLogging(String filePath) {
    runtime.enableLogging(filePath);
  }

  /** Runs disable logging in the Repulsor runtime. */
  public void disableLogging() {
    runtime.disableLogging();
  }

  /**
   * Runs finalize episode in the Repulsor runtime.
   *
   * @param success value used by this operation.
   */
  public void finalizeEpisode(boolean success) {
    runtime.finalizeEpisode(success);
  }

  /**
   * Updates reset episode metrics state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   */
  public void resetEpisodeMetrics() {
    runtime.resetEpisodeMetrics();
  }

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  public void reset() {
    runtime.reset();
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @param goal value used by this operation.
   * @param headingTowardGoal value used by this operation.
   * @param dtSeconds time value in seconds.
   * @param robotX distance or field-coordinate value in meters.
   * @param robotY distance or field-coordinate value in meters.
   * @param dynamicObstacles obstacle set used for safety checks, costs, or replanning.
   * @param intersectsDynamicOnly distance or field-coordinate value in meters.
   * @param canRejoinOriginal value used by this operation.
   * @return value produced by this operation.
   */
  public Optional<Pose2d> update(
      Pose2d pose,
      Pose2d goal,
      Rotation2d headingTowardGoal,
      double dtSeconds,
      double robotX,
      double robotY,
      List<? extends Obstacle> dynamicObstacles,
      Function<Translation2d[], Boolean> intersectsDynamicOnly,
      Function<String, Boolean> canRejoinOriginal) {
    return runtime.update(
        pose,
        goal,
        headingTowardGoal,
        dtSeconds,
        robotX,
        robotY,
        dynamicObstacles,
        intersectsDynamicOnly,
        canRejoinOriginal);
  }

  /**
   * Returns the is pinned mode value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isPinnedMode() {
    return runtime.isPinnedMode();
  }
}
