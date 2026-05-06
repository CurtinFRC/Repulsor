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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateRuntime;

/**
 * Provides field tracker collect objective runtime functionality for the Repulsor collection
 * objective runtime for tracked field resources. Use this type from robot code, field profiles, or
 * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public final class FieldTrackerCollectObjectiveRuntime {
  private final FieldTrackerCollectObjectiveLoop loop;

  /**
   * Creates a field tracker collect objective runtime instance with the dependencies and tuning
   * values used by this Repulsor component.
   *
   * @param predictor value used by this operation.
   * @param collectObjectivePoints value used by this operation.
   * @param dynamicsSupplier value used by this operation.
   * @param collectTypePredicate value used by this operation.
   */
  FieldTrackerCollectObjectiveRuntime(
      PredictiveFieldStateRuntime predictor,
      Supplier<Translation2d[]> collectObjectivePoints,
      Supplier<List<DynamicObject>> dynamicsSupplier,
      Predicate<String> collectTypePredicate) {
    this.loop =
        new FieldTrackerCollectObjectiveLoop(
            predictor, collectObjectivePoints, dynamicsSupplier, collectTypePredicate);
  }

  /**
   * Updates reset all state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   */
  void resetAll() {
    loop.resetAll();
  }

  void configureCollectPlannerTuning(CollectPlannerTuning tuning) {
    loop.configureCollectPlannerTuning(tuning);
  }

  CollectPlannerTuning collectPlannerTuning() {
    return loop.collectPlannerTuning();
  }

  /**
   * Returns the next objective goal blue value maintained by this Repulsor component.
   *
   * @param robotPoseBlue value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param goalUnits value used by this operation.
   * @param cat value used by this operation.
   * @return value produced by this operation.
   */
  Pose2d nextObjectiveGoalBlue(
      Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits, CategorySpec cat) {
    return loop.nextObjectiveGoalBlue(robotPoseBlue, ourSpeedCap, goalUnits, cat);
  }

  /**
   * Updates clear state state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   */
  void clearState() {
    loop.clearState();
  }
}
