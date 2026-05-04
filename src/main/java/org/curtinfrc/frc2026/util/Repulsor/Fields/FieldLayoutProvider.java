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

package org.curtinfrc.frc2026.util.Repulsor.Fields;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointConfig;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;

/**
 * Contract for field layout provider implementations used by the Repulsor field/profile definition
 * layer used to tune Repulsor for a specific game. Use this type from robot code, field profiles,
 * or tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public interface FieldLayoutProvider {
  /**
   * Builds the WPILib command sequence for the current behaviour context.
   *
   * @param ft value used by this operation.
   * @return game element[] result for build.
   */
  GameElement[] build(FieldTrackerCore ft);

  /**
   * Returns the game name value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  String gameName();

  /**
   * Returns the game year value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  int gameYear();

  /**
   * Returns the april tag layout value maintained by this Repulsor component.
   *
   * @return april tag field layout result for april tag layout.
   */
  default AprilTagFieldLayout aprilTagLayout() {
    return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  /**
   * Returns the field length meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  default double fieldLengthMeters() {
    return aprilTagLayout().getFieldLength();
  }

  /**
   * Returns the field width meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  default double fieldWidthMeters() {
    return aprilTagLayout().getFieldWidth();
  }

  /**
   * Returns the geometry value maintained by this Repulsor component.
   *
   * @return field geometry result for geometry.
   */
  default FieldGeometry geometry() {
    return new FieldGeometry(fieldLengthMeters(), fieldWidthMeters());
  }

  /**
   * Returns the field model value maintained by this Repulsor component.
   *
   * @return field model result for field model.
   */
  default FieldModel fieldModel() {
    return new FieldModel(geometry(), aprilTagLayout());
  }

  default FieldPlannerWaypointConfig waypointConfig() {
    return FieldPlannerWaypointConfig.defaults();
  }

  /**
   * Updates configure tracker state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param ft value used by this operation.
   */
  default void configureTracker(FieldTrackerCore ft) {}
}
