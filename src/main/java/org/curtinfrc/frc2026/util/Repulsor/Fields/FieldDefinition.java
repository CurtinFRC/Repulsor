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

import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.AutoPathRuntimeConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap.HeatmapProvider;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

/**
 * Contract for field definition implementations used by the Repulsor field/profile definition layer
 * used to tune Repulsor for a specific game. Use this type from robot code, field profiles, or
 * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public interface FieldDefinition
    extends FieldLayoutProvider, FieldPlanner.ObstacleProvider, HeatmapProvider {
  /**
   * Returns the default collect setpoint value maintained by this Repulsor component.
   *
   * @return optional repulsor setpoint produced by this operation.
   */
  default Optional<RepulsorSetpoint> defaultCollectSetpoint() {
    return Optional.empty();
  }

  /**
   * Returns the default score setpoint value maintained by this Repulsor component.
   *
   * @return optional repulsor setpoint produced by this operation.
   */
  default Optional<RepulsorSetpoint> defaultScoreSetpoint() {
    return Optional.empty();
  }

  /**
   * Returns the action profile value maintained by this Repulsor component.
   *
   * @return field action profile result for action profile.
   */
  default FieldActionProfile actionProfile() {
    return FieldActionProfile.none();
  }

  default AutoPathRuntimeConfig autoPathRuntimeConfig() {
    return AutoPathRuntimeConfig.defaults();
  }

  /**
   * Returns the validate profile value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  default List<String> validateProfile() {
    return FieldProfileValidator.validate(this);
  }
}
