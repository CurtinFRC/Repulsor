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

import java.util.List;
import java.util.Map;

/**
 * Provides game piece physics config functionality for the Repulsor projectile and shot-planning
 * layer. Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public final class GamePiecePhysicsConfig {
  /**
   * Configuration value for name. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public String name;

  /**
   * Configuration value for mass kg. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public double mass_kg;

  /**
   * Configuration value for drag coefficient. The valid range and tuning source are defined by the
   * owning subsystem or field profile.
   */
  public double drag_coefficient;

  /**
   * Configuration value for cross section area m2. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  public double cross_section_area_m2;

  /**
   * Configuration value for air density kg per m3. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public double air_density_kg_per_m3;

  /**
   * Configuration value for metadata. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public Metadata metadata;

  /**
   * Provides metadata functionality for the Repulsor projectile and shot-planning layer. Use this
   * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static final class Metadata {
    /**
     * Configuration value for source mesh. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public String source_mesh;

    /**
     * Configuration value for mesh units. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public String mesh_units;

    /**
     * Configuration value for scale. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public double scale;

    /**
     * Configuration value for volume m3. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public double volume_m3;

    /**
     * Configuration value for surface area m2. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public double surface_area_m2;

    /**
     * Configuration value for bounds extents m. The valid range and tuning source are defined by
     * the owning subsystem or field profile.
     */
    public List<Double> bounds_extents_m;

    /**
     * Configuration value for flight axis. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public String flight_axis;

    /**
     * Configuration value for density kg per m3. The valid range and tuning source are defined by
     * the owning subsystem or field profile.
     */
    public Double density_kg_per_m3;

    /**
     * Configuration value for mass kg override. The valid range and tuning source are defined by
     * the owning subsystem or field profile.
     */
    public Double mass_kg_override;

    /**
     * Configuration value for area method. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public String area_method;

    /**
     * Configuration value for sphericity. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public Double sphericity;

    /**
     * Configuration value for equivalent sphere diameter m. Distances use meters in WPILib field
     * coordinates and should be treated as tunable when sourced from profiles.
     */
    public Double equivalent_sphere_diameter_m;

    /**
     * Configuration value for projected area resolution. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public Integer projected_area_resolution;

    /**
     * Configuration value for projected area rel error. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public Double projected_area_rel_error;

    /**
     * Configuration value for volume method. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public String volume_method;

    /**
     * Configuration value for voxel volume pitch m. The valid range and tuning source are defined
     * by the owning subsystem or field profile.
     */
    public Double voxel_volume_pitch_m;

    /**
     * Configuration value for voxel volume rel error. The valid range and tuning source are defined
     * by the owning subsystem or field profile.
     */
    public Double voxel_volume_rel_error;

    /**
     * Configuration value for auto shape hint. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public String auto_shape_hint;

    /**
     * Configuration value for auto drag coefficient hint. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public Double auto_drag_coefficient_hint;

    /**
     * Configuration value for material. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public String material;

    /**
     * Configuration value for estimated terminal velocity mps. The valid range and tuning source
     * are defined by the owning subsystem or field profile.
     */
    public double estimated_terminal_velocity_mps;

    /**
     * Configuration value for air dynamic viscosity pa s. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public double air_dynamic_viscosity_pa_s;

    /**
     * Configuration value for reynolds number at terminal. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public Double reynolds_number_at_terminal;

    /**
     * Configuration value for drag coefficient at terminal. The valid range and tuning source are
     * defined by the owning subsystem or field profile.
     */
    public Double drag_coefficient_at_terminal;

    /**
     * Configuration value for drag model. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public String drag_model;

    /**
     * Configuration value for alt cross section estimates m2. Time values use seconds and should be
     * tuned against measured robot loop and mechanism latency.
     */
    public Map<String, Double> alt_cross_section_estimates_m2;

    /**
     * Configuration value for warnings. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public List<String> warnings;
  }
}
