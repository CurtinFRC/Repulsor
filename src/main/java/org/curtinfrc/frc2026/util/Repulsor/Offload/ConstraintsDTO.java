package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides constraints dto functionality for the Repulsor offload serialization and native/JNI
 * entrypoint boundary. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class ConstraintsDTO {
  private double minLaunchSpeedMetersPerSecond;
  private double maxLaunchSpeedMetersPerSecond;
  private double minLaunchAngleDeg;
  private double maxLaunchAngleDeg;
  private String shotStyle;

  /** Returns the constraints dto value maintained by this Repulsor component. */
  public ConstraintsDTO() {}

  /**
   * Returns the constraints dto value maintained by this Repulsor component.
   *
   * @param minLaunchSpeedMetersPerSecond distance or field-coordinate value in meters.
   * @param maxLaunchSpeedMetersPerSecond distance or field-coordinate value in meters.
   * @param minLaunchAngleDeg value used by this operation.
   * @param maxLaunchAngleDeg value used by this operation.
   * @param shotStyle value used by this operation.
   */
  public ConstraintsDTO(
      double minLaunchSpeedMetersPerSecond,
      double maxLaunchSpeedMetersPerSecond,
      double minLaunchAngleDeg,
      double maxLaunchAngleDeg,
      String shotStyle) {
    this.minLaunchSpeedMetersPerSecond = minLaunchSpeedMetersPerSecond;
    this.maxLaunchSpeedMetersPerSecond = maxLaunchSpeedMetersPerSecond;
    this.minLaunchAngleDeg = minLaunchAngleDeg;
    this.maxLaunchAngleDeg = maxLaunchAngleDeg;
    this.shotStyle = shotStyle;
  }

  /**
   * Returns the get min launch speed meters per second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getMinLaunchSpeedMetersPerSecond() {
    return minLaunchSpeedMetersPerSecond;
  }

  /**
   * Returns the get max launch speed meters per second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getMaxLaunchSpeedMetersPerSecond() {
    return maxLaunchSpeedMetersPerSecond;
  }

  /**
   * Returns the get min launch angle deg value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getMinLaunchAngleDeg() {
    return minLaunchAngleDeg;
  }

  /**
   * Returns the get max launch angle deg value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getMaxLaunchAngleDeg() {
    return maxLaunchAngleDeg;
  }

  /**
   * Returns the get shot style value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String getShotStyle() {
    return shotStyle;
  }

  /**
   * Updates set min launch speed meters per second state or telemetry as part of the Repulsor
   * runtime loop. This may mutate local state, NetworkTables output, planner caches, or
   * command-side runtime state depending on the owning type.
   *
   * @param minLaunchSpeedMetersPerSecond distance or field-coordinate value in meters.
   */
  public void setMinLaunchSpeedMetersPerSecond(double minLaunchSpeedMetersPerSecond) {
    this.minLaunchSpeedMetersPerSecond = minLaunchSpeedMetersPerSecond;
  }

  /**
   * Updates set max launch speed meters per second state or telemetry as part of the Repulsor
   * runtime loop. This may mutate local state, NetworkTables output, planner caches, or
   * command-side runtime state depending on the owning type.
   *
   * @param maxLaunchSpeedMetersPerSecond distance or field-coordinate value in meters.
   */
  public void setMaxLaunchSpeedMetersPerSecond(double maxLaunchSpeedMetersPerSecond) {
    this.maxLaunchSpeedMetersPerSecond = maxLaunchSpeedMetersPerSecond;
  }

  /**
   * Updates set min launch angle deg state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param minLaunchAngleDeg value used by this operation.
   */
  public void setMinLaunchAngleDeg(double minLaunchAngleDeg) {
    this.minLaunchAngleDeg = minLaunchAngleDeg;
  }

  /**
   * Updates set max launch angle deg state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param maxLaunchAngleDeg value used by this operation.
   */
  public void setMaxLaunchAngleDeg(double maxLaunchAngleDeg) {
    this.maxLaunchAngleDeg = maxLaunchAngleDeg;
  }

  /**
   * Updates set shot style state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param shotStyle value used by this operation.
   */
  public void setShotStyle(String shotStyle) {
    this.shotStyle = shotStyle;
  }
}
