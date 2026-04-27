package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides shot solution dto functionality for the Repulsor offload serialization and native/JNI
 * entrypoint boundary. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class ShotSolutionDTO {
  private Translation2dDTO shooterPosition;
  private double shooterYawRadians;
  private double launchSpeedMetersPerSecond;
  private double launchAngleRadians;
  private double timeToPlaneSeconds;
  private Translation2dDTO impactFieldPosition;
  private double verticalErrorMeters;

  /** Returns the shot solution dto value maintained by this Repulsor component. */
  public ShotSolutionDTO() {}

  /**
   * Returns the get shooter position value maintained by this Repulsor component.
   *
   * @return translation2d dto result for get shooter position.
   */
  public Translation2dDTO getShooterPosition() {
    return shooterPosition;
  }

  /**
   * Returns the get shooter yaw radians value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getShooterYawRadians() {
    return shooterYawRadians;
  }

  /**
   * Returns the get launch speed meters per second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getLaunchSpeedMetersPerSecond() {
    return launchSpeedMetersPerSecond;
  }

  /**
   * Returns the get launch angle radians value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getLaunchAngleRadians() {
    return launchAngleRadians;
  }

  /**
   * Returns the get time to plane seconds value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getTimeToPlaneSeconds() {
    return timeToPlaneSeconds;
  }

  /**
   * Returns the get impact field position value maintained by this Repulsor component.
   *
   * @return translation2d dto result for get impact field position.
   */
  public Translation2dDTO getImpactFieldPosition() {
    return impactFieldPosition;
  }

  /**
   * Returns the get vertical error meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getVerticalErrorMeters() {
    return verticalErrorMeters;
  }

  /**
   * Updates set shooter position state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param shooterPosition value used by this operation.
   */
  public void setShooterPosition(Translation2dDTO shooterPosition) {
    this.shooterPosition = shooterPosition;
  }

  /**
   * Updates set shooter yaw radians state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param shooterYawRadians value used by this operation.
   */
  public void setShooterYawRadians(double shooterYawRadians) {
    this.shooterYawRadians = shooterYawRadians;
  }

  /**
   * Updates set launch speed meters per second state or telemetry as part of the Repulsor runtime
   * loop. This may mutate local state, NetworkTables output, planner caches, or command-side
   * runtime state depending on the owning type.
   *
   * @param launchSpeedMetersPerSecond distance or field-coordinate value in meters.
   */
  public void setLaunchSpeedMetersPerSecond(double launchSpeedMetersPerSecond) {
    this.launchSpeedMetersPerSecond = launchSpeedMetersPerSecond;
  }

  /**
   * Updates set launch angle radians state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param launchAngleRadians value used by this operation.
   */
  public void setLaunchAngleRadians(double launchAngleRadians) {
    this.launchAngleRadians = launchAngleRadians;
  }

  /**
   * Updates set time to plane seconds state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param timeToPlaneSeconds time value in seconds.
   */
  public void setTimeToPlaneSeconds(double timeToPlaneSeconds) {
    this.timeToPlaneSeconds = timeToPlaneSeconds;
  }

  /**
   * Updates set impact field position state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param impactFieldPosition value used by this operation.
   */
  public void setImpactFieldPosition(Translation2dDTO impactFieldPosition) {
    this.impactFieldPosition = impactFieldPosition;
  }

  /**
   * Updates set vertical error meters state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param verticalErrorMeters distance or field-coordinate value in meters.
   */
  public void setVerticalErrorMeters(double verticalErrorMeters) {
    this.verticalErrorMeters = verticalErrorMeters;
  }
}
