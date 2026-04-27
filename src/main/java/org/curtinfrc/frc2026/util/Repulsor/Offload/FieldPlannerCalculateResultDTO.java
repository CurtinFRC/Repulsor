package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides field planner calculate result dto functionality for the Repulsor offload serialization
 * and native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class FieldPlannerCalculateResultDTO {
  private double goalX;
  private double goalY;
  private double vxMetersPerSecond;
  private double vyMetersPerSecond;
  private double omegaRadians;
  private boolean hasErrMeters;
  private double errMeters;
  private double activeGoalX;
  private double activeGoalY;
  private double activeGoalThetaRadians;

  /**
   * Returns the get goal x value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getGoalX() {
    return goalX;
  }

  /**
   * Updates set goal x state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param goalX distance or field-coordinate value in meters.
   */
  public void setGoalX(double goalX) {
    this.goalX = goalX;
  }

  /**
   * Returns the get goal y value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getGoalY() {
    return goalY;
  }

  /**
   * Updates set goal y state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param goalY distance or field-coordinate value in meters.
   */
  public void setGoalY(double goalY) {
    this.goalY = goalY;
  }

  /**
   * Returns the get vx meters per second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getVxMetersPerSecond() {
    return vxMetersPerSecond;
  }

  /**
   * Updates set vx meters per second state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param vxMetersPerSecond distance or field-coordinate value in meters.
   */
  public void setVxMetersPerSecond(double vxMetersPerSecond) {
    this.vxMetersPerSecond = vxMetersPerSecond;
  }

  /**
   * Returns the get vy meters per second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getVyMetersPerSecond() {
    return vyMetersPerSecond;
  }

  /**
   * Updates set vy meters per second state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param vyMetersPerSecond distance or field-coordinate value in meters.
   */
  public void setVyMetersPerSecond(double vyMetersPerSecond) {
    this.vyMetersPerSecond = vyMetersPerSecond;
  }

  /**
   * Returns the get omega radians value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getOmegaRadians() {
    return omegaRadians;
  }

  /**
   * Updates set omega radians state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param omegaRadians value used by this operation.
   */
  public void setOmegaRadians(double omegaRadians) {
    this.omegaRadians = omegaRadians;
  }

  /**
   * Returns the is has err meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isHasErrMeters() {
    return hasErrMeters;
  }

  /**
   * Updates set has err meters state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param hasErrMeters distance or field-coordinate value in meters.
   */
  public void setHasErrMeters(boolean hasErrMeters) {
    this.hasErrMeters = hasErrMeters;
  }

  /**
   * Returns the get err meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getErrMeters() {
    return errMeters;
  }

  /**
   * Updates set err meters state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param errMeters distance or field-coordinate value in meters.
   */
  public void setErrMeters(double errMeters) {
    this.errMeters = errMeters;
  }

  /**
   * Returns the get active goal x value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getActiveGoalX() {
    return activeGoalX;
  }

  /**
   * Updates set active goal x state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param activeGoalX distance or field-coordinate value in meters.
   */
  public void setActiveGoalX(double activeGoalX) {
    this.activeGoalX = activeGoalX;
  }

  /**
   * Returns the get active goal y value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getActiveGoalY() {
    return activeGoalY;
  }

  /**
   * Updates set active goal y state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param activeGoalY distance or field-coordinate value in meters.
   */
  public void setActiveGoalY(double activeGoalY) {
    this.activeGoalY = activeGoalY;
  }

  /**
   * Returns the get active goal theta radians value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getActiveGoalThetaRadians() {
    return activeGoalThetaRadians;
  }

  /**
   * Updates set active goal theta radians state or telemetry as part of the Repulsor runtime loop.
   * This may mutate local state, NetworkTables output, planner caches, or command-side runtime
   * state depending on the owning type.
   *
   * @param activeGoalThetaRadians value used by this operation.
   */
  public void setActiveGoalThetaRadians(double activeGoalThetaRadians) {
    this.activeGoalThetaRadians = activeGoalThetaRadians;
  }
}
