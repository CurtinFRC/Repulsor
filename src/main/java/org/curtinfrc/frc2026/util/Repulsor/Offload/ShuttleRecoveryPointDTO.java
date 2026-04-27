package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides shuttle recovery point dto functionality for the Repulsor offload serialization and
 * native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class ShuttleRecoveryPointDTO {
  private boolean found;
  private double x;
  private double y;
  private double yawDeg;
  private double score;

  /** Returns the shuttle recovery point dto value maintained by this Repulsor component. */
  public ShuttleRecoveryPointDTO() {}

  /**
   * Returns the not found value maintained by this Repulsor component.
   *
   * @return shuttle recovery point dto result for not found.
   */
  public static ShuttleRecoveryPointDTO notFound() {
    return new ShuttleRecoveryPointDTO();
  }

  /**
   * Returns the of value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @param y distance or field-coordinate value in meters.
   * @param yawDeg value used by this operation.
   * @param score value used by this operation.
   * @return shuttle recovery point dto result for of.
   */
  public static ShuttleRecoveryPointDTO of(double x, double y, double yawDeg, double score) {
    ShuttleRecoveryPointDTO dto = new ShuttleRecoveryPointDTO();
    dto.found = true;
    dto.x = x;
    dto.y = y;
    dto.yawDeg = yawDeg;
    dto.score = score;
    return dto;
  }

  /**
   * Returns the is found value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isFound() {
    return found;
  }

  /**
   * Updates set found state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param found value used by this operation.
   */
  public void setFound(boolean found) {
    this.found = found;
  }

  /**
   * Returns the get x value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getX() {
    return x;
  }

  /**
   * Updates set x state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param x distance or field-coordinate value in meters.
   */
  public void setX(double x) {
    this.x = x;
  }

  /**
   * Returns the get y value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getY() {
    return y;
  }

  /**
   * Updates set y state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param y distance or field-coordinate value in meters.
   */
  public void setY(double y) {
    this.y = y;
  }

  /**
   * Returns the get yaw deg value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getYawDeg() {
    return yawDeg;
  }

  /**
   * Updates set yaw deg state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param yawDeg value used by this operation.
   */
  public void setYawDeg(double yawDeg) {
    this.yawDeg = yawDeg;
  }

  /**
   * Returns the get score value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getScore() {
    return score;
  }

  /**
   * Updates set score state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param score value used by this operation.
   */
  public void setScore(double score) {
    this.score = score;
  }
}
