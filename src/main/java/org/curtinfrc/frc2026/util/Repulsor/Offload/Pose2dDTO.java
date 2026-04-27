package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides pose2d dto functionality for the Repulsor offload serialization and native/JNI
 * entrypoint boundary. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class Pose2dDTO {
  private double x;
  private double y;
  private double thetaRadians;

  /** Returns the pose2d dto value maintained by this Repulsor component. */
  public Pose2dDTO() {}

  /**
   * Returns the pose2d dto value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @param y distance or field-coordinate value in meters.
   * @param thetaRadians value used by this operation.
   */
  public Pose2dDTO(double x, double y, double thetaRadians) {
    this.x = x;
    this.y = y;
    this.thetaRadians = thetaRadians;
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
   * Returns the get y value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getY() {
    return y;
  }

  /**
   * Returns the get theta radians value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getThetaRadians() {
    return thetaRadians;
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
   * Updates set theta radians state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param thetaRadians value used by this operation.
   */
  public void setThetaRadians(double thetaRadians) {
    this.thetaRadians = thetaRadians;
  }
}
