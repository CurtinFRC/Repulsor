package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides translation2d dto functionality for the Repulsor offload serialization and native/JNI
 * entrypoint boundary. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class Translation2dDTO {
  private double x;
  private double y;

  /** Returns the translation2d dto value maintained by this Repulsor component. */
  public Translation2dDTO() {}

  /**
   * Returns the translation2d dto value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @param y distance or field-coordinate value in meters.
   */
  public Translation2dDTO(double x, double y) {
    this.x = x;
    this.y = y;
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
}
