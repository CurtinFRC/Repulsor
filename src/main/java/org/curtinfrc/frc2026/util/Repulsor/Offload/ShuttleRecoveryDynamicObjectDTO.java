package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides shuttle recovery dynamic object dto functionality for the Repulsor offload serialization
 * and native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class ShuttleRecoveryDynamicObjectDTO {
  private String id = "";
  private String type = "unknown";
  private double x;
  private double y;
  private double vx;
  private double vy;
  private double ageS;

  /**
   * Returns the shuttle recovery dynamic object dto value maintained by this Repulsor component.
   */
  public ShuttleRecoveryDynamicObjectDTO() {}

  /**
   * Returns the get id value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String getId() {
    return id;
  }

  /**
   * Updates set id state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param id value used by this operation.
   */
  public void setId(String id) {
    this.id = id == null ? "" : id;
  }

  /**
   * Returns the get type value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String getType() {
    return type;
  }

  /**
   * Updates set type state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param type value used by this operation.
   */
  public void setType(String type) {
    this.type = type == null ? "unknown" : type;
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
   * Returns the get vx value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getVx() {
    return vx;
  }

  /**
   * Updates set vx state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param vx distance or field-coordinate value in meters.
   */
  public void setVx(double vx) {
    this.vx = vx;
  }

  /**
   * Returns the get vy value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getVy() {
    return vy;
  }

  /**
   * Updates set vy state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param vy distance or field-coordinate value in meters.
   */
  public void setVy(double vy) {
    this.vy = vy;
  }

  /**
   * Returns the get age s value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getAgeS() {
    return ageS;
  }

  /**
   * Updates set age s state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param ageS value used by this operation.
   */
  public void setAgeS(double ageS) {
    this.ageS = ageS;
  }
}
