package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides obstacle dto functionality for the Repulsor offload serialization and native/JNI
 * entrypoint boundary. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class ObstacleDTO {
  private String kind;
  private double strength;
  private boolean positive;
  private double x;
  private double y;
  private double sizeX;
  private double sizeY;
  private double radius;
  private double horizonWeight;
  private int horizonStep;

  /** Returns the obstacle dto value maintained by this Repulsor component. */
  public ObstacleDTO() {}

  /**
   * Returns the get kind value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public String getKind() {
    return kind;
  }

  /**
   * Updates set kind state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param kind value used by this operation.
   */
  public void setKind(String kind) {
    this.kind = kind;
  }

  /**
   * Returns the get strength value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getStrength() {
    return strength;
  }

  /**
   * Updates set strength state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param strength value used by this operation.
   */
  public void setStrength(double strength) {
    this.strength = strength;
  }

  /**
   * Returns the is positive value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isPositive() {
    return positive;
  }

  /**
   * Updates set positive state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param positive value used by this operation.
   */
  public void setPositive(boolean positive) {
    this.positive = positive;
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
   * Returns the get size x value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getSizeX() {
    return sizeX;
  }

  /**
   * Updates set size x state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param sizeX distance or field-coordinate value in meters.
   */
  public void setSizeX(double sizeX) {
    this.sizeX = sizeX;
  }

  /**
   * Returns the get size y value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getSizeY() {
    return sizeY;
  }

  /**
   * Updates set size y state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param sizeY distance or field-coordinate value in meters.
   */
  public void setSizeY(double sizeY) {
    this.sizeY = sizeY;
  }

  /**
   * Returns the get radius value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getRadius() {
    return radius;
  }

  /**
   * Updates set radius state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param radius value used by this operation.
   */
  public void setRadius(double radius) {
    this.radius = radius;
  }

  public double getHorizonWeight() {
    return horizonWeight;
  }

  public void setHorizonWeight(double horizonWeight) {
    this.horizonWeight = horizonWeight;
  }

  public int getHorizonStep() {
    return horizonStep;
  }

  public void setHorizonStep(int horizonStep) {
    this.horizonStep = horizonStep;
  }
}
