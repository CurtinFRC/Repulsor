package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides game piece physics dto functionality for the Repulsor offload serialization and
 * native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class GamePiecePhysicsDTO {
  private double massKg;
  private double crossSectionAreaM2;
  private double dragCoefficient;
  private double airDensityKgPerM3;

  /** Returns the game piece physics dto value maintained by this Repulsor component. */
  public GamePiecePhysicsDTO() {}

  /**
   * Returns the game piece physics dto value maintained by this Repulsor component.
   *
   * @param massKg value used by this operation.
   * @param crossSectionAreaM2 value used by this operation.
   * @param dragCoefficient value used by this operation.
   * @param airDensityKgPerM3 value used by this operation.
   */
  public GamePiecePhysicsDTO(
      double massKg, double crossSectionAreaM2, double dragCoefficient, double airDensityKgPerM3) {
    this.massKg = massKg;
    this.crossSectionAreaM2 = crossSectionAreaM2;
    this.dragCoefficient = dragCoefficient;
    this.airDensityKgPerM3 = airDensityKgPerM3;
  }

  /**
   * Returns the get mass kg value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getMassKg() {
    return massKg;
  }

  /**
   * Returns the get cross section area m2 value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getCrossSectionAreaM2() {
    return crossSectionAreaM2;
  }

  /**
   * Returns the get drag coefficient value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getDragCoefficient() {
    return dragCoefficient;
  }

  /**
   * Returns the get air density kg per m3 value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getAirDensityKgPerM3() {
    return airDensityKgPerM3;
  }

  /**
   * Updates set mass kg state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param massKg value used by this operation.
   */
  public void setMassKg(double massKg) {
    this.massKg = massKg;
  }

  /**
   * Updates set cross section area m2 state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param crossSectionAreaM2 value used by this operation.
   */
  public void setCrossSectionAreaM2(double crossSectionAreaM2) {
    this.crossSectionAreaM2 = crossSectionAreaM2;
  }

  /**
   * Updates set drag coefficient state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param dragCoefficient value used by this operation.
   */
  public void setDragCoefficient(double dragCoefficient) {
    this.dragCoefficient = dragCoefficient;
  }

  /**
   * Updates set air density kg per m3 state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param airDensityKgPerM3 value used by this operation.
   */
  public void setAirDensityKgPerM3(double airDensityKgPerM3) {
    this.airDensityKgPerM3 = airDensityKgPerM3;
  }
}
