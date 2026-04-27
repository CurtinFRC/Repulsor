package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides drag shot auto response dto functionality for the Repulsor offload serialization and
 * native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class DragShotAutoResponseDTO {
  private boolean present;
  private ShotSolutionDTO solution;

  /** Returns the drag shot auto response dto value maintained by this Repulsor component. */
  public DragShotAutoResponseDTO() {}

  /**
   * Returns the empty value maintained by this Repulsor component.
   *
   * @return drag shot auto response dto result for empty.
   */
  public static DragShotAutoResponseDTO empty() {
    DragShotAutoResponseDTO response = new DragShotAutoResponseDTO();
    response.setPresent(false);
    return response;
  }

  /**
   * Returns the is present value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isPresent() {
    return present;
  }

  /**
   * Updates set present state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param present value used by this operation.
   */
  public void setPresent(boolean present) {
    this.present = present;
  }

  /**
   * Returns the get solution value maintained by this Repulsor component.
   *
   * @return shot solution dto result for get solution.
   */
  public ShotSolutionDTO getSolution() {
    return solution;
  }

  /**
   * Updates set solution state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param solution value used by this operation.
   */
  public void setSolution(ShotSolutionDTO solution) {
    this.solution = solution;
  }
}
