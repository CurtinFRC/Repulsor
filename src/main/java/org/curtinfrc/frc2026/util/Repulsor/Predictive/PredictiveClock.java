package org.curtinfrc.frc2026.util.Repulsor.Predictive;

/**
 * Provides predictive clock functionality for the Repulsor predictive field-state and
 * collection-planning layer. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class PredictiveClock {
  private PredictiveClock() {}

  /**
   * Returns the now seconds value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static double nowSeconds() {
    return System.nanoTime() * 1.0e-9;
  }
}
