package org.curtinfrc.frc2026.util.Repulsor;

/**
 * Provides static instance functionality for the Repulsor core Repulsor coordination layer. Use
 * this type from robot code, field profiles, or tests when integrating the corresponding Repulsor
 * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
 */
public class StaticInstance {
  /**
   * Configuration value for repulsor. The valid range and tuning source are defined by the owning
   * subsystem or field profile.
   */
  public static volatile Repulsor repulsor;

  /**
   * Returns the get instance value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static Repulsor getInstance() {
    Repulsor local = repulsor;
    if (local == null) {
      synchronized (Repulsor.class) {
        local = repulsor;
        if (local == null) {
          throw new IllegalStateException(
              "Repulsor instance has not been initialized. Please initialize it before calling getInstance().");
        }
      }
    }
    return local;
  }

  /**
   * Runs initialize in the Repulsor runtime.
   *
   * @param repulsorInstance value used by this operation.
   */
  public static void initialize(Repulsor repulsorInstance) {
    synchronized (Repulsor.class) {
      if (repulsor == null) {
        repulsor = repulsorInstance;
      } else {
        throw new IllegalStateException("Repulsor instance has already been initialized.");
      }
    }
  }
}
