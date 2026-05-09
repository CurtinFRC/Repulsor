package org.curtinfrc.frc2026.util.Repulsor.Offload;

/** Version numbers for Repulsor offload DTO contracts. */
public final class RepulsorOffloadContract {
  /** Increment when any Repulsor offload DTO shape changes incompatibly. */
  public static final int CONTRACT_VERSION = 1;

  /** Matches the FieldPlanner calculate @Offloadable task version. */
  public static final int FIELD_PLANNER_CALCULATE_VERSION = 3;

  private RepulsorOffloadContract() {}

  public static boolean isCompatible(int contractVersion) {
    return contractVersion == CONTRACT_VERSION;
  }
}
