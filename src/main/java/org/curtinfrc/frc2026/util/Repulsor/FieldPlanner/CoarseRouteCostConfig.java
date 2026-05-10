package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

/** Weighted terms used by the coarse global fallback planner's A* edge cost. */
public record CoarseRouteCostConfig(
    double distanceWeight,
    double obstacleClearanceWeight,
    double wallClearanceWeight,
    double turnWeight) {
  private static final double DEFAULT_DISTANCE_WEIGHT = 1.0;
  private static final double DEFAULT_OBSTACLE_CLEARANCE_WEIGHT = 0.0;
  private static final double DEFAULT_WALL_CLEARANCE_WEIGHT = 0.0;
  private static final double DEFAULT_TURN_WEIGHT = 0.05;

  public CoarseRouteCostConfig {
    distanceWeight = Math.max(0.0, distanceWeight);
    obstacleClearanceWeight = Math.max(0.0, obstacleClearanceWeight);
    wallClearanceWeight = Math.max(0.0, wallClearanceWeight);
    turnWeight = Math.max(0.0, turnWeight);
  }

  public static CoarseRouteCostConfig defaults() {
    return new CoarseRouteCostConfig(
        DEFAULT_DISTANCE_WEIGHT,
        DEFAULT_OBSTACLE_CLEARANCE_WEIGHT,
        DEFAULT_WALL_CLEARANCE_WEIGHT,
        DEFAULT_TURN_WEIGHT);
  }
}
