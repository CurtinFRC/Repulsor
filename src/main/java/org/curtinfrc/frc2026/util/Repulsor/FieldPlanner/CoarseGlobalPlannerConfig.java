package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

/** Tunable limits for the coarse global fallback planner. */
public record CoarseGlobalPlannerConfig(
    double cellMeters,
    double waypointLookaheadMeters,
    int maxExpandedNodes,
    double maxRuntimeSeconds,
    double clearanceBufferMeters,
    CoarseRouteCostConfig routeCostConfig) {
  private static final double DEFAULT_CELL_M = 0.55;
  private static final double DEFAULT_WAYPOINT_LOOKAHEAD_M = 1.4;
  private static final int DEFAULT_MAX_EXPANDED_NODES = 1200;
  private static final double DEFAULT_MAX_RUNTIME_SECONDS = 0.010;
  private static final double DEFAULT_CLEARANCE_BUFFER_METERS = 0.0;

  public CoarseGlobalPlannerConfig(
      double cellMeters,
      double waypointLookaheadMeters,
      int maxExpandedNodes,
      double maxRuntimeSeconds) {
    this(
        cellMeters,
        waypointLookaheadMeters,
        maxExpandedNodes,
        maxRuntimeSeconds,
        0.0,
        CoarseRouteCostConfig.defaults());
  }

  public CoarseGlobalPlannerConfig(
      double cellMeters,
      double waypointLookaheadMeters,
      int maxExpandedNodes,
      double maxRuntimeSeconds,
      double clearanceBufferMeters) {
    this(
        cellMeters,
        waypointLookaheadMeters,
        maxExpandedNodes,
        maxRuntimeSeconds,
        clearanceBufferMeters,
        CoarseRouteCostConfig.defaults());
  }

  public CoarseGlobalPlannerConfig(
      double cellMeters,
      double waypointLookaheadMeters,
      int maxExpandedNodes,
      double maxRuntimeSeconds,
      double clearanceBufferMeters,
      double turnCostWeight) {
    this(
        cellMeters,
        waypointLookaheadMeters,
        maxExpandedNodes,
        maxRuntimeSeconds,
        clearanceBufferMeters,
        new CoarseRouteCostConfig(1.0, 0.0, 0.0, turnCostWeight));
  }

  public CoarseGlobalPlannerConfig {
    cellMeters = Math.max(0.20, cellMeters);
    waypointLookaheadMeters = Math.max(cellMeters, waypointLookaheadMeters);
    maxExpandedNodes = Math.max(1, maxExpandedNodes);
    maxRuntimeSeconds = Math.max(0.0005, maxRuntimeSeconds);
    clearanceBufferMeters = Math.max(0.0, clearanceBufferMeters);
    routeCostConfig = routeCostConfig == null ? CoarseRouteCostConfig.defaults() : routeCostConfig;
  }

  public double turnCostWeight() {
    return routeCostConfig.turnWeight();
  }

  public static CoarseGlobalPlannerConfig defaults() {
    return new CoarseGlobalPlannerConfig(
        doubleProperty("repulsor.fieldplanner.globalFallback.cellMeters", DEFAULT_CELL_M),
        doubleProperty(
            "repulsor.fieldplanner.globalFallback.lookaheadMeters", DEFAULT_WAYPOINT_LOOKAHEAD_M),
        intProperty(
            "repulsor.fieldplanner.globalFallback.maxExpandedNodes", DEFAULT_MAX_EXPANDED_NODES),
        doubleProperty(
            "repulsor.fieldplanner.globalFallback.maxRuntimeSeconds", DEFAULT_MAX_RUNTIME_SECONDS),
        doubleProperty(
            "repulsor.fieldplanner.globalFallback.clearanceBufferMeters",
            DEFAULT_CLEARANCE_BUFFER_METERS),
        new CoarseRouteCostConfig(
            doubleProperty("repulsor.fieldplanner.globalFallback.distanceCostWeight", 1.0),
            doubleProperty("repulsor.fieldplanner.globalFallback.obstacleClearanceCostWeight", 0.0),
            doubleProperty("repulsor.fieldplanner.globalFallback.wallClearanceCostWeight", 0.0),
            doubleProperty("repulsor.fieldplanner.globalFallback.turnCostWeight", 0.05)));
  }

  private static double doubleProperty(String key, double fallback) {
    try {
      return Double.parseDouble(System.getProperty(key, Double.toString(fallback)));
    } catch (NumberFormatException ex) {
      return fallback;
    }
  }

  private static int intProperty(String key, int fallback) {
    try {
      return Integer.parseInt(System.getProperty(key, Integer.toString(fallback)));
    } catch (NumberFormatException ex) {
      return fallback;
    }
  }
}
