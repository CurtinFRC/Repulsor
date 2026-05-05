package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

/**
 * Runtime tuning for planner behaviors that are intentionally separate from waypoint policy and
 * reactive bypass.
 *
 * <p>Waypoint policy decides when/where to stage. Reactive bypass handles short-horizon dynamic
 * detours. This config only controls the FieldPlanner's coarse global fallback and the narrow
 * force-through escape used when a dynamic obstacle blocks a near-wall, near-goal path.
 */
public record FieldPlannerRuntimeConfig(
    boolean globalFallbackEnabled,
    CoarseGlobalPlannerConfig globalFallbackConfig,
    double forceThroughGoalDistanceMeters,
    double forceThroughWallDistanceMeters) {
  private static final double DEFAULT_FORCE_THROUGH_GOAL_DISTANCE_METERS = 2.0;
  private static final double DEFAULT_FORCE_THROUGH_WALL_DISTANCE_METERS = 0.7;

  public FieldPlannerRuntimeConfig {
    if (globalFallbackConfig == null) globalFallbackConfig = CoarseGlobalPlannerConfig.defaults();
    forceThroughGoalDistanceMeters = Math.max(0.0, forceThroughGoalDistanceMeters);
    forceThroughWallDistanceMeters = Math.max(0.0, forceThroughWallDistanceMeters);
  }

  public static FieldPlannerRuntimeConfig defaults() {
    return new FieldPlannerRuntimeConfig(
        booleanProperty("repulsor.fieldplanner.globalFallback.enabled", true),
        CoarseGlobalPlannerConfig.defaults(),
        doubleProperty(
            "repulsor.fieldplanner.forceThrough.goalDistanceMeters",
            DEFAULT_FORCE_THROUGH_GOAL_DISTANCE_METERS),
        doubleProperty(
            "repulsor.fieldplanner.forceThrough.wallDistanceMeters",
            DEFAULT_FORCE_THROUGH_WALL_DISTANCE_METERS));
  }

  public FieldPlannerRuntimeConfig withGlobalFallbackConfig(CoarseGlobalPlannerConfig config) {
    return new FieldPlannerRuntimeConfig(
        globalFallbackEnabled,
        config,
        forceThroughGoalDistanceMeters,
        forceThroughWallDistanceMeters);
  }

  public FieldPlannerRuntimeConfig withGlobalFallbackEnabled(boolean enabled) {
    return new FieldPlannerRuntimeConfig(
        enabled,
        globalFallbackConfig,
        forceThroughGoalDistanceMeters,
        forceThroughWallDistanceMeters);
  }

  private static boolean booleanProperty(String key, boolean fallback) {
    return Boolean.parseBoolean(System.getProperty(key, Boolean.toString(fallback)));
  }

  private static double doubleProperty(String key, double fallback) {
    try {
      return Double.parseDouble(System.getProperty(key, Double.toString(fallback)));
    } catch (NumberFormatException ex) {
      return fallback;
    }
  }
}
