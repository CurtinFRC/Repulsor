package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;

/**
 * Runtime tuning for planner behaviors that are intentionally separate from waypoint policy and
 * reactive bypass.
 *
 * <p>Waypoint policy decides when/where to stage. Reactive bypass handles short-horizon dynamic
 * detours. This config only controls the FieldPlanner's coarse global fallback, the reroute
 * candidate query, and the narrow force-through escape used when a dynamic obstacle blocks a
 * near-wall, near-goal path.
 */
public record FieldPlannerRuntimeConfig(
    boolean globalFallbackEnabled,
    CoarseGlobalPlannerConfig globalFallbackConfig,
    double forceThroughGoalDistanceMeters,
    double forceThroughWallDistanceMeters,
    double rerouteCandidateRadiusMeters,
    int rerouteCandidateCount,
    Alliance fallbackAllianceWhenUnknown) {
  private static final double DEFAULT_FORCE_THROUGH_GOAL_DISTANCE_METERS = 2.0;
  private static final double DEFAULT_FORCE_THROUGH_WALL_DISTANCE_METERS = 0.7;
  private static final double DEFAULT_REROUTE_CANDIDATE_RADIUS_METERS = 3.5;
  private static final int DEFAULT_REROUTE_CANDIDATE_COUNT = 8;
  private static final Alliance DEFAULT_FALLBACK_ALLIANCE_WHEN_UNKNOWN = Alliance.kRed;

  public FieldPlannerRuntimeConfig {
    if (globalFallbackConfig == null) globalFallbackConfig = CoarseGlobalPlannerConfig.defaults();
    forceThroughGoalDistanceMeters = Math.max(0.0, forceThroughGoalDistanceMeters);
    forceThroughWallDistanceMeters = Math.max(0.0, forceThroughWallDistanceMeters);
    rerouteCandidateRadiusMeters =
        Double.isFinite(rerouteCandidateRadiusMeters)
            ? Math.max(0.0, rerouteCandidateRadiusMeters)
            : DEFAULT_REROUTE_CANDIDATE_RADIUS_METERS;
    rerouteCandidateCount =
        rerouteCandidateCount >= 1 ? rerouteCandidateCount : DEFAULT_REROUTE_CANDIDATE_COUNT;
    if (fallbackAllianceWhenUnknown == null) {
      fallbackAllianceWhenUnknown = DEFAULT_FALLBACK_ALLIANCE_WHEN_UNKNOWN;
    }
  }

  public FieldPlannerRuntimeConfig(
      boolean globalFallbackEnabled,
      CoarseGlobalPlannerConfig globalFallbackConfig,
      double forceThroughGoalDistanceMeters,
      double forceThroughWallDistanceMeters) {
    this(
        globalFallbackEnabled,
        globalFallbackConfig,
        forceThroughGoalDistanceMeters,
        forceThroughWallDistanceMeters,
        DEFAULT_REROUTE_CANDIDATE_RADIUS_METERS,
        DEFAULT_REROUTE_CANDIDATE_COUNT,
        DEFAULT_FALLBACK_ALLIANCE_WHEN_UNKNOWN);
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
            DEFAULT_FORCE_THROUGH_WALL_DISTANCE_METERS),
        doubleProperty(
            "repulsor.fieldplanner.reroute.radiusMeters", DEFAULT_REROUTE_CANDIDATE_RADIUS_METERS),
        (int)
            doubleProperty(
                "repulsor.fieldplanner.reroute.candidateCount", DEFAULT_REROUTE_CANDIDATE_COUNT),
        allianceProperty("repulsor.fieldplanner.reroute.fallbackAlliance"));
  }

  public static double defaultRerouteCandidateRadiusMeters() {
    return DEFAULT_REROUTE_CANDIDATE_RADIUS_METERS;
  }

  public static int defaultRerouteCandidateCount() {
    return DEFAULT_REROUTE_CANDIDATE_COUNT;
  }

  public static Alliance defaultFallbackAllianceWhenUnknown() {
    return DEFAULT_FALLBACK_ALLIANCE_WHEN_UNKNOWN;
  }

  public FieldPlannerRuntimeConfig withGlobalFallbackConfig(CoarseGlobalPlannerConfig config) {
    return new FieldPlannerRuntimeConfig(
        globalFallbackEnabled,
        config,
        forceThroughGoalDistanceMeters,
        forceThroughWallDistanceMeters,
        rerouteCandidateRadiusMeters,
        rerouteCandidateCount,
        fallbackAllianceWhenUnknown);
  }

  public FieldPlannerRuntimeConfig withGlobalFallbackEnabled(boolean enabled) {
    return new FieldPlannerRuntimeConfig(
        enabled,
        globalFallbackConfig,
        forceThroughGoalDistanceMeters,
        forceThroughWallDistanceMeters,
        rerouteCandidateRadiusMeters,
        rerouteCandidateCount,
        fallbackAllianceWhenUnknown);
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

  private static Alliance allianceProperty(String key) {
    String value = System.getProperty(key);
    if (value == null || value.isBlank()) return DEFAULT_FALLBACK_ALLIANCE_WHEN_UNKNOWN;
    if ("red".equalsIgnoreCase(value.trim())) return Alliance.kRed;
    if ("blue".equalsIgnoreCase(value.trim())) return Alliance.kBlue;
    return DEFAULT_FALLBACK_ALLIANCE_WHEN_UNKNOWN;
  }
}
