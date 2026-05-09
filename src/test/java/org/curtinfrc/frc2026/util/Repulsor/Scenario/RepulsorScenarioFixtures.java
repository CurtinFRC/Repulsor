package org.curtinfrc.frc2026.util.Repulsor.Scenario;

import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldDefinition;

/** Game/profile-specific scenario fixtures built from reusable templates. */
public final class RepulsorScenarioFixtures {
  private RepulsorScenarioFixtures() {}

  public static List<RepulsorPlannerScenario> commonPlannerScenarios(FieldDefinition field) {
    String prefix = field == null ? "generic" : field.gameName().toLowerCase();
    double length = field == null ? 16.0 : field.geometry().lengthMeters();
    double width = field == null ? 8.0 : field.geometry().widthMeters();
    return List.of(
        RepulsorPlannerScenarioTemplates.clearRoute(prefix + "-clear-route"),
        RepulsorPlannerScenarioTemplates.centerCrossing(prefix + "-center-crossing", length, width),
        RepulsorPlannerScenarioTemplates.dynamicObstacle(prefix + "-dynamic-obstacle"),
        RepulsorPlannerScenarioTemplates.edgeOfField(prefix + "-edge-of-field", width));
  }

  public static List<RepulsorPlannerScenario> scoringTransitionScenarios(FieldDefinition field) {
    String prefix = field == null ? "generic" : field.gameName().toLowerCase();
    return List.of(
        RepulsorPlannerScenarioTemplates.forcedCollect(prefix + "-forced-collect"),
        RepulsorPlannerScenarioTemplates.forcedScore(prefix + "-forced-score"),
        RepulsorPlannerScenarioTemplates.customWaypointStage(prefix + "-strategy-waypoint"));
  }

  public static List<RepulsorPlannerScenario> blockedRouteScenarios(FieldDefinition field) {
    String prefix = field == null ? "generic" : field.gameName().toLowerCase();
    return List.of(
        RepulsorPlannerScenarioTemplates.blockedRoute(prefix + "-blocked-route"),
        RepulsorPlannerScenarioTemplates.blockedWithoutGlobalFallback(
            prefix + "-blocked-no-fallback"),
        RepulsorPlannerScenarioTemplates.defensiveObstacle(prefix + "-defensive-obstacle"));
  }
}
