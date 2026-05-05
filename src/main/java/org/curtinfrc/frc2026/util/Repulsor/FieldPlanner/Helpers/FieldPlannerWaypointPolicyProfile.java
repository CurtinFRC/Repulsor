package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import java.util.List;

/** Named waypoint policy profile, useful for autonomous modes, driver modes, or game strategies. */
public record FieldPlannerWaypointPolicyProfile(
    String name, FieldPlannerWaypointConfig config, FieldPlannerWaypointStrategy strategy) {
  public FieldPlannerWaypointPolicyProfile {
    if (name == null || name.isBlank()) name = "default";
    if (config == null) config = FieldPlannerWaypointConfig.defaults();
    if (strategy == null) strategy = FieldPlannerWaypointStrategy.defaults();
  }

  public static FieldPlannerWaypointPolicyProfile fromRules(
      String name, FieldPlannerWaypointConfig config, List<FieldPlannerWaypointRule> rules) {
    return new FieldPlannerWaypointPolicyProfile(
        name, config, new FieldPlannerWaypointRuleStrategy(rules));
  }
}
