package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import java.util.List;

/** Waypoint strategy backed by ordered declarative rules. */
public final class FieldPlannerWaypointRuleStrategy implements FieldPlannerWaypointStrategy {
  private final List<FieldPlannerWaypointRule> rules;

  public FieldPlannerWaypointRuleStrategy(List<FieldPlannerWaypointRule> rules) {
    this.rules = rules == null ? List.of() : List.copyOf(rules);
  }

  public static FieldPlannerWaypointRuleStrategy of(FieldPlannerWaypointRule... rules) {
    return new FieldPlannerWaypointRuleStrategy(rules == null ? List.of() : List.of(rules));
  }

  @Override
  public FieldPlannerWaypointDecision decide(FieldPlannerWaypointContext context) {
    for (FieldPlannerWaypointRule rule : rules) {
      if (rule == null) continue;
      FieldPlannerWaypointDecision decision = rule.decide(context);
      if (decision != null && !decision.usesDefaultPolicy()) return decision;
    }
    return FieldPlannerWaypointDecision.useDefault();
  }
}
