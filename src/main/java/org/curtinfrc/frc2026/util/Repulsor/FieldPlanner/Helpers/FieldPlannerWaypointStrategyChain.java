package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import java.util.ArrayList;
import java.util.List;

/** Composes multiple strategy layers without mixing waypoint decisions with reactive bypass. */
public final class FieldPlannerWaypointStrategyChain implements FieldPlannerWaypointStrategy {
  private final List<FieldPlannerWaypointStrategy> strategies;

  public FieldPlannerWaypointStrategyChain(List<FieldPlannerWaypointStrategy> strategies) {
    if (strategies == null) {
      this.strategies = List.of();
      return;
    }
    List<FieldPlannerWaypointStrategy> filtered = new ArrayList<>();
    for (FieldPlannerWaypointStrategy strategy : strategies) {
      if (strategy != null) filtered.add(strategy);
    }
    this.strategies = List.copyOf(filtered);
  }

  public static FieldPlannerWaypointStrategy of(FieldPlannerWaypointStrategy... strategies) {
    return new FieldPlannerWaypointStrategyChain(
        strategies == null ? List.of() : List.of(strategies));
  }

  @Override
  public FieldPlannerWaypointDecision decide(FieldPlannerWaypointContext context) {
    for (FieldPlannerWaypointStrategy strategy : strategies) {
      FieldPlannerWaypointDecision decision = strategy.decide(context);
      if (decision != null && !decision.usesDefaultPolicy()) return decision;
    }
    return FieldPlannerWaypointDecision.useDefault();
  }
}
