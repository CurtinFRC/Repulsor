package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import java.util.Optional;

/**
 * Strategy hook for staged waypoint generation. Return {@link Optional#empty()} to let the default
 * goal-manager policy decide. Return a plan to define exactly when and where this update should
 * stage.
 */
@FunctionalInterface
public interface FieldPlannerWaypointStrategy {
  Optional<FieldPlannerWaypointPlan> plan(FieldPlannerWaypointContext context);

  static FieldPlannerWaypointStrategy defaults() {
    return context -> Optional.empty();
  }
}
