package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

/**
 * Strategy hook for staged waypoint generation. Return {@code useDefault()} to delegate to the
 * built-in goal-manager policy, {@code stage(plan)} to provide exact waypoints, or {@code direct()}
 * to suppress staging for this update.
 */
@FunctionalInterface
public interface FieldPlannerWaypointStrategy {
  FieldPlannerWaypointDecision decide(FieldPlannerWaypointContext context);

  static FieldPlannerWaypointStrategy defaults() {
    return context -> FieldPlannerWaypointDecision.useDefault();
  }
}
