package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

/**
 * Explicit result from a waypoint strategy. This lets game/strategy code either delegate to the
 * built-in corridor policy, provide an exact staged waypoint plan, or suppress staging entirely for
 * this update.
 */
public record FieldPlannerWaypointDecision(Mode mode, FieldPlannerWaypointPlan plan) {
  public enum Mode {
    USE_DEFAULT,
    STAGE,
    DIRECT
  }

  public FieldPlannerWaypointDecision {
    if (mode == null) mode = Mode.USE_DEFAULT;
    if (mode == Mode.STAGE && plan == null) {
      throw new IllegalArgumentException("STAGE decisions require a waypoint plan");
    }
    if (mode != Mode.STAGE) plan = null;
  }

  public static FieldPlannerWaypointDecision useDefault() {
    return new FieldPlannerWaypointDecision(Mode.USE_DEFAULT, null);
  }

  public static FieldPlannerWaypointDecision stage(FieldPlannerWaypointPlan plan) {
    return new FieldPlannerWaypointDecision(Mode.STAGE, plan);
  }

  public static FieldPlannerWaypointDecision direct() {
    return new FieldPlannerWaypointDecision(Mode.DIRECT, null);
  }

  public boolean usesDefaultPolicy() {
    return mode == Mode.USE_DEFAULT;
  }

  public boolean stages() {
    return mode == Mode.STAGE;
  }

  public boolean goesDirectlyToRequestedGoal() {
    return mode == Mode.DIRECT;
  }
}
