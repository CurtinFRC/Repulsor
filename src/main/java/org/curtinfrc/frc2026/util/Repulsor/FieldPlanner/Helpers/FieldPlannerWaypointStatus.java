package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Structured snapshot of the goal manager's waypoint/staging state for tests and telemetry. */
public record FieldPlannerWaypointStatus(
    Pose2d requestedGoal,
    Pose2d activeGoal,
    FieldPlannerWaypointDecision lastStrategyDecision,
    FieldPlannerWaypointObjectiveRole lastObjectiveRole,
    String transitionReason,
    boolean activeStage,
    Translation2d stagedAttractor,
    Translation2d stagedExitPoint,
    Translation2d stagedGateCenter,
    boolean exitPhase,
    boolean stagedComplete,
    boolean usingBypass,
    boolean centerReturn,
    int stagedModeTicks) {
  public FieldPlannerWaypointStatus {
    if (lastStrategyDecision == null)
      lastStrategyDecision = FieldPlannerWaypointDecision.useDefault();
    if (lastObjectiveRole == null) lastObjectiveRole = FieldPlannerWaypointObjectiveRole.ANY;
    if (transitionReason == null || transitionReason.isBlank()) transitionReason = "unknown";
  }

  public FieldPlannerWaypointStatus(
      Pose2d requestedGoal,
      Pose2d activeGoal,
      FieldPlannerWaypointDecision lastStrategyDecision,
      FieldPlannerWaypointObjectiveRole lastObjectiveRole,
      boolean activeStage,
      Translation2d stagedAttractor,
      Translation2d stagedExitPoint,
      Translation2d stagedGateCenter,
      boolean exitPhase,
      boolean stagedComplete,
      boolean usingBypass,
      boolean centerReturn,
      int stagedModeTicks) {
    this(
        requestedGoal,
        activeGoal,
        lastStrategyDecision,
        lastObjectiveRole,
        "unknown",
        activeStage,
        stagedAttractor,
        stagedExitPoint,
        stagedGateCenter,
        exitPhase,
        stagedComplete,
        usingBypass,
        centerReturn,
        stagedModeTicks);
  }
}
