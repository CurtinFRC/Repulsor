package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;

/** Candidate temporary waypoint before scoring selects the actual stage plan. */
public record FieldPlannerWaypointCandidate(
    String name,
    Translation2d entryPoint,
    Translation2d exitPoint,
    GatedAttractorObstacle gate,
    double preference,
    boolean forceStage) {
  public FieldPlannerWaypointCandidate {
    if (name == null || name.isBlank()) name = "candidate";
    if (entryPoint == null) throw new IllegalArgumentException("entryPoint cannot be null");
  }

  public static FieldPlannerWaypointCandidate single(String name, Translation2d entryPoint) {
    return new FieldPlannerWaypointCandidate(name, entryPoint, null, null, 0.0, true);
  }

  public FieldPlannerWaypointPlan toPlan() {
    return new FieldPlannerWaypointPlan(entryPoint, exitPoint, gate, false, false, forceStage);
  }
}
