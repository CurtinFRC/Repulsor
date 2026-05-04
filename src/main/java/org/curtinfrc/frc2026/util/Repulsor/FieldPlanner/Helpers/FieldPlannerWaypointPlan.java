package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;

/**
 * A strategy-produced staged waypoint sequence. The entry point is the immediate temporary goal.
 * The optional exit point lets a strategy ask the goal manager to advance through a second waypoint
 * before releasing to the requested goal.
 */
public record FieldPlannerWaypointPlan(
    Translation2d entryPoint,
    Translation2d exitPoint,
    GatedAttractorObstacle gate,
    boolean centerReturn,
    boolean usingBypass,
    boolean forceStage) {
  public FieldPlannerWaypointPlan {
    if (entryPoint == null) throw new IllegalArgumentException("entryPoint cannot be null");
  }

  public static FieldPlannerWaypointPlan single(Translation2d entryPoint) {
    return new FieldPlannerWaypointPlan(entryPoint, null, null, false, false, true);
  }

  public static FieldPlannerWaypointPlan through(
      Translation2d entryPoint, Translation2d exitPoint, GatedAttractorObstacle gate) {
    return new FieldPlannerWaypointPlan(entryPoint, exitPoint, gate, false, gate != null, true);
  }
}
