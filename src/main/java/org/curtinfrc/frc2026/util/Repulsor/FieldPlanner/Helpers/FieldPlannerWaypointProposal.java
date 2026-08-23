package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;

/**
 * Single waypoint proposal emitted by a {@link FieldPlannerWaypointPolicy}. The entry point is the
 * immediate temporary goal; the optional exit point advances through a second waypoint before
 * releasing to the requested goal. Role and zone constraints filter when the proposal applies, and
 * priority orders proposals across policies (higher values win; ties keep registration order).
 */
public record FieldPlannerWaypointProposal(
    String name,
    Translation2d entryPoint,
    Translation2d exitPoint,
    GatedAttractorObstacle gate,
    FieldPlannerWaypointObjectiveRole role,
    FieldPlannerWaypointZone robotZone,
    FieldPlannerWaypointZone goalZone,
    double preference,
    int priority,
    boolean forceStage) {
  public FieldPlannerWaypointProposal {
    if (name == null || name.isBlank()) name = "policy-proposal";
    if (role == null) role = FieldPlannerWaypointObjectiveRole.ANY;
    if (!Double.isFinite(preference)) preference = 0.0;
  }

  /**
   * Returns a forced-stage proposal at the given entry point with default priority.
   *
   * @param name value used by this operation.
   * @param entryPoint distance or field-coordinate value in meters.
   */
  public static FieldPlannerWaypointProposal at(String name, Translation2d entryPoint) {
    return new FieldPlannerWaypointProposal(
        name, entryPoint, null, null, null, null, null, 0.0, 0, true);
  }

  public FieldPlannerWaypointProposal withPriority(int priority) {
    return new FieldPlannerWaypointProposal(
        name, entryPoint, exitPoint, gate, role, robotZone, goalZone, preference, priority,
        forceStage);
  }

  public FieldPlannerWaypointProposal withExitPoint(Translation2d exitPoint) {
    return new FieldPlannerWaypointProposal(
        name, entryPoint, exitPoint, gate, role, robotZone, goalZone, preference, priority,
        forceStage);
  }

  public FieldPlannerWaypointProposal withPreference(double preference) {
    return new FieldPlannerWaypointProposal(
        name, entryPoint, exitPoint, gate, role, robotZone, goalZone, preference, priority,
        forceStage);
  }

  /** Maps this proposal onto the shared candidate type used by built-in scoring. */
  public FieldPlannerWaypointCandidate toCandidate() {
    return new FieldPlannerWaypointCandidate(
        name, entryPoint, exitPoint, gate, preference, forceStage);
  }
}
