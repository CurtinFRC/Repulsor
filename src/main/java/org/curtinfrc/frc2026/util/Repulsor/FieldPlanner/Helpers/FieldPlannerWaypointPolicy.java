package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import java.util.List;

/**
 * Season-custom waypoint policy consulted by {@link FieldPlannerGoalManager} whenever the active
 * {@link FieldPlannerWaypointStrategy} delegates to the default policy. Policies are registered on
 * {@link FieldPlannerWaypointConfig} and evaluated deterministically: proposals are grouped into
 * priority tiers (highest {@link FieldPlannerWaypointProposal#priority()} first, ties keep
 * registration order), and within a tier the shared candidate scorer picks the winner. Every
 * proposal passes through the same validation and scoring as built-in rule candidates.
 */
@FunctionalInterface
public interface FieldPlannerWaypointPolicy {
  /**
   * Returns zero or more waypoint proposals for the immutable planning context. Returning null is
   * treated the same as returning an empty list.
   *
   * @param context value used by this operation.
   */
  List<FieldPlannerWaypointProposal> propose(FieldPlannerWaypointContext context);
}
