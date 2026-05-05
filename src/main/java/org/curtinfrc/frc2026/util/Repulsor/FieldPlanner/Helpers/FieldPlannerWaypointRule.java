package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

/** Declarative waypoint rule that can be reused across games and autonomous strategies. */
public record FieldPlannerWaypointRule(
    String name,
    FieldPlannerWaypointObjectiveRole objectiveRole,
    FieldPlannerWaypointZone fromZone,
    FieldPlannerWaypointZone toZone,
    boolean onlyWhenNotAlreadyStaging,
    boolean direct,
    List<FieldPlannerWaypointCandidate> candidates,
    FieldPlannerWaypointScoringConfig scoringConfig) {
  public FieldPlannerWaypointRule {
    if (name == null || name.isBlank()) name = "waypoint-rule";
    if (objectiveRole == null) objectiveRole = FieldPlannerWaypointObjectiveRole.ANY;
    if (candidates == null) candidates = List.of();
    else candidates = List.copyOf(candidates);
    if (scoringConfig == null) scoringConfig = FieldPlannerWaypointScoringConfig.defaults();
  }

  public static FieldPlannerWaypointRule stageBetweenZones(
      String name,
      FieldPlannerWaypointObjectiveRole objectiveRole,
      FieldPlannerWaypointZone fromZone,
      FieldPlannerWaypointZone toZone,
      List<FieldPlannerWaypointCandidate> candidates) {
    return new FieldPlannerWaypointRule(
        name,
        objectiveRole,
        fromZone,
        toZone,
        true,
        false,
        candidates,
        FieldPlannerWaypointScoringConfig.defaults());
  }

  public boolean matches(FieldPlannerWaypointContext context) {
    Objects.requireNonNull(context);
    if (!objectiveRole.matches(context.objectiveRole())) return false;
    if (onlyWhenNotAlreadyStaging && context.currentlyStaging()) return false;
    Translation2d robot = context.robotPosition();
    Translation2d goal = context.requestedGoal().getTranslation();
    return zoneMatches(fromZone, robot, context) && zoneMatches(toZone, goal, context);
  }

  public FieldPlannerWaypointDecision decide(FieldPlannerWaypointContext context) {
    if (!matches(context)) return FieldPlannerWaypointDecision.useDefault();
    if (direct) return FieldPlannerWaypointDecision.direct();
    Optional<FieldPlannerWaypointCandidateScorer.ScoredCandidate> best =
        FieldPlannerWaypointCandidateScorer.best(context, candidates, scoringConfig);
    return best.map(scored -> FieldPlannerWaypointDecision.stage(scored.candidate().toPlan()))
        .orElseGet(FieldPlannerWaypointDecision::useDefault);
  }

  private static boolean zoneMatches(
      FieldPlannerWaypointZone zone, Translation2d point, FieldPlannerWaypointContext context) {
    if (zone == null) return true;
    return zone.contains(point)
        || FieldPlannerWaypointZone.wholeField(
                context.fieldLengthMeters(), context.fieldWidthMeters())
            .equals(zone);
  }
}
