package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

/** Scores waypoint candidates without changing reactive bypass or force-through behavior. */
public final class FieldPlannerWaypointCandidateScorer {
  private FieldPlannerWaypointCandidateScorer() {}

  public record ScoredCandidate(FieldPlannerWaypointCandidate candidate, double score) {}

  public static Optional<ScoredCandidate> best(
      FieldPlannerWaypointContext context,
      List<FieldPlannerWaypointCandidate> candidates,
      FieldPlannerWaypointScoringConfig config) {
    if (context == null || candidates == null || candidates.isEmpty()) return Optional.empty();
    FieldPlannerWaypointScoringConfig weights =
        config == null ? FieldPlannerWaypointScoringConfig.defaults() : config;
    return candidates.stream()
        .filter(candidate -> candidate != null && candidate.entryPoint() != null)
        .map(candidate -> new ScoredCandidate(candidate, score(context, candidate, weights)))
        .max(Comparator.comparingDouble(ScoredCandidate::score));
  }

  public static double score(
      FieldPlannerWaypointContext context,
      FieldPlannerWaypointCandidate candidate,
      FieldPlannerWaypointScoringConfig config) {
    if (context == null || candidate == null || candidate.entryPoint() == null) {
      return Double.NEGATIVE_INFINITY;
    }
    FieldPlannerWaypointScoringConfig weights =
        config == null ? FieldPlannerWaypointScoringConfig.defaults() : config;
    Translation2d robot = context.robotPosition();
    Translation2d goal = context.requestedGoal().getTranslation();
    Translation2d entry = candidate.entryPoint();

    double routeDistance = robot.getDistance(entry) + entry.getDistance(goal);
    double directDistance = Math.max(1e-9, robot.getDistance(goal));
    double detour = Math.max(0.0, routeDistance - directDistance);
    double goalAlignment = directDistance / Math.max(1e-9, routeDistance);
    double obstacleClearance = approximateObstacleClearance(entry, context.obstacles());

    return candidate.preference() * weights.preferenceGain()
        - detour * weights.distanceCost()
        + goalAlignment * weights.goalAlignmentGain()
        + obstacleClearance * weights.obstacleClearanceGain();
  }

  private static double approximateObstacleClearance(
      Translation2d point, List<? extends Obstacle> obstacles) {
    if (point == null || obstacles == null || obstacles.isEmpty()) return 1.0;
    double forceMagnitude = 0.0;
    for (Obstacle obstacle : obstacles) {
      if (obstacle == null) continue;
      var force = obstacle.sampleForceAtPosition(point, point);
      if (force != null && Double.isFinite(force.getNorm())) {
        forceMagnitude += force.getNorm();
      }
    }
    return 1.0 / (1.0 + forceMagnitude);
  }
}
