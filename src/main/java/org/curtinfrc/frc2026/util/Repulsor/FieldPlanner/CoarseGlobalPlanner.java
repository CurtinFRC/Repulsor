package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PredictedDynamicObstacleEnvelope;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;

/**
 * Coarse A* fallback planner used when the local repulsor force field has no direct line to the
 * goal.
 *
 * <p>This intentionally returns only the next waypoint. FieldPlanner still owns smoothing, velocity
 * limiting, heading control, and dynamic bypass.
 */
public final class CoarseGlobalPlanner {
  private static final double SHARP_TURN_RADIANS = Math.toRadians(50.0);
  private static final double NARROW_PASSAGE_CLEARANCE_METERS = 0.35;
  private static final double MIN_CORNER_LOOKAHEAD_FRACTION = 0.45;
  private static final double WAYPOINT_HYSTERESIS_METERS = 0.55;

  private final CoarseGlobalPlannerConfig config;
  private CoarseGlobalPlannerStats lastStats = CoarseGlobalPlannerStats.empty();
  private Translation2d previousWaypoint = null;

  public CoarseGlobalPlanner() {
    this(CoarseGlobalPlannerConfig.defaults());
  }

  public CoarseGlobalPlanner(double cellMeters, double waypointLookaheadMeters) {
    this(
        new CoarseGlobalPlannerConfig(
            cellMeters,
            waypointLookaheadMeters,
            CoarseGlobalPlannerConfig.defaults().maxExpandedNodes(),
            CoarseGlobalPlannerConfig.defaults().maxRuntimeSeconds()));
  }

  public CoarseGlobalPlanner(CoarseGlobalPlannerConfig config) {
    this.config = config == null ? CoarseGlobalPlannerConfig.defaults() : config;
  }

  public CoarseGlobalPlannerStats lastStats() {
    return lastStats;
  }

  public Optional<Pose2d> nextWaypoint(
      Translation2d start,
      Pose2d goal,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters) {
    long startNanos = System.nanoTime();
    if (start == null || goal == null) {
      finishStats(
          false, false, false, 0, 0, 0, startNanos, CoarseGlobalPlannerFailureReason.INVALID_INPUT);
      return Optional.empty();
    }

    int nx = Math.max(2, (int) Math.ceil(fieldLengthMeters / config.cellMeters()) + 1);
    int ny = Math.max(2, (int) Math.ceil(fieldWidthMeters / config.cellMeters()) + 1);
    ClearanceField clearanceField =
        buildClearanceField(
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny);
    Node s = nearestNode(start, nx, ny, fieldLengthMeters, fieldWidthMeters);
    Node g = nearestNode(goal.getTranslation(), nx, ny, fieldLengthMeters, fieldWidthMeters);
    if (!isFree(
        s,
        obstacles,
        robotLengthMeters,
        robotWidthMeters,
        fieldLengthMeters,
        fieldWidthMeters,
        nx,
        ny)) {
      finishStats(
          false, false, false, 0, 0, 0, startNanos, CoarseGlobalPlannerFailureReason.START_BLOCKED);
      return Optional.empty();
    }
    boolean goalBlocked =
        !isFree(
            g,
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny);
    if (goalBlocked && !config.partialRouteFallbackEnabled()) {
      finishStats(
          false, false, false, 0, 0, 0, startNanos, CoarseGlobalPlannerFailureReason.GOAL_BLOCKED);
      return Optional.empty();
    }

    int n = nx * ny;
    double[] best = new double[n];
    int[] parent = new int[n];
    boolean[] closed = new boolean[n];
    for (int i = 0; i < n; i++) {
      best[i] = Double.POSITIVE_INFINITY;
      parent[i] = -1;
    }
    int[] expansionOrder = new int[n];
    double[] liveObstacleCostCache = new double[n];
    Arrays.fill(liveObstacleCostCache, Double.NaN);

    PriorityQueue<Entry> open = new PriorityQueue<>((a, b) -> Double.compare(a.f, b.f));
    int startIdx = index(s, ny);
    int goalIdx = index(g, ny);
    best[startIdx] = 0.0;
    open.add(new Entry(s, 0.0, heuristic(s, g)));

    int expanded = 0;
    int generated = 1;
    boolean timedOut = false;
    boolean exhaustedBudget = false;
    long maxRuntimeNanos = (long) (config.maxRuntimeSeconds() * 1_000_000_000.0);
    long searchStartNanos = System.nanoTime();

    while (!open.isEmpty()) {
      if (System.nanoTime() - searchStartNanos > maxRuntimeNanos) {
        timedOut = true;
        break;
      }
      if (expanded >= config.maxExpandedNodes()) {
        exhaustedBudget = true;
        break;
      }

      Entry cur = open.poll();
      int curIdx = index(cur.node, ny);
      if (closed[curIdx]) continue;
      closed[curIdx] = true;
      expansionOrder[expanded++] = curIdx;
      if (curIdx == goalIdx) break;

      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0) continue;
          Node next = new Node(cur.node.x + dx, cur.node.y + dy);
          if (next.x < 0 || next.x >= nx || next.y < 0 || next.y >= ny) continue;
          int nextIdx = index(next, ny);
          if (closed[nextIdx]) continue;
          if (!isFree(
              next,
              obstacles,
              robotLengthMeters,
              robotWidthMeters,
              fieldLengthMeters,
              fieldWidthMeters,
              nx,
              ny)) {
            continue;
          }
          if (!edgeFree(
              cur.node,
              next,
              obstacles,
              robotLengthMeters,
              robotWidthMeters,
              fieldLengthMeters,
              fieldWidthMeters,
              nx,
              ny)) {
            continue;
          }

          CoarseRouteCostBreakdown edgeCost =
              edgeCost(
                  parent[curIdx],
                  cur.node,
                  next,
                  obstacles,
                  robotLengthMeters,
                  robotWidthMeters,
                  fieldLengthMeters,
                  fieldWidthMeters,
                  nx,
                  ny,
                  clearanceField,
                  liveObstacleCostCache);
          double tentative = best[curIdx] + edgeCost.total();
          if (tentative < best[nextIdx]) {
            best[nextIdx] = tentative;
            parent[nextIdx] = curIdx;
            open.add(new Entry(next, tentative, tentative + heuristic(next, g)));
            generated++;
          }
        }
      }
    }

    if (parent[goalIdx] < 0 && goalIdx != startIdx) {
      double directGoalDistance = start.getDistance(goal.getTranslation());
      int bestPartialIdx = -1;
      double bestPartialScore = Double.POSITIVE_INFINITY;
      if (config.partialRouteFallbackEnabled()) {
        for (int i = 0; i < expanded; i++) {
          int nodeIdx = expansionOrder[i];
          PartialRouteCandidate partialCandidate =
              partialRouteCandidate(
                  new Node(nodeIdx / ny, nodeIdx % ny),
                  nodeIdx,
                  startIdx,
                  parent,
                  best,
                  goal.getTranslation(),
                  directGoalDistance,
                  fieldLengthMeters,
                  fieldWidthMeters,
                  nx,
                  ny,
                  clearanceField,
                  obstacles,
                  robotLengthMeters,
                  robotWidthMeters);
          if (partialCandidate.usable() && partialCandidate.score() < bestPartialScore) {
            bestPartialScore = partialCandidate.score();
            bestPartialIdx = nodeIdx;
          }
        }
      }
      Optional<Pose2d> partialWaypoint =
          partialRouteWaypoint(
              bestPartialIdx,
              startIdx,
              parent,
              ny,
              obstacles,
              robotLengthMeters,
              robotWidthMeters,
              fieldLengthMeters,
              fieldWidthMeters,
              nx,
              clearanceField,
              start,
              goal.getTranslation(),
              timedOut,
              exhaustedBudget,
              expanded,
              generated,
              startNanos);
      if (partialWaypoint.isPresent()) return partialWaypoint;
      previousWaypoint = null;
      finishStats(
          false,
          timedOut,
          exhaustedBudget,
          expanded,
          generated,
          0,
          startNanos,
          noFullRouteFailureReason(timedOut, exhaustedBudget, goalBlocked));
      return Optional.empty();
    }
    List<Node> path = reconstruct(goalIdx, startIdx, parent, ny);
    if (path.size() < 2) {
      previousWaypoint = null;
      finishStats(
          false,
          timedOut,
          exhaustedBudget,
          expanded,
          generated,
          path.size(),
          startNanos,
          CoarseGlobalPlannerFailureReason.PATH_TOO_SHORT);
      return Optional.empty();
    }

    List<Node> smoothedPath =
        smoothPath(
            path,
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny);
    CoarseRouteClearanceMetrics routeClearanceMetrics = routeClearance(path, clearanceField);
    CoarseRouteCostBreakdown routeCost =
        routeCost(
            path,
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny,
            clearanceField,
            liveObstacleCostCache);

    LookaheadSelection selection =
        chooseLookahead(
            smoothedPath,
            clearanceField,
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny,
            start);
    Translation2d waypoint = selection.point();
    previousWaypoint = waypoint;
    Rotation2d heading = goal.getTranslation().minus(waypoint).getAngle();
    finishStats(
        true,
        timedOut,
        exhaustedBudget,
        expanded,
        generated,
        path.size(),
        smoothedPath.size(),
        routeCost,
        routeClearanceMetrics,
        selection.index(),
        selection.reason(),
        startNanos,
        CoarseGlobalPlannerFailureReason.NONE);
    return Optional.of(new Pose2d(waypoint, heading));
  }

  private CoarseGlobalPlannerFailureReason noFullRouteFailureReason(
      boolean timedOut, boolean exhaustedBudget, boolean goalBlocked) {
    if (timedOut) return CoarseGlobalPlannerFailureReason.TIMEOUT;
    if (exhaustedBudget) return CoarseGlobalPlannerFailureReason.NODE_BUDGET;
    if (config.partialRouteFallbackEnabled()) {
      return CoarseGlobalPlannerFailureReason.PARTIAL_ROUTE_REJECTED_UNSAFE;
    }
    return goalBlocked
        ? CoarseGlobalPlannerFailureReason.GOAL_BLOCKED
        : CoarseGlobalPlannerFailureReason.NO_ROUTE;
  }

  private Optional<Pose2d> partialRouteWaypoint(
      int partialIdx,
      int startIdx,
      int[] parent,
      int ny,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      ClearanceField clearanceField,
      Translation2d start,
      Translation2d goal,
      boolean timedOut,
      boolean exhaustedBudget,
      int expanded,
      int generated,
      long startNanos) {
    if (!config.partialRouteFallbackEnabled() || partialIdx < 0 || parent == null) {
      return Optional.empty();
    }
    List<Node> path = reconstruct(partialIdx, startIdx, parent, ny);
    if (path.size() < 2) return Optional.empty();
    List<Node> smoothedPath =
        smoothPath(
            path,
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny);
    CoarseRouteClearanceMetrics routeClearanceMetrics = routeClearance(path, clearanceField);
    double[] partialLiveObstacleCostCache = new double[nx * ny];
    Arrays.fill(partialLiveObstacleCostCache, Double.NaN);
    CoarseRouteCostBreakdown routeCost =
        routeCost(
            path,
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny,
            clearanceField,
            partialLiveObstacleCostCache);
    LookaheadSelection selection =
        chooseLookahead(
            smoothedPath,
            clearanceField,
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny,
            start);
    Translation2d waypoint = selection.point();
    if (waypoint == null || start.getDistance(waypoint) < config.cellMeters() * 0.5) {
      return Optional.empty();
    }
    previousWaypoint = waypoint;
    Rotation2d heading = goal.minus(waypoint).getAngle();
    finishStats(
        true,
        timedOut,
        exhaustedBudget,
        expanded,
        generated,
        path.size(),
        smoothedPath.size(),
        routeCost,
        routeClearanceMetrics,
        selection.index(),
        selection.reason(),
        startNanos,
        CoarseGlobalPlannerFailureReason.PARTIAL_ROUTE_USED);
    return Optional.of(new Pose2d(waypoint, heading));
  }

  private PartialRouteCandidate partialRouteCandidate(
      Node node,
      int nodeIdx,
      int startIdx,
      int[] parent,
      double[] best,
      Translation2d goal,
      double directGoalDistance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny,
      ClearanceField clearanceField,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters) {
    if (!config.partialRouteFallbackEnabled()
        || node == null
        || nodeIdx == startIdx
        || parent == null
        || best == null
        || parent[nodeIdx] < 0) {
      return PartialRouteCandidate.unusable();
    }
    Translation2d point = toPoint(node, fieldLengthMeters, fieldWidthMeters, nx, ny);
    double remaining = point.getDistance(goal);
    double progress = directGoalDistance - remaining;
    if (progress < config.partialRouteMinProgressMeters()) {
      return PartialRouteCandidate.unusable();
    }
    double clearance =
        clearanceField == null
            ? Double.POSITIVE_INFINITY
            : clearanceField.routeClearanceMeters(node);
    if (clearance < config.partialRouteMinClearanceMeters()) {
      return PartialRouteCandidate.unusable();
    }
    if (!hasPartialRouteEscape(
        node,
        point,
        goal,
        obstacles,
        robotLengthMeters,
        robotWidthMeters,
        fieldLengthMeters,
        fieldWidthMeters,
        nx,
        ny)) {
      return PartialRouteCandidate.unusable();
    }
    double boundedClearance = Double.isFinite(clearance) ? Math.min(clearance, 2.0) : 2.0;
    double pathCost = Double.isFinite(best[nodeIdx]) ? best[nodeIdx] : 0.0;
    double score = remaining + 0.15 * pathCost - 0.25 * boundedClearance;
    return new PartialRouteCandidate(true, score);
  }

  private boolean hasPartialRouteEscape(
      Node node,
      Translation2d point,
      Translation2d goal,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny) {
    int exits = 0;
    double remaining = point.getDistance(goal);
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        Node next = new Node(node.x + dx, node.y + dy);
        if (next.x < 0 || next.x >= nx || next.y < 0 || next.y >= ny) continue;
        if (!isFree(
            next,
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny)) {
          continue;
        }
        if (!edgeFree(
            node,
            next,
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny)) {
          continue;
        }
        Translation2d nextPoint = toPoint(next, fieldLengthMeters, fieldWidthMeters, nx, ny);
        if (nextPoint.getDistance(goal) <= remaining + config.cellMeters()) exits++;
      }
    }
    return exits >= 2;
  }

  private void finishStats(
      boolean found,
      boolean timedOut,
      boolean exhaustedBudget,
      int expanded,
      int generated,
      int pathNodes,
      long startNanos,
      CoarseGlobalPlannerFailureReason failureReason) {
    finishStats(
        found,
        timedOut,
        exhaustedBudget,
        expanded,
        generated,
        pathNodes,
        pathNodes,
        CoarseRouteCostBreakdown.empty(),
        CoarseRouteClearanceMetrics.empty(),
        -1,
        CoarseGlobalPlannerWaypointReason.NONE,
        startNanos,
        failureReason);
  }

  private void finishStats(
      boolean found,
      boolean timedOut,
      boolean exhaustedBudget,
      int expanded,
      int generated,
      int rawPathNodes,
      int pathNodes,
      CoarseRouteCostBreakdown routeCost,
      CoarseRouteClearanceMetrics clearanceMetrics,
      int selectedWaypointIndex,
      CoarseGlobalPlannerWaypointReason selectedWaypointReason,
      long startNanos,
      CoarseGlobalPlannerFailureReason failureReason) {
    lastStats =
        new CoarseGlobalPlannerStats(
            found,
            timedOut,
            exhaustedBudget,
            expanded,
            generated,
            rawPathNodes,
            pathNodes,
            routeCost,
            clearanceMetrics,
            selectedWaypointIndex,
            selectedWaypointReason,
            System.nanoTime() - startNanos,
            failureReason);
  }

  private CoarseRouteCostBreakdown routeCost(
      List<Node> path,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny,
      ClearanceField clearanceField,
      double[] liveObstacleCostCache) {
    if (path == null || path.size() < 2) return CoarseRouteCostBreakdown.empty();
    CoarseRouteCostBreakdown total = CoarseRouteCostBreakdown.empty();
    for (int i = 1; i < path.size(); i++) {
      int previousIdx = i >= 2 ? index(path.get(i - 2), ny) : -1;
      total =
          total.plus(
              edgeCost(
                  previousIdx,
                  path.get(i - 1),
                  path.get(i),
                  obstacles,
                  robotLengthMeters,
                  robotWidthMeters,
                  fieldLengthMeters,
                  fieldWidthMeters,
                  nx,
                  ny,
                  clearanceField,
                  liveObstacleCostCache));
    }
    return total;
  }

  private CoarseRouteCostBreakdown edgeCost(
      int previousIdx,
      Node current,
      Node next,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny,
      ClearanceField clearanceField,
      double[] liveObstacleCostCache) {
    CoarseRouteCostConfig weights = config.routeCostConfig();
    double step = Math.hypot(next.x - current.x, next.y - current.y);
    Translation2d point = toPoint(next, fieldLengthMeters, fieldWidthMeters, nx, ny);
    double turnPenalty = turnPenalty(previousIdx, current, next, ny);
    double distanceCost = weights.distanceWeight() * step;
    int nextIdx = index(next, ny);
    double liveObstacleCost = liveObstacleCostCache[nextIdx];
    if (Double.isNaN(liveObstacleCost)) {
      liveObstacleCost = obstacleProximityCost(point, obstacles);
      liveObstacleCostCache[nextIdx] = liveObstacleCost;
    }
    double obstacleCost =
        weights.obstacleClearanceWeight()
            * (clearanceField.obstacleProximityCost(next) + liveObstacleCost);
    double wallCost = weights.wallClearanceWeight() * clearanceField.wallProximityCost(next);
    double turnCost = weights.turnWeight() * turnPenalty;
    double corridorCost = weights.corridorPreferenceWeight() * corridorPreferenceCost(point);
    return new CoarseRouteCostBreakdown(
        distanceCost + obstacleCost + wallCost + turnCost + corridorCost,
        distanceCost,
        obstacleCost,
        wallCost,
        turnCost,
        corridorCost);
  }

  private double corridorPreferenceCost(Translation2d point) {
    if (point == null || config.corridorPreferences().isEmpty()) return 0.0;
    double cost = 0.0;
    for (PlannerCorridorPreference preference : config.corridorPreferences()) {
      if (preference != null) cost += preference.costAt(point);
    }
    return cost;
  }

  private CoarseRouteClearanceMetrics routeClearance(
      List<Node> path, ClearanceField clearanceField) {
    if (path == null || path.isEmpty() || clearanceField == null) {
      return CoarseRouteClearanceMetrics.empty();
    }
    double min = Double.POSITIVE_INFINITY;
    double sum = 0.0;
    int count = 0;
    for (Node node : path) {
      double clearance = clearanceField.routeClearanceMeters(node);
      if (!Double.isFinite(clearance)) continue;
      min = Math.min(min, clearance);
      sum += clearance;
      count++;
    }
    if (count == 0) return CoarseRouteClearanceMetrics.empty();
    return new CoarseRouteClearanceMetrics(min, sum / count);
  }

  private List<Node> smoothPath(
      List<Node> path,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny) {
    if (path == null || path.size() <= 2) return path == null ? List.of() : path;
    ArrayList<Node> smoothed = new ArrayList<>();
    int i = 0;
    smoothed.add(path.get(0));
    while (i < path.size() - 1) {
      int best = i + 1;
      for (int j = path.size() - 1; j > i + 1; j--) {
        if (segmentFree(
            path.get(i),
            path.get(j),
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny)) {
          best = j;
          break;
        }
      }
      smoothed.add(path.get(best));
      i = best;
    }
    return smoothed;
  }

  private LookaheadSelection chooseLookahead(
      List<Node> path,
      ClearanceField clearanceField,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLength,
      double fieldWidth,
      int nx,
      int ny,
      Translation2d start) {
    if (path == null || path.size() < 2) {
      return new LookaheadSelection(start, 0, CoarseGlobalPlannerWaypointReason.NONE);
    }

    LookaheadSelection base =
        chooseBaseLookahead(path, clearanceField, fieldLength, fieldWidth, nx, ny, start);
    LookaheadSelection stable =
        maybeKeepPreviousWaypoint(
            base,
            path,
            obstacles,
            robotLengthMeters,
            robotWidthMeters,
            fieldLength,
            fieldWidth,
            nx,
            ny,
            start);
    return stable;
  }

  private LookaheadSelection chooseBaseLookahead(
      List<Node> path,
      ClearanceField clearanceField,
      double fieldLength,
      double fieldWidth,
      int nx,
      int ny,
      Translation2d start) {
    double lookahead = config.waypointLookaheadMeters();
    int fallbackIndex = path.size() - 1;

    for (int i = 1; i < path.size(); i++) {
      Translation2d point = toPoint(path.get(i), fieldLength, fieldWidth, nx, ny);
      double distance = start.getDistance(point);
      if (isSharpTurn(path, i, ny) && distance >= lookahead * MIN_CORNER_LOOKAHEAD_FRACTION) {
        return new LookaheadSelection(
            point, i, CoarseGlobalPlannerWaypointReason.BEFORE_SHARP_TURN);
      }
      if (clearanceField != null
          && i < path.size() - 1
          && clearanceField.routeClearanceMeters(path.get(i)) < NARROW_PASSAGE_CLEARANCE_METERS
          && distance >= lookahead * MIN_CORNER_LOOKAHEAD_FRACTION) {
        return new LookaheadSelection(
            point, i, CoarseGlobalPlannerWaypointReason.BEFORE_NARROW_PASSAGE);
      }
      if (distance >= lookahead) {
        return new LookaheadSelection(
            point, i, CoarseGlobalPlannerWaypointReason.LOOKAHEAD_DISTANCE);
      }
    }

    return new LookaheadSelection(
        toPoint(path.get(fallbackIndex), fieldLength, fieldWidth, nx, ny),
        fallbackIndex,
        CoarseGlobalPlannerWaypointReason.ROUTE_END);
  }

  private LookaheadSelection maybeKeepPreviousWaypoint(
      LookaheadSelection base,
      List<Node> path,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLength,
      double fieldWidth,
      int nx,
      int ny,
      Translation2d start) {
    if (previousWaypoint == null || base == null || start == null) return base;
    if (previousWaypoint.getDistance(base.point()) > WAYPOINT_HYSTERESIS_METERS) return base;
    if (start.getDistance(previousWaypoint) < config.waypointLookaheadMeters() * 0.35) return base;
    if (!pointInsideField(
        previousWaypoint, fieldLength, fieldWidth, robotLengthMeters, robotWidthMeters)) {
      return base;
    }
    Node previousNode = nearestNode(previousWaypoint, nx, ny, fieldLength, fieldWidth);
    if (!segmentFree(
        nearestNode(start, nx, ny, fieldLength, fieldWidth),
        previousNode,
        obstacles,
        robotLengthMeters,
        robotWidthMeters,
        fieldLength,
        fieldWidth,
        nx,
        ny)) {
      return base;
    }
    int routeIndex = nearestPathIndex(path, previousWaypoint, fieldLength, fieldWidth, nx, ny);
    if (routeIndex < 1) return base;
    return new LookaheadSelection(
        previousWaypoint, routeIndex, CoarseGlobalPlannerWaypointReason.HYSTERESIS_KEEP);
  }

  private boolean isSharpTurn(List<Node> path, int index, int ny) {
    if (path == null || index <= 0 || index >= path.size() - 1) return false;
    return turnPenalty(index(path.get(index - 1), ny), path.get(index), path.get(index + 1), ny)
            * Math.PI
        >= SHARP_TURN_RADIANS;
  }

  private int nearestPathIndex(
      List<Node> path, Translation2d point, double fieldLength, double fieldWidth, int nx, int ny) {
    int bestIndex = -1;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < path.size(); i++) {
      double distance = toPoint(path.get(i), fieldLength, fieldWidth, nx, ny).getDistance(point);
      if (distance < bestDistance) {
        bestDistance = distance;
        bestIndex = i;
      }
    }
    return bestDistance <= WAYPOINT_HYSTERESIS_METERS ? bestIndex : -1;
  }

  private List<Node> reconstruct(int goalIdx, int startIdx, int[] parent, int ny) {
    ArrayList<Node> rev = new ArrayList<>();
    int cur = goalIdx;
    while (cur >= 0) {
      rev.add(new Node(cur / ny, cur % ny));
      if (cur == startIdx) break;
      cur = parent[cur];
    }
    Collections.reverse(rev);
    return rev;
  }

  private boolean edgeFree(
      Node a,
      Node b,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny) {
    return segmentFree(
        a,
        b,
        obstacles,
        robotLengthMeters,
        robotWidthMeters,
        fieldLengthMeters,
        fieldWidthMeters,
        nx,
        ny);
  }

  private boolean segmentFree(
      Node a,
      Node b,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny) {
    Translation2d pa = toPoint(a, fieldLengthMeters, fieldWidthMeters, nx, ny);
    Translation2d pb = toPoint(b, fieldLengthMeters, fieldWidthMeters, nx, ny);
    Rotation2d yaw = pb.minus(pa).getAngle();
    int samples = Math.max(2, (int) Math.ceil(Math.hypot(a.x - b.x, a.y - b.y) * 2.0));
    for (int i = 0; i <= samples; i++) {
      double t = i / (double) samples;
      Translation2d p =
          new Translation2d(
              pa.getX() + (pb.getX() - pa.getX()) * t, pa.getY() + (pb.getY() - pa.getY()) * t);
      if (!pointInsideField(
          p, fieldLengthMeters, fieldWidthMeters, robotLengthMeters, robotWidthMeters))
        return false;
      if (rectIntersects(p, yaw, obstacles, robotLengthMeters, robotWidthMeters)) {
        return false;
      }
    }
    return true;
  }

  private boolean isFree(
      Node node,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny) {
    Translation2d p = toPoint(node, fieldLengthMeters, fieldWidthMeters, nx, ny);
    if (!pointInsideField(
        p, fieldLengthMeters, fieldWidthMeters, robotLengthMeters, robotWidthMeters)) {
      return false;
    }
    return !rectIntersects(p, Rotation2d.kZero, obstacles, robotLengthMeters, robotWidthMeters);
  }

  private boolean pointInsideField(
      Translation2d p,
      double fieldLengthMeters,
      double fieldWidthMeters,
      double robotLengthMeters,
      double robotWidthMeters) {
    double halfLength = 0.5 * robotLengthMeters;
    double halfWidth = 0.5 * robotWidthMeters;
    double marginX = Math.max(0.0, halfLength + config.clearanceBufferMeters());
    double marginY = Math.max(0.0, halfWidth + config.clearanceBufferMeters());
    if (p.getX() < marginX || p.getX() > fieldLengthMeters - marginX) return false;
    return !(p.getY() < marginY) && !(p.getY() > fieldWidthMeters - marginY);
  }

  private boolean rectIntersects(
      Translation2d center,
      Rotation2d yaw,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters) {
    Translation2d[] rect =
        FieldPlanner.robotRect(
            center,
            yaw,
            robotLengthMeters + 2.0 * config.clearanceBufferMeters(),
            robotWidthMeters + 2.0 * config.clearanceBufferMeters());
    for (Obstacle obstacle : obstacles) if (obstacle.intersectsRectangle(rect)) return true;
    return false;
  }

  private Node nearestNode(Translation2d p, int nx, int ny, double fieldLength, double fieldWidth) {
    int x = (int) Math.round((p.getX() / Math.max(1e-9, fieldLength)) * (nx - 1));
    int y = (int) Math.round((p.getY() / Math.max(1e-9, fieldWidth)) * (ny - 1));
    return new Node(Math.max(0, Math.min(nx - 1, x)), Math.max(0, Math.min(ny - 1, y)));
  }

  private Translation2d toPoint(Node n, double fieldLength, double fieldWidth, int nx, int ny) {
    double x = n.x * fieldLength / Math.max(1, nx - 1);
    double y = n.y * fieldWidth / Math.max(1, ny - 1);
    return new Translation2d(Math.min(fieldLength, x), Math.min(fieldWidth, y));
  }

  private int index(Node n, int ny) {
    return n.x * ny + n.y;
  }

  private ClearanceField buildClearanceField(
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny) {
    double[] obstacleClearance = new double[nx * ny];
    double[] wallClearance = new double[nx * ny];
    for (int x = 0; x < nx; x++) {
      for (int y = 0; y < ny; y++) {
        Node node = new Node(x, y);
        Translation2d point = toPoint(node, fieldLengthMeters, fieldWidthMeters, nx, ny);
        int idx = index(node, ny);
        obstacleClearance[idx] =
            nearestObstacleClearanceMeters(point, obstacles, robotLengthMeters, robotWidthMeters);
        wallClearance[idx] =
            wallClearanceMeters(
                point, fieldLengthMeters, fieldWidthMeters, robotLengthMeters, robotWidthMeters);
      }
    }
    return new ClearanceField(obstacleClearance, wallClearance, ny);
  }

  private double nearestObstacleClearanceMeters(
      Translation2d point,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters) {
    if (point == null || obstacles == null || obstacles.isEmpty()) return Double.POSITIVE_INFINITY;
    double best = Double.POSITIVE_INFINITY;
    for (Obstacle obstacle : obstacles) {
      double clearance =
          obstacleClearanceMeters(point, obstacle, robotLengthMeters, robotWidthMeters);
      if (Double.isFinite(clearance)) best = Math.min(best, clearance);
    }
    return best;
  }

  private double obstacleClearanceMeters(
      Translation2d point, Obstacle obstacle, double robotLengthMeters, double robotWidthMeters) {
    if (obstacle instanceof PredictedDynamicObstacleEnvelope prediction) {
      return prediction.clearanceMeters(
          point,
          0.5 * robotLengthMeters + config.clearanceBufferMeters(),
          0.5 * robotWidthMeters + config.clearanceBufferMeters());
    }
    if (obstacle instanceof RectangleObstacle rectangle) {
      return rectangleClearanceMeters(point, rectangle, robotLengthMeters, robotWidthMeters);
    }
    return Double.POSITIVE_INFINITY;
  }

  private double rectangleClearanceMeters(
      Translation2d point,
      RectangleObstacle rectangle,
      double robotLengthMeters,
      double robotWidthMeters) {
    double dx = point.getX() - rectangle.center.getX();
    double dy = point.getY() - rectangle.center.getY();
    double cos = rectangle.rot.getCos();
    double sin = rectangle.rot.getSin();
    double localX = dx * cos + dy * sin;
    double localY = -dx * sin + dy * cos;
    double inflatedHalfX =
        rectangle.halfX + 0.5 * robotLengthMeters + config.clearanceBufferMeters();
    double inflatedHalfY =
        rectangle.halfY + 0.5 * robotWidthMeters + config.clearanceBufferMeters();
    double outsideX = Math.abs(localX) - inflatedHalfX;
    double outsideY = Math.abs(localY) - inflatedHalfY;
    double positiveX = Math.max(0.0, outsideX);
    double positiveY = Math.max(0.0, outsideY);
    if (outsideX <= 0.0 && outsideY <= 0.0) {
      return -Math.min(-outsideX, -outsideY);
    }
    return Math.hypot(positiveX, positiveY);
  }

  private double wallClearanceMeters(
      Translation2d point,
      double fieldLengthMeters,
      double fieldWidthMeters,
      double robotLengthMeters,
      double robotWidthMeters) {
    if (point == null) return 0.0;
    double halfLength = 0.5 * robotLengthMeters;
    double halfWidth = 0.5 * robotWidthMeters;
    double marginX = Math.max(0.0, halfLength + config.clearanceBufferMeters());
    double marginY = Math.max(0.0, halfWidth + config.clearanceBufferMeters());
    double clearanceX =
        Math.min(point.getX() - marginX, fieldLengthMeters - marginX - point.getX());
    double clearanceY = Math.min(point.getY() - marginY, fieldWidthMeters - marginY - point.getY());
    return Math.min(clearanceX, clearanceY);
  }

  private double heuristic(Node a, Node b) {
    return Math.max(0.0, config.routeCostConfig().distanceWeight())
        * Math.hypot(a.x - b.x, a.y - b.y);
  }

  private double obstacleProximityCost(Translation2d point, List<? extends Obstacle> obstacles) {
    if (point == null || obstacles == null || obstacles.isEmpty()) return 0.0;
    double cost = 0.0;
    for (Obstacle obstacle : obstacles) {
      if (obstacle == null) continue;
      var force = obstacle.sampleForceAtPosition(point, point);
      if (force != null && Double.isFinite(force.getNorm())) {
        cost += force.getNorm();
      }
    }
    return cost;
  }

  private double wallProximityCost(
      Translation2d point,
      double fieldLengthMeters,
      double fieldWidthMeters,
      double robotLengthMeters,
      double robotWidthMeters) {
    if (point == null) return 0.0;
    double halfLength = 0.5 * robotLengthMeters;
    double halfWidth = 0.5 * robotWidthMeters;
    double marginX = Math.max(0.0, halfLength + config.clearanceBufferMeters());
    double marginY = Math.max(0.0, halfWidth + config.clearanceBufferMeters());
    double clearanceX =
        Math.min(point.getX() - marginX, fieldLengthMeters - marginX - point.getX());
    double clearanceY = Math.min(point.getY() - marginY, fieldWidthMeters - marginY - point.getY());
    double clearance = Math.max(0.0, Math.min(clearanceX, clearanceY));
    return 1.0 / (0.05 + clearance);
  }

  private static double inverseClearanceCost(double clearanceMeters) {
    if (!Double.isFinite(clearanceMeters)) return 0.0;
    return 1.0 / (0.05 + Math.max(0.0, clearanceMeters));
  }

  private double turnPenalty(int previousIdx, Node current, Node next, int ny) {
    if (previousIdx < 0) return 0.0;
    Node previous = new Node(previousIdx / ny, previousIdx % ny);
    double ax = current.x - previous.x;
    double ay = current.y - previous.y;
    double bx = next.x - current.x;
    double by = next.y - current.y;
    double an = Math.hypot(ax, ay);
    double bn = Math.hypot(bx, by);
    if (an <= 1e-9 || bn <= 1e-9) return 0.0;
    double cos = Math.max(-1.0, Math.min(1.0, (ax * bx + ay * by) / (an * bn)));
    return Math.acos(cos) / Math.PI;
  }

  private record Node(int x, int y) {}

  private record Entry(Node node, double g, double f) {}

  private record PartialRouteCandidate(boolean usable, double score) {
    static PartialRouteCandidate unusable() {
      return new PartialRouteCandidate(false, Double.POSITIVE_INFINITY);
    }
  }

  private record LookaheadSelection(
      Translation2d point, int index, CoarseGlobalPlannerWaypointReason reason) {
    private LookaheadSelection {
      if (reason == null) reason = CoarseGlobalPlannerWaypointReason.NONE;
      index = Math.max(0, index);
    }
  }

  private record ClearanceField(double[] obstacleClearance, double[] wallClearance, int ny) {
    double obstacleProximityCost(Node node) {
      return inverseClearanceCost(obstacleClearanceMeters(node));
    }

    double wallProximityCost(Node node) {
      return inverseClearanceCost(wallClearanceMeters(node));
    }

    double routeClearanceMeters(Node node) {
      double obstacle = obstacleClearanceMeters(node);
      double wall = wallClearanceMeters(node);
      if (!Double.isFinite(obstacle)) return Math.max(0.0, wall);
      return Math.max(0.0, Math.min(obstacle, wall));
    }

    private double obstacleClearanceMeters(Node node) {
      return obstacleClearance[index(node)];
    }

    private double wallClearanceMeters(Node node) {
      return wallClearance[index(node)];
    }

    private int index(Node node) {
      return node.x * ny + node.y;
    }
  }
}
