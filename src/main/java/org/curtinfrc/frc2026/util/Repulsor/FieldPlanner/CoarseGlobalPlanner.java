package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.RectangleObstacle;

/**
 * Coarse A* fallback planner used when the local repulsor force field has no direct line to the
 * goal.
 *
 * <p>This intentionally returns only the next waypoint. FieldPlanner still owns smoothing, velocity
 * limiting, heading control, and dynamic bypass.
 */
public final class CoarseGlobalPlanner {
  private final CoarseGlobalPlannerConfig config;
  private CoarseGlobalPlannerStats lastStats = CoarseGlobalPlannerStats.empty();

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
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
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
            robotHalfLengthMeters,
            robotHalfWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny);
    Node s = nearestNode(start, nx, ny, fieldLengthMeters, fieldWidthMeters);
    Node g = nearestNode(goal.getTranslation(), nx, ny, fieldLengthMeters, fieldWidthMeters);
    if (!isFree(
        s,
        obstacles,
        robotHalfLengthMeters,
        robotHalfWidthMeters,
        fieldLengthMeters,
        fieldWidthMeters,
        nx,
        ny)) {
      finishStats(
          false, false, false, 0, 0, 0, startNanos, CoarseGlobalPlannerFailureReason.START_BLOCKED);
      return Optional.empty();
    }
    if (!isFree(
        g,
        obstacles,
        robotHalfLengthMeters,
        robotHalfWidthMeters,
        fieldLengthMeters,
        fieldWidthMeters,
        nx,
        ny)) {
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
      expanded++;
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
              robotHalfLengthMeters,
              robotHalfWidthMeters,
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
              robotHalfLengthMeters,
              robotHalfWidthMeters,
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
                  robotHalfLengthMeters,
                  robotHalfWidthMeters,
                  fieldLengthMeters,
                  fieldWidthMeters,
                  nx,
                  ny,
                  clearanceField);
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
      finishStats(
          false,
          timedOut,
          exhaustedBudget,
          expanded,
          generated,
          0,
          startNanos,
          failureReason(timedOut, exhaustedBudget, CoarseGlobalPlannerFailureReason.NO_ROUTE));
      return Optional.empty();
    }
    List<Node> path = reconstruct(goalIdx, startIdx, parent, ny);
    if (path.size() < 2) {
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
            robotHalfLengthMeters,
            robotHalfWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny);
    CoarseRouteClearanceMetrics routeClearanceMetrics = routeClearance(path, clearanceField);
    CoarseRouteCostBreakdown routeCost =
        routeCost(
            path,
            obstacles,
            robotHalfLengthMeters,
            robotHalfWidthMeters,
            fieldLengthMeters,
            fieldWidthMeters,
            nx,
            ny,
            clearanceField);

    Translation2d waypoint =
        chooseLookahead(smoothedPath, fieldLengthMeters, fieldWidthMeters, nx, ny, start);
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
        startNanos,
        CoarseGlobalPlannerFailureReason.NONE);
    return Optional.of(new Pose2d(waypoint, heading));
  }

  private static CoarseGlobalPlannerFailureReason failureReason(
      boolean timedOut, boolean exhaustedBudget, CoarseGlobalPlannerFailureReason fallback) {
    if (timedOut) return CoarseGlobalPlannerFailureReason.TIMEOUT;
    if (exhaustedBudget) return CoarseGlobalPlannerFailureReason.NODE_BUDGET;
    return fallback;
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
            System.nanoTime() - startNanos,
            failureReason);
  }

  private CoarseRouteCostBreakdown routeCost(
      List<Node> path,
      List<? extends Obstacle> obstacles,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny,
      ClearanceField clearanceField) {
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
                  robotHalfLengthMeters,
                  robotHalfWidthMeters,
                  fieldLengthMeters,
                  fieldWidthMeters,
                  nx,
                  ny,
                  clearanceField));
    }
    return total;
  }

  private CoarseRouteCostBreakdown edgeCost(
      int previousIdx,
      Node current,
      Node next,
      List<? extends Obstacle> obstacles,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny,
      ClearanceField clearanceField) {
    CoarseRouteCostConfig weights = config.routeCostConfig();
    double step = Math.hypot(next.x - current.x, next.y - current.y);
    Translation2d point = toPoint(next, fieldLengthMeters, fieldWidthMeters, nx, ny);
    double turnPenalty = turnPenalty(previousIdx, current, next, ny);
    double distanceCost = weights.distanceWeight() * step;
    double obstacleCost =
        weights.obstacleClearanceWeight()
            * (clearanceField.obstacleProximityCost(next)
                + obstacleProximityCost(point, obstacles));
    double wallCost = weights.wallClearanceWeight() * clearanceField.wallProximityCost(next);
    double turnCost = weights.turnWeight() * turnPenalty;
    return new CoarseRouteCostBreakdown(
        distanceCost + obstacleCost + wallCost + turnCost,
        distanceCost,
        obstacleCost,
        wallCost,
        turnCost);
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
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
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
            robotHalfLengthMeters,
            robotHalfWidthMeters,
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

  private Translation2d chooseLookahead(
      List<Node> path, double fieldLength, double fieldWidth, int nx, int ny, Translation2d start) {
    Translation2d bestPoint = toPoint(path.get(1), fieldLength, fieldWidth, nx, ny);
    for (int i = 1; i < path.size(); i++) {
      Translation2d p = toPoint(path.get(i), fieldLength, fieldWidth, nx, ny);
      bestPoint = p;
      if (start.getDistance(p) >= config.waypointLookaheadMeters()) break;
    }
    return bestPoint;
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
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny) {
    return segmentFree(
        a,
        b,
        obstacles,
        robotHalfLengthMeters,
        robotHalfWidthMeters,
        fieldLengthMeters,
        fieldWidthMeters,
        nx,
        ny);
  }

  private boolean segmentFree(
      Node a,
      Node b,
      List<? extends Obstacle> obstacles,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
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
          p, fieldLengthMeters, fieldWidthMeters, robotHalfLengthMeters, robotHalfWidthMeters))
        return false;
      if (rectIntersects(p, yaw, obstacles, robotHalfLengthMeters, robotHalfWidthMeters)) {
        return false;
      }
    }
    return true;
  }

  private boolean isFree(
      Node node,
      List<? extends Obstacle> obstacles,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      double fieldLengthMeters,
      double fieldWidthMeters,
      int nx,
      int ny) {
    Translation2d p = toPoint(node, fieldLengthMeters, fieldWidthMeters, nx, ny);
    if (!pointInsideField(
        p, fieldLengthMeters, fieldWidthMeters, robotHalfLengthMeters, robotHalfWidthMeters)) {
      return false;
    }
    return !rectIntersects(
        p, Rotation2d.kZero, obstacles, robotHalfLengthMeters, robotHalfWidthMeters);
  }

  private boolean pointInsideField(
      Translation2d p,
      double fieldLengthMeters,
      double fieldWidthMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters) {
    double marginX = Math.max(0.0, robotHalfLengthMeters + config.clearanceBufferMeters());
    double marginY = Math.max(0.0, robotHalfWidthMeters + config.clearanceBufferMeters());
    if (p.getX() < marginX || p.getX() > fieldLengthMeters - marginX) return false;
    return !(p.getY() < marginY) && !(p.getY() > fieldWidthMeters - marginY);
  }

  private boolean rectIntersects(
      Translation2d center,
      Rotation2d yaw,
      List<? extends Obstacle> obstacles,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters) {
    Translation2d[] rect =
        FieldPlanner.robotRect(
            center,
            yaw,
            robotHalfLengthMeters + config.clearanceBufferMeters(),
            robotHalfWidthMeters + config.clearanceBufferMeters());
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
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
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
            nearestObstacleClearanceMeters(
                point, obstacles, robotHalfLengthMeters, robotHalfWidthMeters);
        wallClearance[idx] =
            wallClearanceMeters(
                point,
                fieldLengthMeters,
                fieldWidthMeters,
                robotHalfLengthMeters,
                robotHalfWidthMeters);
      }
    }
    return new ClearanceField(obstacleClearance, wallClearance, ny);
  }

  private double nearestObstacleClearanceMeters(
      Translation2d point,
      List<? extends Obstacle> obstacles,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters) {
    if (point == null || obstacles == null || obstacles.isEmpty()) return Double.POSITIVE_INFINITY;
    double best = Double.POSITIVE_INFINITY;
    for (Obstacle obstacle : obstacles) {
      double clearance =
          obstacleClearanceMeters(point, obstacle, robotHalfLengthMeters, robotHalfWidthMeters);
      if (Double.isFinite(clearance)) best = Math.min(best, clearance);
    }
    return best;
  }

  private double obstacleClearanceMeters(
      Translation2d point,
      Obstacle obstacle,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters) {
    if (!(obstacle instanceof RectangleObstacle rectangle)) return Double.POSITIVE_INFINITY;
    double dx = point.getX() - rectangle.center.getX();
    double dy = point.getY() - rectangle.center.getY();
    double cos = rectangle.rot.getCos();
    double sin = rectangle.rot.getSin();
    double localX = dx * cos + dy * sin;
    double localY = -dx * sin + dy * cos;
    double inflatedHalfX = rectangle.halfX + robotHalfLengthMeters + config.clearanceBufferMeters();
    double inflatedHalfY = rectangle.halfY + robotHalfWidthMeters + config.clearanceBufferMeters();
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
      double robotHalfLengthMeters,
      double robotHalfWidthMeters) {
    if (point == null) return 0.0;
    double marginX = Math.max(0.0, robotHalfLengthMeters + config.clearanceBufferMeters());
    double marginY = Math.max(0.0, robotHalfWidthMeters + config.clearanceBufferMeters());
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
      double robotHalfLengthMeters,
      double robotHalfWidthMeters) {
    if (point == null) return 0.0;
    double marginX = Math.max(0.0, robotHalfLengthMeters + config.clearanceBufferMeters());
    double marginY = Math.max(0.0, robotHalfWidthMeters + config.clearanceBufferMeters());
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
