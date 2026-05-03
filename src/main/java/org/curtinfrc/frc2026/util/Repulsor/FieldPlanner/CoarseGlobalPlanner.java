package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;

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
      finishStats(false, false, false, 0, 0, 0, startNanos);
      return Optional.empty();
    }

    int nx = Math.max(2, (int) Math.ceil(fieldLengthMeters / config.cellMeters()) + 1);
    int ny = Math.max(2, (int) Math.ceil(fieldWidthMeters / config.cellMeters()) + 1);
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
      finishStats(false, false, false, 0, 0, 0, startNanos);
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
      finishStats(false, false, false, 0, 0, 0, startNanos);
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

          double step = (dx != 0 && dy != 0) ? Math.sqrt(2.0) : 1.0;
          double tentative = best[curIdx] + step;
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
      finishStats(false, timedOut, exhaustedBudget, expanded, generated, 0, startNanos);
      return Optional.empty();
    }
    List<Node> path = reconstruct(goalIdx, startIdx, parent, ny);
    if (path.size() < 2) {
      finishStats(false, timedOut, exhaustedBudget, expanded, generated, path.size(), startNanos);
      return Optional.empty();
    }

    Translation2d waypoint =
        chooseLookahead(path, fieldLengthMeters, fieldWidthMeters, nx, ny, start);
    Rotation2d heading = goal.getTranslation().minus(waypoint).getAngle();
    finishStats(true, timedOut, exhaustedBudget, expanded, generated, path.size(), startNanos);
    return Optional.of(new Pose2d(waypoint, heading));
  }

  private void finishStats(
      boolean found,
      boolean timedOut,
      boolean exhaustedBudget,
      int expanded,
      int generated,
      int pathNodes,
      long startNanos) {
    lastStats =
        new CoarseGlobalPlannerStats(
            found,
            timedOut,
            exhaustedBudget,
            expanded,
            generated,
            pathNodes,
            System.nanoTime() - startNanos);
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
    Translation2d pa = toPoint(a, fieldLengthMeters, fieldWidthMeters, nx, ny);
    Translation2d pb = toPoint(b, fieldLengthMeters, fieldWidthMeters, nx, ny);
    Rotation2d yaw = pb.minus(pa).getAngle();
    for (int i = 0; i <= 2; i++) {
      double t = i / 2.0;
      Translation2d p =
          new Translation2d(
              pa.getX() + (pb.getX() - pa.getX()) * t, pa.getY() + (pb.getY() - pa.getY()) * t);
      if (rectIntersects(p, yaw, obstacles, robotHalfLengthMeters, robotHalfWidthMeters))
        return false;
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
    double marginX = Math.max(0.0, robotHalfLengthMeters);
    double marginY = Math.max(0.0, robotHalfWidthMeters);
    if (p.getX() < marginX || p.getX() > fieldLengthMeters - marginX) return false;
    if (p.getY() < marginY || p.getY() > fieldWidthMeters - marginY) return false;
    return !rectIntersects(
        p, Rotation2d.kZero, obstacles, robotHalfLengthMeters, robotHalfWidthMeters);
  }

  private boolean rectIntersects(
      Translation2d center,
      Rotation2d yaw,
      List<? extends Obstacle> obstacles,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters) {
    Translation2d[] rect =
        FieldPlanner.robotRect(center, yaw, robotHalfLengthMeters, robotHalfWidthMeters);
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

  private double heuristic(Node a, Node b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
  }

  private record Node(int x, int y) {}

  private record Entry(Node node, double g, double f) {}
}
