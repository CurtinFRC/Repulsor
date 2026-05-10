package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

/** Diagnostics from the most recent coarse global fallback search. */
public record CoarseGlobalPlannerStats(
    boolean found,
    boolean timedOut,
    boolean exhaustedNodeBudget,
    int expandedNodes,
    int generatedNodes,
    int rawPathNodes,
    int pathNodes,
    CoarseRouteCostBreakdown routeCostBreakdown,
    CoarseRouteClearanceMetrics routeClearanceMetrics,
    long elapsedNanos,
    CoarseGlobalPlannerFailureReason failureReason) {
  public CoarseGlobalPlannerStats(
      boolean found,
      boolean timedOut,
      boolean exhaustedNodeBudget,
      int expandedNodes,
      int generatedNodes,
      int pathNodes,
      long elapsedNanos) {
    this(
        found,
        timedOut,
        exhaustedNodeBudget,
        expandedNodes,
        generatedNodes,
        pathNodes,
        pathNodes,
        CoarseRouteCostBreakdown.empty(),
        CoarseRouteClearanceMetrics.empty(),
        elapsedNanos,
        found ? CoarseGlobalPlannerFailureReason.NONE : CoarseGlobalPlannerFailureReason.NO_ROUTE);
  }

  public CoarseGlobalPlannerStats(
      boolean found,
      boolean timedOut,
      boolean exhaustedNodeBudget,
      int expandedNodes,
      int generatedNodes,
      int rawPathNodes,
      int pathNodes,
      long elapsedNanos) {
    this(
        found,
        timedOut,
        exhaustedNodeBudget,
        expandedNodes,
        generatedNodes,
        rawPathNodes,
        pathNodes,
        CoarseRouteCostBreakdown.empty(),
        CoarseRouteClearanceMetrics.empty(),
        elapsedNanos,
        found ? CoarseGlobalPlannerFailureReason.NONE : CoarseGlobalPlannerFailureReason.NO_ROUTE);
  }

  public CoarseGlobalPlannerStats(
      boolean found,
      boolean timedOut,
      boolean exhaustedNodeBudget,
      int expandedNodes,
      int generatedNodes,
      int rawPathNodes,
      int pathNodes,
      long elapsedNanos,
      CoarseGlobalPlannerFailureReason failureReason) {
    this(
        found,
        timedOut,
        exhaustedNodeBudget,
        expandedNodes,
        generatedNodes,
        rawPathNodes,
        pathNodes,
        CoarseRouteCostBreakdown.empty(),
        CoarseRouteClearanceMetrics.empty(),
        elapsedNanos,
        failureReason);
  }

  public CoarseGlobalPlannerStats {
    rawPathNodes = Math.max(0, rawPathNodes);
    pathNodes = Math.max(0, pathNodes);
    if (routeCostBreakdown == null) routeCostBreakdown = CoarseRouteCostBreakdown.empty();
    if (routeClearanceMetrics == null) routeClearanceMetrics = CoarseRouteClearanceMetrics.empty();
    if (failureReason == null) failureReason = CoarseGlobalPlannerFailureReason.NONE;
  }

  public static CoarseGlobalPlannerStats empty() {
    return new CoarseGlobalPlannerStats(
        false,
        false,
        false,
        0,
        0,
        0,
        0,
        CoarseRouteCostBreakdown.empty(),
        CoarseRouteClearanceMetrics.empty(),
        0L,
        CoarseGlobalPlannerFailureReason.NONE);
  }
}
