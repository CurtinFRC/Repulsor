package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

/** Diagnostics from the most recent coarse global fallback search. */
public record CoarseGlobalPlannerStats(
    boolean found,
    boolean timedOut,
    boolean exhaustedNodeBudget,
    int expandedNodes,
    int generatedNodes,
    int pathNodes,
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
        elapsedNanos,
        found ? CoarseGlobalPlannerFailureReason.NONE : CoarseGlobalPlannerFailureReason.NO_ROUTE);
  }

  public CoarseGlobalPlannerStats {
    if (failureReason == null) failureReason = CoarseGlobalPlannerFailureReason.NONE;
  }

  public static CoarseGlobalPlannerStats empty() {
    return new CoarseGlobalPlannerStats(
        false, false, false, 0, 0, 0, 0L, CoarseGlobalPlannerFailureReason.NONE);
  }
}
