package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

/** Explainable terminal reason for the most recent coarse global fallback search. */
public enum CoarseGlobalPlannerFailureReason {
  NONE,
  INVALID_INPUT,
  START_BLOCKED,
  GOAL_BLOCKED,
  TIMEOUT,
  NODE_BUDGET,
  NO_ROUTE,
  PARTIAL_ROUTE_USED,
  PARTIAL_ROUTE_REJECTED_UNSAFE,
  PATH_TOO_SHORT
}
