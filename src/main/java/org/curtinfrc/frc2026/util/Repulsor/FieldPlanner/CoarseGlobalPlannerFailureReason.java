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
  PATH_TOO_SHORT
}
