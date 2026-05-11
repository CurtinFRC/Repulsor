package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

/** Explains why the coarse global fallback selected its returned temporary waypoint. */
public enum CoarseGlobalPlannerWaypointReason {
  NONE,
  LOOKAHEAD_DISTANCE,
  BEFORE_SHARP_TURN,
  BEFORE_NARROW_PASSAGE,
  HYSTERESIS_KEEP,
  ROUTE_END
}
