package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;

/** Semantic objective role used by waypoint policies without coupling them to a specific game. */
public enum FieldPlannerWaypointObjectiveRole {
  ANY,
  COLLECT,
  SCORE,
  OTHER;

  public static FieldPlannerWaypointObjectiveRole fromCategory(CategorySpec category) {
    if (category == null) return ANY;
    return switch (category) {
      case kCollect -> COLLECT;
      case kScore -> SCORE;
      default -> OTHER;
    };
  }

  public boolean matches(FieldPlannerWaypointObjectiveRole actual) {
    if (this == ANY) return true;
    return actual != null && actual != ANY && this == actual;
  }
}
