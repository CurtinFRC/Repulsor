package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Translation2d;

/** Named rectangular field zone used by rule-based waypoint policies. */
public record FieldPlannerWaypointZone(
    String name, double minX, double maxX, double minY, double maxY) {
  public FieldPlannerWaypointZone {
    if (name == null || name.isBlank()) name = "zone";
    double loX = Math.min(minX, maxX);
    double hiX = Math.max(minX, maxX);
    double loY = Math.min(minY, maxY);
    double hiY = Math.max(minY, maxY);
    minX = loX;
    maxX = hiX;
    minY = loY;
    maxY = hiY;
  }

  public static FieldPlannerWaypointZone wholeField(double lengthMeters, double widthMeters) {
    return new FieldPlannerWaypointZone("field", 0.0, lengthMeters, 0.0, widthMeters);
  }

  public boolean contains(Translation2d point) {
    if (point == null) return false;
    return point.getX() >= minX
        && point.getX() <= maxX
        && point.getY() >= minY
        && point.getY() <= maxY;
  }
}
