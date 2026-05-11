package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Fields.SemanticRegion;

/** Soft route preference shared by strategic profiles and geometric fallback routing. */
public record PlannerCorridorPreference(
    String id,
    double minXMeters,
    double maxXMeters,
    double minYMeters,
    double maxYMeters,
    Kind kind,
    double weight) {
  public enum Kind {
    PREFERRED,
    AVOID
  }

  public PlannerCorridorPreference {
    if (id == null || id.isBlank()) id = "corridor";
    double minX = Math.min(minXMeters, maxXMeters);
    double maxX = Math.max(minXMeters, maxXMeters);
    double minY = Math.min(minYMeters, maxYMeters);
    double maxY = Math.max(minYMeters, maxYMeters);
    minXMeters = minX;
    maxXMeters = maxX;
    minYMeters = minY;
    maxYMeters = maxY;
    if (kind == null) kind = Kind.AVOID;
    weight = Double.isFinite(weight) ? Math.max(0.0, weight) : 0.0;
  }

  public static PlannerCorridorPreference preferred(
      String id,
      double minXMeters,
      double maxXMeters,
      double minYMeters,
      double maxYMeters,
      double weight) {
    return new PlannerCorridorPreference(
        id, minXMeters, maxXMeters, minYMeters, maxYMeters, Kind.PREFERRED, weight);
  }

  public static PlannerCorridorPreference avoided(
      String id,
      double minXMeters,
      double maxXMeters,
      double minYMeters,
      double maxYMeters,
      double weight) {
    return new PlannerCorridorPreference(
        id, minXMeters, maxXMeters, minYMeters, maxYMeters, Kind.AVOID, weight);
  }

  public boolean contains(Translation2d point) {
    if (point == null) return false;
    return point.getX() >= minXMeters
        && point.getX() <= maxXMeters
        && point.getY() >= minYMeters
        && point.getY() <= maxYMeters;
  }

  /** Returns an additive non-negative cost for sampling this route point. */
  public double costAt(Translation2d point) {
    if (weight <= 0.0) return 0.0;
    boolean inside = contains(point);
    return switch (kind) {
      case AVOID -> inside ? weight : 0.0;
      case PREFERRED -> inside ? 0.0 : weight;
    };
  }

  public static List<PlannerCorridorPreference> fromSemanticRegions(List<SemanticRegion> regions) {
    if (regions == null || regions.isEmpty()) return List.of();
    ArrayList<PlannerCorridorPreference> preferences = new ArrayList<>();
    for (SemanticRegion region : regions) {
      if (region == null) continue;
      double avoidWeight =
          region.collectPenalty() > 0.0
              ? region.collectPenalty()
              : (region.penaltyTags().isEmpty() ? 0.0 : 1.0);
      double preferWeight =
          region.collectPreference() > 0.0
              ? region.collectPreference()
              : (region.preferenceTags().isEmpty() ? 0.0 : 1.0);
      if (avoidWeight > 0.0) {
        preferences.add(
            avoided(
                region.id(),
                region.minXMeters(),
                region.maxXMeters(),
                region.minYMeters(),
                region.maxYMeters(),
                avoidWeight));
      }
      if (preferWeight > 0.0) {
        preferences.add(
            preferred(
                region.id(),
                region.minXMeters(),
                region.maxXMeters(),
                region.minYMeters(),
                region.maxYMeters(),
                preferWeight));
      }
    }
    return List.copyOf(preferences);
  }
}
