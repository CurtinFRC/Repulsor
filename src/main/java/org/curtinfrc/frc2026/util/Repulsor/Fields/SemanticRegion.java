package org.curtinfrc.frc2026.util.Repulsor.Fields;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/** Declarative field/profile region carrying strategy tags and collect scoring adjustment. */
public record SemanticRegion(
    String id,
    double minXMeters,
    double maxXMeters,
    double minYMeters,
    double maxYMeters,
    List<String> penaltyTags,
    List<String> preferenceTags,
    double collectPenalty,
    double collectPreference) {
  public SemanticRegion {
    if (id == null || id.isBlank()) id = "region";
    double minX = Math.min(minXMeters, maxXMeters);
    double maxX = Math.max(minXMeters, maxXMeters);
    double minY = Math.min(minYMeters, maxYMeters);
    double maxY = Math.max(minYMeters, maxYMeters);
    minXMeters = minX;
    maxXMeters = maxX;
    minYMeters = minY;
    maxYMeters = maxY;
    penaltyTags = penaltyTags == null ? List.of() : List.copyOf(penaltyTags);
    preferenceTags = preferenceTags == null ? List.of() : List.copyOf(preferenceTags);
    collectPenalty = finiteNonNegative(collectPenalty);
    collectPreference = finiteNonNegative(collectPreference);
  }

  public boolean contains(Translation2d point) {
    if (point == null) return false;
    return point.getX() >= minXMeters
        && point.getX() <= maxXMeters
        && point.getY() >= minYMeters
        && point.getY() <= maxYMeters;
  }

  public double collectAdjustment() {
    return collectPreference - collectPenalty;
  }

  private static double finiteNonNegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
