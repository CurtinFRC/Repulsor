package org.curtinfrc.frc2026.util.Repulsor.Diagnostics;

import java.util.Map;

/** One explainable decision made by a Repulsor planning, objective, or selection layer. */
public record RepulsorDecisionEntry(
    String layer,
    String decision,
    String reason,
    String selected,
    String previous,
    double score,
    double scoreDelta,
    Map<String, String> metadata) {
  public RepulsorDecisionEntry {
    layer = clean(layer, "Unknown");
    decision = clean(decision, "none");
    reason = clean(reason, decision);
    selected = clean(selected, "");
    previous = clean(previous, "");
    if (!Double.isFinite(score)) score = 0.0;
    if (!Double.isFinite(scoreDelta)) scoreDelta = 0.0;
    metadata = metadata == null ? Map.of() : Map.copyOf(metadata);
  }

  public static RepulsorDecisionEntry of(String layer, String decision, String reason) {
    return new RepulsorDecisionEntry(layer, decision, reason, "", "", 0.0, 0.0, Map.of());
  }

  public RepulsorDecisionEntry withSelection(String selected, String previous) {
    return new RepulsorDecisionEntry(
        layer, decision, reason, selected, previous, score, scoreDelta, metadata);
  }

  public RepulsorDecisionEntry withScore(double score, double scoreDelta) {
    return new RepulsorDecisionEntry(
        layer, decision, reason, selected, previous, score, scoreDelta, metadata);
  }

  public RepulsorDecisionEntry withMetadata(Map<String, String> metadata) {
    return new RepulsorDecisionEntry(
        layer, decision, reason, selected, previous, score, scoreDelta, metadata);
  }

  private static String clean(String value, String fallback) {
    return value == null || value.isBlank() ? fallback : value;
  }
}
