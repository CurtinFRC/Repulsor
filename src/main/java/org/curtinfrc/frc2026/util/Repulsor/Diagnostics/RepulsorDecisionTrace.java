package org.curtinfrc.frc2026.util.Repulsor.Diagnostics;

import java.util.Arrays;
import java.util.List;

/** Immutable list of explainable Repulsor decisions for one planning/selection cycle. */
public record RepulsorDecisionTrace(
    List<RepulsorDecisionEntry> entries, String activeFieldProfile, String activeStrategyPreset) {
  public RepulsorDecisionTrace {
    entries = entries == null ? List.of() : List.copyOf(entries);
    activeFieldProfile = clean(activeFieldProfile);
    activeStrategyPreset = clean(activeStrategyPreset);
  }

  public static RepulsorDecisionTrace empty() {
    return new RepulsorDecisionTrace(List.of(), "", "");
  }

  public static RepulsorDecisionTrace of(RepulsorDecisionEntry... entries) {
    return new RepulsorDecisionTrace(entries == null ? List.of() : Arrays.asList(entries), "", "");
  }

  public boolean hasLayer(String layer) {
    if (layer == null) return false;
    return entries.stream().anyMatch(entry -> entry != null && layer.equals(entry.layer()));
  }

  public String summary() {
    if (entries.isEmpty()) return "empty";
    StringBuilder out = new StringBuilder();
    for (RepulsorDecisionEntry entry : entries) {
      if (entry == null) continue;
      if (out.length() > 0) out.append(" | ");
      out.append(entry.layer()).append(':').append(entry.decision());
      if (!entry.reason().isBlank()) out.append('(').append(entry.reason()).append(')');
    }
    return out.toString();
  }

  private static String clean(String value) {
    return value == null ? "" : value;
  }
}
