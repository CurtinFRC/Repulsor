package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import org.curtinfrc.frc2026.util.Repulsor.Diagnostics.RepulsorDecisionTrace;

/** Result of a full Repulsor planning cycle, including motion command and diagnostics. */
public record RepulsorPlanningResult(
    RepulsorPlanningRequest request,
    RepulsorSample sample,
    RepulsorDiagnosticsSnapshot diagnostics,
    RepulsorDecisionTrace decisionTrace) {
  public RepulsorPlanningResult(
      RepulsorPlanningRequest request,
      RepulsorSample sample,
      RepulsorDiagnosticsSnapshot diagnostics) {
    this(request, sample, diagnostics, RepulsorDecisionTrace.empty());
  }

  public RepulsorPlanningResult {
    if (request == null) request = RepulsorPlanningRequest.from(null);
    if (sample == null)
      sample = new RepulsorSample(request.pose().getTranslation(), 0.0, 0.0, null);
    if (diagnostics == null) diagnostics = RepulsorDiagnosticsSnapshot.empty();
    if (decisionTrace == null) decisionTrace = RepulsorDecisionTrace.empty();
  }

  public static RepulsorPlanningResult empty() {
    RepulsorPlanningRequest request = RepulsorPlanningRequest.from(null);
    return new RepulsorPlanningResult(
        request,
        new RepulsorSample(request.pose().getTranslation(), 0.0, 0.0, null),
        RepulsorDiagnosticsSnapshot.empty());
  }
}
