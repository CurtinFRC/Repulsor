package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

/** Result of a full Repulsor planning cycle, including motion command and diagnostics. */
public record RepulsorPlanningResult(
    RepulsorPlanningRequest request,
    RepulsorSample sample,
    RepulsorDiagnosticsSnapshot diagnostics) {
  public RepulsorPlanningResult {
    if (request == null) request = RepulsorPlanningRequest.from(null);
    if (sample == null)
      sample = new RepulsorSample(request.pose().getTranslation(), 0.0, 0.0, null);
    if (diagnostics == null) diagnostics = RepulsorDiagnosticsSnapshot.empty();
  }

  public static RepulsorPlanningResult empty() {
    RepulsorPlanningRequest request = RepulsorPlanningRequest.from(null);
    return new RepulsorPlanningResult(
        request,
        new RepulsorSample(request.pose().getTranslation(), 0.0, 0.0, null),
        RepulsorDiagnosticsSnapshot.empty());
  }
}
