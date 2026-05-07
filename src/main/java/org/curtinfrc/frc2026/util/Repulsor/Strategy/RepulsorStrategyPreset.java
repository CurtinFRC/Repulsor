package org.curtinfrc.frc2026.util.Repulsor.Strategy;

import org.curtinfrc.frc2026.util.Repulsor.Behaviours.AutoPathRuntimeConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlannerRuntimeConfig;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerWaypointPolicyProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PredictiveRankingConfig;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective.ObjectiveSelectionConfig;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.CollectPlannerTuning;

/** Immutable high-level Repulsor strategy bundle that can be applied atomically at runtime. */
public record RepulsorStrategyPreset(
    String name,
    CollectPlannerTuning collectPlanner,
    PredictiveRankingConfig predictiveRanking,
    ObjectiveSelectionConfig objectiveSelection,
    FieldPlannerRuntimeConfig plannerRuntime,
    FieldPlannerWaypointPolicyProfile waypointPolicy,
    AutoPathRuntimeConfig autoPath) {
  public RepulsorStrategyPreset {
    if (name == null || name.isBlank()) name = "default";
    if (collectPlanner == null) collectPlanner = CollectPlannerTuning.defaults();
    if (predictiveRanking == null) predictiveRanking = PredictiveRankingConfig.defaults();
    if (objectiveSelection == null) objectiveSelection = ObjectiveSelectionConfig.defaults();
    if (plannerRuntime == null) plannerRuntime = FieldPlannerRuntimeConfig.defaults();
    if (waypointPolicy == null)
      waypointPolicy = new FieldPlannerWaypointPolicyProfile(name, null, null);
    if (autoPath == null) autoPath = AutoPathRuntimeConfig.defaults();
  }
}
