package org.curtinfrc.frc2026.util.Repulsor.Scenario;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective.ObjectiveSelectionConfig;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective.ObjectiveSelectionDecision;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Objective.ObjectiveSelector;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

/** Repeatable scenario wrapper for strategy-level predictive objective selection. */
public record RepulsorObjectiveSelectionScenario(
    String name,
    List<Candidate> candidates,
    RepulsorSetpoint currentObjective,
    ObjectiveSelectionConfig config) {
  public RepulsorObjectiveSelectionScenario {
    if (name == null || name.isBlank()) name = "objective-selection";
    candidates = candidates == null ? List.of() : List.copyOf(candidates);
    config = config == null ? ObjectiveSelectionConfig.defaults() : config;
  }

  public ObjectiveSelectionDecision run() {
    return ObjectiveSelector.select(candidates, currentObjective, config);
  }

  public static Candidate candidate(RepulsorSetpoint setpoint, double score) {
    return new Candidate(setpoint, new Translation2d(score, 0.0), 0.0, 0.0, 0.0, 0.0, 0.0, score);
  }
}
