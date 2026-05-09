package org.curtinfrc.frc2026.util.Repulsor.Scenario;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Reefscape2025;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadExecutionContext;
import org.junit.jupiter.api.Test;

class RepulsorScenarioLibraryTest {
  @Test
  void commonScenarioFixturesCoverSupportedGames() {
    List<RepulsorPlannerScenario> rebuilt =
        RepulsorScenarioFixtures.commonPlannerScenarios(new Rebuilt2026());
    List<RepulsorPlannerScenario> reefscape =
        RepulsorScenarioFixtures.commonPlannerScenarios(new Reefscape2025());

    assertTrue(rebuilt.stream().anyMatch(scenario -> scenario.name().contains("center-crossing")));
    assertTrue(reefscape.stream().anyMatch(scenario -> scenario.name().contains("edge-of-field")));
    assertEquals(4, rebuilt.size());
    assertEquals(4, reefscape.size());
  }

  @Test
  void blockedScenarioTemplateSeparatesFallbackFromHardBlockedCase() {
    RepulsorPlannerScenarioResult fallback =
        OffloadExecutionContext.runWorker(
            () ->
                RepulsorPlannerScenarioRunner.run(
                    RepulsorPlannerScenarioTemplates.blockedRoute("blocked")));
    RepulsorPlannerScenarioResult hardBlocked =
        OffloadExecutionContext.runWorker(
            () ->
                RepulsorPlannerScenarioRunner.run(
                    RepulsorPlannerScenarioTemplates.blockedWithoutGlobalFallback("hard-blocked")));

    assertTrue(fallback.globalFallbackCycles() > 0);
    assertEquals(0, fallback.pathBlockedCycles());
    assertEquals(0, hardBlocked.globalFallbackCycles());
    assertTrue(hardBlocked.pathBlockedCycles() > 0);
    assertFalse(hardBlocked.madeProgress());
  }

  @Test
  void reportIncludesDecisionAndPlanningSummaryFields() {
    RepulsorPlannerScenarioResult result =
        RepulsorPlannerScenarioRunner.run(
            RepulsorPlannerScenarioTemplates.clearRoute("report-clear"));

    String report = RepulsorScenarioReport.summarize(result);

    assertTrue(report.contains("scenario: report-clear"));
    assertTrue(report.contains("finalErrorMeters:"));
    assertTrue(report.contains("fallbackCycles:"));
    assertTrue(report.contains("waypointStageCycles:"));
    assertTrue(report.contains("reactiveBypassCycles:"));
    assertTrue(report.contains("decisionTrace:"));
  }

  @Test
  void scoringTransitionFixturesIncludeCollectScoreAndStrategyWaypointCases() {
    List<RepulsorPlannerScenario> scenarios =
        RepulsorScenarioFixtures.scoringTransitionScenarios(new Rebuilt2026());

    assertEquals(3, scenarios.size());
    assertTrue(scenarios.stream().anyMatch(scenario -> scenario.name().contains("forced-collect")));
    assertTrue(scenarios.stream().anyMatch(scenario -> scenario.name().contains("forced-score")));
    assertTrue(
        scenarios.stream().anyMatch(scenario -> scenario.name().contains("strategy-waypoint")));
  }
}
