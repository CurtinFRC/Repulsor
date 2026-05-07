package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Fields.SemanticRegion;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.junit.jupiter.api.Test;

class FieldTrackerCollectObjectiveLoopTest {
  @Test
  void countLiveCollectResourcesWithinIgnoresStaleObservations() {
    FieldTrackerCollectObjectiveLoop loop =
        new FieldTrackerCollectObjectiveLoop(
            null,
            () -> new Translation2d[] {new Translation2d(1.0, 2.0)},
            List::of,
            type -> "fuel".equalsIgnoreCase(type));

    Translation2d center = new Translation2d(1.0, 2.0);
    List<DynamicObject> dyn =
        List.of(
            new DynamicObject("fresh", "fuel", center, new Translation2d(), 0.10),
            new DynamicObject(
                "stale",
                "fuel",
                new Translation2d(1.02, 2.0),
                new Translation2d(),
                FieldTrackerCollectObjectiveLoop.COLLECT_LIVE_OBS_MAX_AGE_S + 0.20),
            new DynamicObject("nonfuel", "other", center, new Translation2d(), 0.05));

    int n = loop.countLiveCollectResourcesWithin(dyn, center, 0.10);
    assertEquals(1, n);
  }

  @Test
  void filterDynamicsForCollectPredictorDropsOnlyStaleCollectObservations() {
    FieldTrackerCollectObjectiveLoop loop =
        new FieldTrackerCollectObjectiveLoop(
            null,
            () -> new Translation2d[] {new Translation2d(1.0, 2.0)},
            List::of,
            type -> "fuel".equalsIgnoreCase(type));

    DynamicObject staleFuel =
        new DynamicObject(
            "stale",
            "fuel",
            new Translation2d(1.0, 2.0),
            new Translation2d(),
            FieldTrackerCollectObjectiveLoop.COLLECT_PREDICTOR_OBS_MAX_AGE_S + 0.20);
    DynamicObject freshFuel =
        new DynamicObject("fresh", "fuel", new Translation2d(1.1, 2.0), new Translation2d(), 0.10);
    DynamicObject staleOther =
        new DynamicObject(
            "other",
            "robot",
            new Translation2d(2.0, 2.0),
            new Translation2d(),
            FieldTrackerCollectObjectiveLoop.COLLECT_PREDICTOR_OBS_MAX_AGE_S + 0.40);

    List<DynamicObject> filtered =
        loop.filterDynamicsForCollectPredictor(List.of(staleFuel, freshFuel, staleOther));

    assertEquals(2, filtered.size());
    assertTrue(filtered.contains(freshFuel));
    assertTrue(filtered.contains(staleOther));
  }

  @Test
  void semanticRegionBreakdownAppliesProfileDrivenCollectAdjustments() {
    FieldTrackerCollectObjectiveLoop loop =
        new FieldTrackerCollectObjectiveLoop(
            null,
            () -> new Translation2d[] {new Translation2d(1.0, 2.0)},
            List::of,
            type -> "fuel".equalsIgnoreCase(type));

    loop.configureSemanticRegions(
        List.of(
            new SemanticRegion(
                "contested", 1.0, 3.0, 1.0, 3.0, List.of("risky"), List.of(), 0.8, 0.0),
            new SemanticRegion(
                "safeLane", 2.0, 4.0, 1.0, 3.0, List.of(), List.of("safeCollect"), 0.0, 0.3)));

    var insideBoth = loop.semanticRegionBreakdown(new Translation2d(2.5, 2.0));
    var outside = loop.semanticRegionBreakdown(new Translation2d(6.0, 2.0));

    assertEquals(-0.5, insideBoth.total(), 1e-9);
    assertEquals(0.0, outside.total(), 1e-9);
    assertTrue(insideBoth.term("regionPenalty:contested").isPresent());
    assertTrue(insideBoth.term("regionPreference:safeLane").isPresent());
  }
}
