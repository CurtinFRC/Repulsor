package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.RepulsorStrategyPreset;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultDriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultTurnTuning;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class FieldProfileYamlLoaderTest {
  @TempDir Path tempDir;

  @Test
  void loadsYamlProfileOverridesWithoutTouchingConstants() throws Exception {
    Path profile = tempDir.resolve("custom.yaml");
    Files.writeString(
        profile,
        """
        id: custom
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources:
          cube:
            radiusMeters: 0.2
            unitValue: 2.0
            sigmaMeters: 0.7
        projectileShots: {}
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      FieldProfileConfig cfg =
          FieldProfileYamlLoader.loadOrDefault("custom", new FieldProfileConfig());

      assertEquals("CUSTOM", cfg.gameName);
      assertEquals(2099, cfg.gameYear);
      assertEquals(12.5, cfg.geometry.lengthMeters, 1e-9);
      assertEquals(6.25, cfg.geometry.widthMeters, 1e-9);
      assertTrue(cfg.resources.containsKey("cube"));
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void loadsSchemaVersionAndDeclarativeWaypointRulesFromYaml() throws Exception {
    Path profile = tempDir.resolve("profile-schema-v2.yaml");
    Files.writeString(
        profile,
        """
        schemaVersion: 2
        id: profile-schema-v2
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources: {}
        projectileShots: {}
        waypointing:
          zones:
            loadingSide:
              minXMeters: 0.0
              maxXMeters: 4.0
              minYMeters: 0.0
              maxYMeters: 6.25
            scoringSide:
              minXMeters: 8.0
              maxXMeters: 12.5
              minYMeters: 0.0
              maxYMeters: 6.25
          rules:
            - name: loading-to-score
              objectiveRole: SCORE
              fromZone: loadingSide
              toZone: scoringSide
              candidates:
                - name: center-lane
                  entryXMeters: 6.25
                  entryYMeters: 3.0
                  preference: 0.5
                  forceStage: true
        strategyPresets:
          safeCycle:
            waypointing:
              rules:
                - name: collect-direct
                  objectiveRole: COLLECT
                  fromZone: scoringSide
                  toZone: loadingSide
                  direct: true
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      FieldProfileConfig cfg =
          FieldProfileYamlLoader.loadOrDefault("profile-schema-v2", new FieldProfileConfig());

      assertEquals(2, cfg.schemaVersion);
      assertEquals(2, cfg.waypointing.zones.size());
      assertEquals(1, cfg.waypointing.rules.size());
      assertEquals(
          "loading-to-score",
          cfg.waypointing
                  .toFieldPlannerWaypointPolicyProfile("base")
                  .strategy()
                  .getClass()
                  .getName()
                  .contains("FieldPlannerWaypointRuleStrategy")
              ? cfg.waypointing.rules.get(0).name
              : "missing");
      assertTrue(cfg.toStrategyPresets().containsKey("safeCycle"));
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void rejectsUnsupportedSchemaAndBrokenWaypointReferences() throws Exception {
    Path profile = tempDir.resolve("invalid-schema-v2.yaml");
    Files.writeString(
        profile,
        """
        schemaVersion: 999
        id: invalid-schema-v2
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources: {}
        projectileShots: {}
        waypointing:
          zones:
            loadingSide:
              minXMeters: 4.0
              maxXMeters: 1.0
              minYMeters: 0.0
              maxYMeters: 6.25
          rules:
            - name: bad-rule
              objectiveRole: MYSTERY
              fromZone: missingZone
              toZone: loadingSide
              candidates:
                - name: bad-candidate
                  entryXMeters: 5.0
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      IllegalArgumentException ex =
          assertThrows(
              IllegalArgumentException.class,
              () ->
                  FieldProfileYamlLoader.loadOrDefault(
                      "invalid-schema-v2", new FieldProfileConfig()));
      String message = ex.getMessage();
      assertTrue(message.contains("schemaVersion 999"));
      assertTrue(message.contains("fromZone must reference"));
      assertTrue(message.contains("objectiveRole"));
      assertTrue(message.contains("entryXMeters and entryYMeters"));
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void acceptsLegacyProfilesWithoutSchemaVersion() throws Exception {
    FieldProfileConfig cfg = new FieldProfileConfig();
    cfg.id = "legacy";
    cfg.gameName = "LEGACY";
    cfg.gameYear = 2099;
    cfg.geometry.lengthMeters = 12.5;
    cfg.geometry.widthMeters = 6.25;

    assertFalse(FieldProfileValidator.validate(cfg).contains("schemaVersion must be positive"));
  }

  @Test
  void rejectsInvalidYamlProfileValues() throws Exception {
    Path profile = tempDir.resolve("invalid.yaml");
    Files.writeString(
        profile,
        """
        id: invalid
        gameName: INVALID
        gameYear: 2099
        geometry:
          lengthMeters: -1.0
          widthMeters: 6.25
        resources: {}
        projectileShots: {}
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      assertThrows(
          IllegalArgumentException.class,
          () -> FieldProfileYamlLoader.loadOrDefault("invalid", new FieldProfileConfig()));
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void loadsMovingShotTuningFromYaml() throws Exception {
    Path profile = tempDir.resolve("moving-shot.yaml");
    Files.writeString(
        profile,
        """
        id: moving-shot
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources: {}
        projectileShots:
          pass:
            enabled: true
            role: TRANSFER_TO_SCORE
            gamePieceId: ball
            target:
              kind: alliance_side
              blueXMeters: 3.0
              blueYMeters: 2.0
            targetHeightMeters: 1.0
            constraints:
              minLaunchSpeedMetersPerSecond: 3.0
              maxLaunchSpeedMetersPerSecond: 22.0
              minLaunchAngleDegrees: 10.0
              maxLaunchAngleDegrees: 55.0
              shotStyle: DIRECT
            routeLevel: pass
            routeMechanismSetpoint: NET
            behindTargetMeters: 1.0
            lateralOffsetsMeters: [0.0]
            fieldMarginMeters: 0.25
            movingShot:
              enabled: true
              releaseLatencySeconds: 0.12
              minFlightPredictionSeconds: 0.20
              maxFlightPredictionSeconds: 0.70
              defaultFlightPredictionSeconds: 0.30
              maxCompensatedSpeedMetersPerSecond: 5.0
              maxReleaseSpeedMetersPerSecond: 4.0
              yawToleranceDegrees: 9.0
              maxVerticalErrorMeters: 0.15
              iterations: 4
            fallbackGamePiece:
              massKg: 0.27
              crossSectionAreaM2: 0.014
              dragCoefficient: 0.95
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      FieldProfileConfig cfg =
          FieldProfileYamlLoader.loadOrDefault("moving-shot", new FieldProfileConfig());

      FieldProfileConfig.MovingShotConfig moving = cfg.projectileShots.get("pass").movingShot;
      assertTrue(moving.enabled);
      assertEquals(0.12, moving.releaseLatencySeconds, 1e-9);
      assertEquals(0.70, moving.maxFlightPredictionSeconds, 1e-9);
      assertEquals(4, moving.iterations);
      assertEquals(9.0, moving.solverConfig().yawToleranceDegrees(), 1e-9);
      FieldProfileConfig.ShotConstraintsConfig constraints =
          cfg.projectileShots.get("pass").constraints;
      assertEquals(3.0, constraints.minLaunchSpeedMetersPerSecond, 1e-9);
      assertEquals(22.0, constraints.maxLaunchSpeedMetersPerSecond, 1e-9);
      assertEquals(10.0, constraints.minLaunchAngleDegrees, 1e-9);
      assertEquals(55.0, constraints.maxLaunchAngleDegrees, 1e-9);
      assertEquals("DIRECT", constraints.shotStyle);
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void loadsPredictiveRankingTuningFromYaml() throws Exception {
    Path profile = tempDir.resolve("ranking.yaml");
    Files.writeString(
        profile,
        """
        id: ranking
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources: {}
        projectileShots: {}
        predictiveRanking:
          advantageGain: 2.0
          distanceCost: 0.5
          pressureCost: 0.25
          congestionCost: 0.75
          capacityGain: 1.25
          headingGain: 0.1
          hysteresisBonus: 0.4
          hysteresisPersistSeconds: 1.5
        objectiveSelection:
          candidateLimit: 5
          switchScoreMargin: 0.33
          holdCurrentWhenRanked: false
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      FieldProfileConfig cfg =
          FieldProfileYamlLoader.loadOrDefault("ranking", new FieldProfileConfig());

      assertEquals(2.0, cfg.predictiveRanking.advantageGain, 1e-9);
      assertEquals(0.5, cfg.predictiveRanking.distanceCost, 1e-9);
      assertEquals(1.25, cfg.predictiveRanking.toPredictiveRankingConfig().capacityGain(), 1e-9);
      assertEquals(5, cfg.objectiveSelection.toObjectiveSelectionConfig().candidateLimit());
      assertEquals(
          0.33, cfg.objectiveSelection.toObjectiveSelectionConfig().switchScoreMargin(), 1e-9);
      assertEquals(
          false, cfg.objectiveSelection.toObjectiveSelectionConfig().holdCurrentWhenRanked());

      Rebuilt2026 field = new Rebuilt2026(cfg);
      FieldTrackerCore tracker = new FieldTrackerCore(field);
      assertEquals(5, tracker.objectiveSelectionConfig().candidateLimit());
      assertEquals(0.33, tracker.objectiveSelectionConfig().switchScoreMargin(), 1e-9);
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void loadsWaypointingPolicyFromYamlAndPlannerUsesFieldProfile() throws Exception {
    Path profile = tempDir.resolve("waypointing.yaml");
    Files.writeString(
        profile,
        """
        id: waypointing
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources: {}
        projectileShots: {}
        waypointing:
          bandTransitionStagingEnabled: false
          occludingGateStagingEnabled: true
          centerReturnStagingEnabled: false
          centerBandMeters: 2.2
          restageDistanceMeters: 0.9
          gatePaddingMeters: 0.15
          leadThroughScale: 0.4
          leadThroughMinMeters: 0.6
          leadThroughMaxMeters: 1.4
          deepCenterBandMeters: 1.1
          centerReturnStageTriggerMeters: 2.0
          centerReturnIntersectionTriggerMeters: 3.1
          centerReturnExitMinMeters: 0.5
          centerReturnExitMaxMeters: 1.5
          centerReturnGateMinOffsetMeters: 1.2
          fieldEdgeMarginMeters: 0.2
        plannerRuntime:
          globalFallbackEnabled: false
          globalFallbackCellMeters: 0.42
          globalFallbackLookaheadMeters: 1.7
          globalFallbackMaxExpandedNodes: 321
          globalFallbackMaxRuntimeSeconds: 0.02
          globalFallbackClearanceBufferMeters: 0.23
          forceThroughGoalDistanceMeters: 1.8
          forceThroughWallDistanceMeters: 0.5
        autoPath:
          episodeCooldownSeconds: 0.75
          pinnedFailSeconds: 1.25
          stuckFailSeconds: 2.25
          progressEpsilonMeters: 0.04
          pinnedProgressMinMeters: 0.16
          stuckDistanceMinMeters: 0.55
          successNearDistanceMeters: 0.35
          collectGoalUnits: 4
          shootLockEnterMeters: 2.6
          shootLockExitMeters: 3.2
          shootLockMinRotationDegrees: 7.0
          shootReadyPositionToleranceMeters: 0.22
          shootReadyRotationToleranceDegrees: 6.0
          collectHoldGoalNearMeters: 0.3
          collectFarResourceMinDistanceMeters: 1.4
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      FieldProfileConfig cfg =
          FieldProfileYamlLoader.loadOrDefault("waypointing", new FieldProfileConfig());

      assertEquals(2.2, cfg.waypointing.centerBandMeters, 1e-9);
      assertEquals(
          1.4, cfg.waypointing.toFieldPlannerWaypointConfig().leadThroughMaxMeters(), 1e-9);
      assertEquals(
          false, cfg.waypointing.toFieldPlannerWaypointConfig().bandTransitionStagingEnabled());
      assertEquals(false, cfg.plannerRuntime.toFieldPlannerRuntimeConfig().globalFallbackEnabled());
      assertEquals(
          0.42,
          cfg.plannerRuntime.toFieldPlannerRuntimeConfig().globalFallbackConfig().cellMeters(),
          1e-9);
      assertEquals(
          321,
          cfg.plannerRuntime
              .toFieldPlannerRuntimeConfig()
              .globalFallbackConfig()
              .maxExpandedNodes());
      assertEquals(
          0.23,
          cfg.plannerRuntime
              .toFieldPlannerRuntimeConfig()
              .globalFallbackConfig()
              .clearanceBufferMeters(),
          1e-9);

      Rebuilt2026 field = new Rebuilt2026(cfg);
      FieldPlanner planner =
          new FieldPlanner(new DefaultTurnTuning(), new DefaultDriveTuning(), field);
      assertEquals(2.2, planner.getWaypointConfig().centerBandMeters(), 1e-9);
      assertEquals(false, planner.getWaypointConfig().centerReturnStagingEnabled());
      assertEquals(false, planner.getRuntimeConfig().globalFallbackEnabled());
      assertEquals(
          1.7, planner.getRuntimeConfig().globalFallbackConfig().waypointLookaheadMeters(), 1e-9);
      assertEquals(
          0.23, planner.getRuntimeConfig().globalFallbackConfig().clearanceBufferMeters(), 1e-9);
      assertEquals(1.8, planner.getRuntimeConfig().forceThroughGoalDistanceMeters(), 1e-9);
      assertEquals(0.75, field.autoPathRuntimeConfig().episodeCooldownSeconds(), 1e-9);
      assertEquals(750_000_000L, field.autoPathRuntimeConfig().episodeCooldownNanos());
      assertEquals(4, field.autoPathRuntimeConfig().collectGoalUnits());
      assertEquals(0.22, field.autoPathRuntimeConfig().shootReadyPositionToleranceMeters(), 1e-9);
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void loadsCollectPlannerTuningFromYamlAndTrackerUsesFieldProfile() throws Exception {
    Path profile = tempDir.resolve("collect-planner.yaml");
    Files.writeString(
        profile,
        """
        id: collect-planner
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources:
          fuel:
            radiusMeters: 0.2
            unitValue: 1.0
            sigmaMeters: 0.5
        projectileShots: {}
        collectPlanner:
          groupCellMeters: 0.55
          nearbyRadiusMeters: 2.75
          liveObservationMaxAgeSeconds: 0.45
          predictorObservationMaxAgeSeconds: 0.35
          stickyNoProgressSeconds: 0.65
          switchCooldownSeconds: 0.95
          collectCellMeters: 0.18
          resourceUnitGain: 1.4
          etaCost: 0.7
          hubFrontTrapPenalty: 0.9
          canonicalScoreDropLimit: 0.05
          richerUnitsAbsGain: 0.11
          richerUnitsRelGain: 1.8
          richerEtaDeltaMaxSeconds: 0.35
          richerScoreDropLimit: 0.22
          liveFuelPreferScoreMargin: 0.06
          hubFrontTrapEscapeScoreAllowDrop: 0.25
          nearbyCentroidScoreDropLimit: 0.12
          liveRelockScoreDropLimit: 0.07
          stickyPreferRankedScoreMargin: 0.09
          farSwitchLockDistanceMeters: 3.6
          farSwitchForceMultiplier: 2.5
          closeSwitchEasyDistanceMeters: 1.6
          closeSwitchMarginScale: 0.35
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      FieldProfileConfig cfg =
          FieldProfileYamlLoader.loadOrDefault("collect-planner", new FieldProfileConfig());

      assertEquals(0.55, cfg.collectPlanner.toCollectPlannerTuning().groupCellMeters(), 1e-9);
      assertEquals(
          0.05,
          cfg.collectPlanner.toCollectPlannerTuning().selection().canonicalScoreDropLimit(),
          1e-9);
      assertEquals(
          3.6,
          cfg.collectPlanner.toCollectPlannerTuning().selection().farSwitchLockDistanceMeters(),
          1e-9);

      FieldTrackerCore tracker = new FieldTrackerCore(new Rebuilt2026(cfg));
      assertEquals(0.95, tracker.collectPlannerTuning().switchCooldownSeconds(), 1e-9);
      assertEquals(1.4, tracker.collectPlannerTuning().selection().resourceUnitGain(), 1e-9);
      assertEquals(0.35, tracker.collectPlannerTuning().selection().closeSwitchMarginScale(), 1e-9);
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void loadsStrategyPresetsFromYamlAsHighLevelRuntimeBundles() throws Exception {
    Path profile = tempDir.resolve("strategy-presets.yaml");
    Files.writeString(
        profile,
        """
        id: strategy-presets
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources: {}
        projectileShots: {}
        collectPlanner:
          switchCooldownSeconds: 0.80
          etaCost: 1.20
        defaultStrategyPreset: fastCollect
        strategyPresets:
          fastCollect:
            collectPlanner:
              switchCooldownSeconds: 0.25
              etaCost: 0.60
            predictiveRanking:
              distanceCost: 0.40
            objectiveSelection:
              switchScoreMargin: 0.11
            waypointing:
              bandTransitionStagingEnabled: false
            plannerRuntime:
              globalFallbackEnabled: false
            autoPath:
              collectGoalUnits: 2
          safeCycle:
            collectPlanner:
              switchCooldownSeconds: 1.10
            waypointing:
              occludingGateStagingEnabled: true
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      FieldProfileConfig cfg =
          FieldProfileYamlLoader.loadOrDefault("strategy-presets", new FieldProfileConfig());

      assertEquals("fastCollect", cfg.defaultStrategyPreset);
      assertEquals(2, cfg.toStrategyPresets().size());
      RepulsorStrategyPreset fastCollect = cfg.toStrategyPresets().get("fastCollect");
      assertEquals(0.25, fastCollect.collectPlanner().switchCooldownSeconds(), 1e-9);
      assertEquals(0.60, fastCollect.collectPlanner().selection().etaCost(), 1e-9);
      assertEquals(0.40, fastCollect.predictiveRanking().distanceCost(), 1e-9);
      assertEquals(0.11, fastCollect.objectiveSelection().switchScoreMargin(), 1e-9);
      assertEquals(false, fastCollect.waypointPolicy().config().bandTransitionStagingEnabled());
      assertEquals(false, fastCollect.plannerRuntime().globalFallbackEnabled());
      assertEquals(2, fastCollect.autoPath().collectGoalUnits());

      RepulsorStrategyPreset safeCycle = cfg.toStrategyPresets().get("safeCycle");
      assertEquals(1.10, safeCycle.collectPlanner().switchCooldownSeconds(), 1e-9);
      assertEquals(1.20, safeCycle.collectPlanner().selection().etaCost(), 1e-9);
      assertEquals(true, safeCycle.waypointPolicy().config().occludingGateStagingEnabled());
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }

  @Test
  void loadsSemanticRegionsFromYamlForStrategyScoring() throws Exception {
    Path profile = tempDir.resolve("semantic-regions.yaml");
    Files.writeString(
        profile,
        """
        id: semantic-regions
        gameName: CUSTOM
        gameYear: 2099
        geometry:
          lengthMeters: 12.5
          widthMeters: 6.25
        resources: {}
        projectileShots: {}
        semanticRegions:
          centerContested:
            shape: rectangle
            minXMeters: 4.0
            maxXMeters: 8.0
            minYMeters: 1.0
            maxYMeters: 5.0
            penaltyTags: [contested, risky]
            collectPenalty: 0.75
          loadingLane:
            shape: rectangle
            minXMeters: 0.5
            maxXMeters: 2.0
            minYMeters: 1.0
            maxYMeters: 5.0
            preferenceTags: [safeCollect]
            collectPreference: 0.25
        """);

    String previous = System.getProperty("repulsor.profile.path");
    try {
      System.setProperty("repulsor.profile.path", profile.toString());
      FieldProfileConfig cfg =
          FieldProfileYamlLoader.loadOrDefault("semantic-regions", new FieldProfileConfig());

      assertEquals(2, cfg.toSemanticRegions().size());
      SemanticRegion contested =
          cfg.toSemanticRegions().stream()
              .filter(region -> region.id().equals("centerContested"))
              .findFirst()
              .orElseThrow();
      assertTrue(contested.contains(new edu.wpi.first.math.geometry.Translation2d(6.0, 3.0)));
      assertEquals(-0.75, contested.collectAdjustment(), 1e-9);
      assertTrue(contested.penaltyTags().contains("risky"));
    } finally {
      if (previous == null) {
        System.clearProperty("repulsor.profile.path");
      } else {
        System.setProperty("repulsor.profile.path", previous);
      }
    }
  }
}
