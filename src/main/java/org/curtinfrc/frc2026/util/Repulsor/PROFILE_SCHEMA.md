# Repulsor Field Profile Schema

This guide documents the YAML profile shape supported by `FieldProfileConfig`.
Profiles are field-relative and use meters, seconds, degrees, and enum names unless a field says otherwise.

## Compatibility

```yaml
schemaVersion: 2
id: rebuilt2026
```

- `schemaVersion` is optional. Missing versions are treated as legacy schema 1.
- The current schema is `2`.
- Newer schema versions are rejected so a robot does not silently run an unsupported profile.

## Required top-level fields

```yaml
id: custom-field
gameName: CUSTOM
gameYear: 2099
geometry:
  lengthMeters: 16.54
  widthMeters: 8.21
```

`geometry.lengthMeters` and `geometry.widthMeters` must be finite and positive.

## Resources

```yaml
resources:
  fuel:
    radiusMeters: 0.18
    unitValue: 1.0
    sigmaMeters: 0.45
```

Resource keys are game-specific. Radius, unit value, and sigma must be positive when provided.

## Semantic regions

Semantic regions give strategy layers a shared language for penalties and preferences without hardcoding game names.

```yaml
semanticRegions:
  centerContested:
    shape: rectangle
    minXMeters: 6.0
    maxXMeters: 10.0
    minYMeters: 2.0
    maxYMeters: 6.0
    penaltyTags: [traffic, defense]
    preferenceTags: []
    collectPenalty: 0.75
    collectPreference: 0.0
```

Only rectangular semantic regions are currently supported. Bounds and weights must be non-negative.

## Waypointing zones and rules

Declarative waypointing lets a game profile describe when to stage and where to stage without changing Java code.
This layer is for strategic waypoint/staging policy. Reactive bypass remains separate and handles immediate local obstacle avoidance.

```yaml
waypointing:
  zones:
    loadingSide:
      minXMeters: 0.0
      maxXMeters: 5.0
      minYMeters: 0.0
      maxYMeters: 8.21
    scoringSide:
      minXMeters: 11.5
      maxXMeters: 16.54
      minYMeters: 0.0
      maxYMeters: 8.21

  rules:
    - name: loading-to-score-safe-lane
      objectiveRole: SCORE
      fromZone: loadingSide
      toZone: scoringSide
      onlyWhenNotAlreadyStaging: true
      direct: false
      scoring:
        distanceCost: 1.0
        goalAlignmentGain: 0.35
        obstacleClearanceGain: 0.2
        preferenceGain: 1.0
      candidates:
        - name: center-lane-entry
          entryXMeters: 8.2
          entryYMeters: 4.1
          preference: 0.4
          forceStage: true
```

Validation catches:

- missing rule names
- unknown objective roles
- rules referencing missing zones
- non-direct rules without candidates
- candidate entries missing X/Y coordinates
- negative zone or scoring values

## Strategy presets

Presets bundle behavior for autonomous modes or driver strategy switches.
They inherit base profile values, then apply overrides.

```yaml
defaultStrategyPreset: fastCollect
strategyPresets:
  fastCollect:
    collectPlanner:
      switchCooldownSeconds: 0.25
    waypointing:
      rules:
        - name: collect-shortcut
          objectiveRole: COLLECT
          direct: true
```

Preset waypoint rules can reference zones declared in the base `waypointing.zones` map.

## Planner runtime

`plannerRuntime` controls bounded geometric fallback behavior. These values can live in the base profile or inside strategy presets when a mode needs safer or more aggressive routing.

```yaml
plannerRuntime:
  globalFallbackEnabled: true
  globalFallbackCellMeters: 0.55
  globalFallbackLookaheadMeters: 1.4
  globalFallbackMaxExpandedNodes: 1200
  globalFallbackMaxRuntimeSeconds: 0.010
  globalFallbackClearanceBufferMeters: 0.10
  globalFallbackDistanceCostWeight: 1.0
  globalFallbackObstacleClearanceCostWeight: 0.0
  globalFallbackWallClearanceCostWeight: 0.0
  globalFallbackTurnCostWeight: 0.05
  globalFallbackCorridorPreferenceCostWeight: 0.0
```

`globalFallbackClearanceBufferMeters` adds extra margin around the robot and field edge for the coarse route only. Use it to make a strategy preset avoid tight corridors without changing waypoint staging or reactive bypass behavior.
`globalFallbackDistanceCostWeight`, `globalFallbackObstacleClearanceCostWeight`, and `globalFallbackWallClearanceCostWeight` let a profile prefer shorter routes, routes with softer obstacle clearance, or routes farther from field edges.
`globalFallbackTurnCostWeight` adds a soft cost for route kinks before smoothing, which helps prefer cleaner coarse paths when multiple routes are otherwise similar.
`globalFallbackCorridorPreferenceCostWeight` scales soft route costs created from `semanticRegions`: regions with `penaltyTags` or `collectPenalty` become avoided corridors, while regions with `preferenceTags` or `collectPreference` become preferred corridors. This only biases coarse fallback routing; waypoint policy still owns strategic staging and reactive bypass still owns local obstacle recovery.

## Common mistakes

- Using feet or inches instead of meters.
- Adding a waypoint rule that references a missing zone.
- Setting `schemaVersion` higher than the code supports.
- Using an objective role string that is not a `FieldPlannerWaypointObjectiveRole` enum value.
- Putting reactive obstacle avoidance behavior into waypointing rules. Use waypointing for strategy staging, not local collision recovery.
