# Repulsor

Repulsor is a field-aware autonomy and utility stack used by the `2026-Rebuilt` robot codebase.
It combines pathing, obstacle avoidance, objective selection, behavior orchestration, tracking, predictive scoring, shooting setpoints, and simulation support in one package.

## Overview

The primary runtime entry point is `Repulsor.java`.
It connects:

- Robot drive IO (`DriveRepulsor`)
- Motion planning (`FieldPlanner`)
- Dynamic obstacle sources (`VisionPlanner`, `RepulsorVision`)
- Behavior scheduling (`BehaviourManager`)
- Match-state reasoning (`Reasoning`)
- Field/objective tracking (`FieldTrackerCore` and `FieldVision`)
- Setpoint and shot selection (`Setpoints`, `Shooting`)

`Repulsor` exposes a command-focused API (`alignTo`, `within`, gate-following helpers) so this module can be dropped directly into WPILib command-based robot code.

## Simulating

Create a Python virtual environment:

```bash
python -m venv .venv
```

Activate it:

```bash
# Windows (PowerShell)
.\.venv\Scripts\Activate.ps1

# Windows (cmd)
.\.venv\Scripts\activate.bat

# macOS / Linux
source .venv/bin/activate
```

Install dependencies:

```bash
python -m pip install -r requirements.txt
```

Run the main simulation (vision + game pieces on-field + basic physics):

```bash
python -m repulsor_sim.main
```

(Optional) Run the live 3D/debug visualiser (robot, cameras, game pieces, etc.):

```bash
python repulsor_3d_sim/run.py
```

Finally, run the WPILib robot simulation from your robot project:

```bash
./gradlew simulateJava
```

## Runtime Flow

At a high level, each cycle:

1. `Repulsor.update()` refreshes field tracking from any configured `FieldVision` feeds.
2. Tracked element state can update next scoring intent (`refreshNextScoreFromFieldTracker`).
3. `VisionPlanner` updates dynamic obstacle snapshots.
4. `BehaviourManager` selects one active behavior using flags from the configured reasoner.
5. Active behavior calls `FieldPlanner.calculate(...)` to produce a `RepulsorSample`.
6. `RepulsorSample` is converted to `ChassisSpeeds` and sent to `DriveRepulsor.runVelocity(...)`.

## Core Modules

- `Repulsor.java`: facade, command builders, goal/phase gate integration.
- `FieldPlanner/`: vector-field planning, wall and obstacle forces, staged goals, fallback rerouting, bypass integration.
- `FieldPlanner/Obstacles`: geometric obstacle primitives and gated attractors.
- `ReactiveBypass/`: short-horizon bypass runtime for local blockage, relatch logic, and pinned-mode handling.
- `ExtraPathing*`: collision tests, clear-path checks, and helper geometry.
- `Setpoints/`: alliance-aware setpoint system and contextual setpoint resolution.
- `Setpoints/Specific/_Rebuilt2026.java`: game-specific setpoints and dynamic hub shot pose solving.
- `Shooting/`: drag-based shot solver, shot library generation, and online refinement.
- `Tracking/`: field model, dynamic object ingestion, objective caches, collect planner runtime.
- `Predictive/`: ranking and scoring of future objective choices from dynamic world state.
- `Vision/` and `VisionPlanner.java`: adapters from vision detections to planner obstacles.
- `Behaviours/`: behavior contracts and concrete modes (`AutoPath`, `Shuttle`, `Defense`, `Test`).
- `Reasoning/`: signal-driven finite-state reasoning for behavior flags.
- `Fields/`: field definitions (`Rebuilt2026`, `Reefscape2025`) including obstacles, heatmaps, and objective layout.
- `Tuning/`: translational and turning tuning strategies.
- `DriverStation/`: NetworkTables-backed control/config layer for Repulsor runtime parameters and commands.
- `Simulation/`, `Metrics/`, `Profiler/`, `Target/`: supporting simulation models, telemetry, profiling, and targeting utilities.

## Creating And Registering Behaviours

Behaviours are now registered from robot code using `Repulsor.addBehaviour(...)` or
`Repulsor.addBehaviours(...)`.
`Repulsor.java` no longer hardcodes a fixed behaviour list.

Typical pattern:

1. Construct `Repulsor`.
2. Build any suppliers/triggers your behaviour needs.
3. Register behaviours.
4. Attach a reasoner with `repulsor.setReasoner(...)`.

Example:

```java
repulsor.addBehaviours(
    new DefenseBehaviour(30, defenseGoalSup, () -> 2.8),
    new ShuttleBehaviour(20, shuttleGoalSup, () -> 3.2),
    new TestBehaviour(),
    new AutoPathBehaviour(
        10,
        repulsor::isInScoringGate,
        repulsor::isInCollectingGate,
        repulsor::getNextScore,
        hpOptionsSup,
        atHPSup,
        hasPieceSup,
        sp -> sp.point().name(),
        () -> 3.5));

repulsor.setReasoner(reasoner);
```

If you need to rebuild the behaviour set at runtime, call `repulsor.clearBehaviours()` and re-add.

## Field And Objective Model

- `FieldModel` contains pure field data: geometry plus AprilTag layout.
- `FieldDefinition` is the game profile on top of that field model: obstacle provider, heatmap provider, objective layout provider, tracker resource configuration, default collect/score setpoints, and optional action profiles.
- `Rebuilt2026` is the default field definition via `Constants.FIELD`; override it with the JVM property `-Drepulsor.field=reefscape2025` or pass a `FieldDefinition` into the `Repulsor` constructor.
- Field-specific tuning should live in a `FieldDefinition`; runtime classes should consume `ctx.repulsor.getFieldDefinition()` instead of importing a specific game class.
- `FieldGeometry` centralizes dimensions, center, bounds checks, diagonal, and margin clamping.
- `FieldActionProfile` is where field/game-specific action capabilities live. The core model exposes generic projectile actions by role, such as `TRANSFER_TO_SCORE`; Rebuilt 2026's mid-field fuel return is one configured action, not a universal Repulsor concept.
- `RepulsorSetpoint` carries both a generic game `levelId` such as `hub`, `reef.l2`, or `coral.station` and the legacy mechanism setpoint needed by current robot mechanisms.
- `FieldMapBuilder` constructs alliance-tagged `GameElement` objectives with category tags:
  - `kScore`
  - `kCollect`
  - `kEndgame`

This allows the planner and predictive layers to ask for mode-specific candidates while reusing one field map.

## YAML Game Profile Tuning

Game-tunable values can be overridden from YAML in `src/main/deploy/repulsor/profiles`.
The default profile lookup is:

1. `-Drepulsor.profile.path=C:\path\to\profile.yaml`
2. `-Drepulsor.profile.dir=C:\path\to\profiles`
3. `src/main/deploy/repulsor/profiles/<profile>.yaml` in the current working directory
4. `<deploy>/repulsor/profiles/<profile>.yaml` on the robot

Current YAML-backed values include:

- `geometry.lengthMeters` and `geometry.widthMeters`
- Collect resource types and their `radiusMeters`, `unitValue`, and `sigmaMeters`
- Projectile action tuning: role, game-piece id, target height, route level, mechanism setpoint, behind-target distance, lateral offsets, field margin, and fallback physics
- Rebuilt corridor obstacle tuning: rectangle dimensions, pull points, bypass strengths, and rail parameters

Use YAML for values that change per game or during tuning.
Keep Java for behavior logic, setpoint functions, and geometric builders that need code.
Each profile is validated as it loads; invalid values fail early instead of silently changing planner behavior.

## Configuration And Control

Repulsor exposes runtime tuning and command channels through `NtRepulsorDriverStation`.
Default keys include:

- `clearance_scale`
- `repulsion_scale`
- `force_controller_override`

Default command channels include pose override, pose reset, and goal setpoint command endpoints under `/Repulsor/DriverStation`.

## FieldPlanner Architecture And Fallback

`FieldPlanner` is intentionally layered:

1. `FieldPlannerGoalManager` resolves the requested goal into the active field goal, including staged gates and attractor waypoints.
2. `ExtraPathing` checks whether the robot rectangle has a clear corridor to the active goal.
3. If clear, the force model combines goal attraction, wall repulsion, field obstacles, and dynamic obstacles into a local step.
4. If blocked, predicted setpoint reroutes are tried first so game-aware alternatives win over geometric-only choices.
5. If still blocked, `CoarseGlobalPlanner` runs a bounded coarse A* search and returns a temporary waypoint for that single calculation.
6. `ReactiveBypass` can still adjust the local target when the force field is valid but short-horizon geometry says a rejoin/bypass is safer.

The global fallback does **not** replace the requested goal or permanently mutate the active goal. It only changes the calculation target for the current `calculate(...)` call, then normal planner state remains owned by `FieldPlannerGoalManager`.

### Global Fallback Tuning

These JVM properties tune the coarse fallback planner:

- `repulsor.fieldplanner.globalFallback.enabled` defaults to `true`.
- `repulsor.fieldplanner.globalFallback.cellMeters` defaults to `0.55`.
- `repulsor.fieldplanner.globalFallback.lookaheadMeters` defaults to `1.4`.
- `repulsor.fieldplanner.globalFallback.maxExpandedNodes` defaults to `1200`.
- `repulsor.fieldplanner.globalFallback.maxRuntimeSeconds` defaults to `0.010`.
- `repulsor.fieldplanner.globalFallback.clearanceBufferMeters` defaults to `0.0`.
- `repulsor.fieldplanner.globalFallback.distanceCostWeight` defaults to `1.0`.
- `repulsor.fieldplanner.globalFallback.obstacleClearanceCostWeight` defaults to `0.0`.
- `repulsor.fieldplanner.globalFallback.wallClearanceCostWeight` defaults to `0.0`.
- `repulsor.fieldplanner.globalFallback.turnCostWeight` defaults to `0.05`.

Smaller cells make paths more precise but increase node count and loop time. Larger lookahead values smooth the next target but can cut too close to obstacles if the cell size is coarse. The clearance buffer inflates the robot footprint and field-edge margin for strategy-specific safe routes. Route cost weights let A* trade off distance, soft obstacle clearance, wall clearance, and turn/kink avoidance before route smoothing runs. The node and runtime limits are guardrails for robot-loop safety; if either trips, the planner stops instead of spending unbounded time searching.

Representative Rebuilt 2026 corridor cases are covered by tests using default-like cell, lookahead, and node budgets. Re-tune these values if the field profile changes obstacle density or corridor width.

### Planner Responsibility Boundary

The planner layers should not overlap responsibilities:

- Objective selectors choose **what** destination is worth pursuing for the current strategy.
- Waypoint policy chooses **strategic staging** waypoints such as lane entries, gates, or profile-defined semantic regions.
- Global fallback chooses a **temporary coarse route** when the active target is geometrically blocked by large static or dynamic obstacles.
- Reactive bypass handles **short-horizon local recovery** around the current route target. It should not encode game strategy, objective selection, or profile staging rules.

When adding a new game or strategy preset, put reusable strategic decisions in profile rules/presets first. Only tune global fallback for route feasibility and clearance, and only tune reactive bypass for local safety/rejoin behavior.

### Global Fallback Telemetry

Planner fallback telemetry is grouped under `Repulsor/GlobalFallback`:

- `Active`: current sample is using a temporary global waypoint.
- `Found`: the last coarse search found a path.
- `TimedOut`: the runtime guardrail stopped search.
- `ExhaustedNodeBudget`: the expanded-node guardrail stopped search.
- `FailureReason`: explainable terminal state such as `START_BLOCKED`, `GOAL_BLOCKED`, `TIMEOUT`, `NODE_BUDGET`, or `NO_ROUTE`.
- `ExpandedNodes`, `GeneratedNodes`, `RawPathNodes`, `PathNodes`: search size and path complexity before/after smoothing.
- `RouteTotalCost`, `RouteDistanceCost`, `RouteObstacleClearanceCost`, `RouteWallClearanceCost`, `RouteTurnCost`: weighted cost breakdown for the raw selected route.
- `MinRouteClearanceMeters`, `AverageRouteClearanceMeters`: clearance-field diagnostics for the selected raw route, including field-wall and rectangular obstacle margins.
- `SelectedWaypointIndex`, `SelectedWaypointReason`: selected smoothed-route waypoint index and why it was chosen, such as lookahead distance, before a sharp turn, before a narrow passage, or hysteresis keep.
- `ElapsedMs`: elapsed coarse planner time.
- `Waypoint`: temporary waypoint selected for the current sample, or zero pose when inactive.

Use these together when tuning. A healthy robot loop should show occasional `Active=true` during blocked paths, low `ElapsedMs`, and no persistent timeout or node-budget exhaustion.

### Offload Boundary

`FieldPlanner.calculate(...)` may run through the offload entrypoint when enabled. The offload path receives the requested/active goal, dynamic obstacles, category, alliance preference, and shooter height, then returns both the sample and the resulting active goal/error state. Regression tests cover parity for clear paths, dynamic obstacles, and global-fallback temporary waypoint cases.

## Integration Notes

Typical wiring pattern:

```java
RepulsorDriverStationBootstrap.useDefaultNt();

Repulsor repulsor =
    new Repulsor(driveRepulsor, robotX, robotY, coralOffset, algaeOffset, hasPieceSupplier)
        .withVision(myVisionSource)
        .withShooterReleaseHeightMetersSupplier(shooterReleaseHeightMetersSupplier);
```

Then:

- Register behaviours through `repulsor.addBehaviour(...)` or `repulsor.addBehaviours(...)`.
- Set the reasoner via `repulsor.setReasoner(...)`.
- Call `repulsor.update()` during periodic execution.
- Use `repulsor.alignTo(...)` when you need direct command-based alignment to a setpoint.
- Use `repulsor.within(...)` triggers for arrival checks.

## Future

There are lots of things to be added.

## Licence

Repulsor is licensed under the GNU GPLv3.

Copyright (c) 2026 Paul Hodges

See `LICENCE.md` for the full licence text.

## Acknowledgements

Repulsor's earliest vector-field prototype was inspired by a widely shared community approach used across many FRC codebases.
The current implementation has since been rewritten and expanded substantially.
