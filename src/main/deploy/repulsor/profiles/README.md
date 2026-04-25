# Repulsor Field Profile Schema

Field profiles are YAML files loaded from `src/main/deploy/repulsor/profiles`.
They should describe game-specific facts while keeping behaviours and strategy reusable.

## Top Level

```yaml
id: rebuilt2026
gameName: REBUILT
gameYear: 2026

geometry:
  lengthMeters: 16.540988
  widthMeters: 8.211236
```

## Resources

Resources define collectable game pieces for tracking and strategy evaluation.

```yaml
resources:
  fuel:
    radiusMeters: 0.075
    unitValue: 1.0
    sigmaMeters: 0.95
```

## Projectile Shots

Each projectile shot is an action that can be selected by a reasoner or behaviour.
Use roles rather than year-specific names so the same behaviours can work across games.

```yaml
projectileShots:
  fuelScoreHub:
    enabled: true
    role: SCORE
    gamePieceId: fuel
    target:
      kind: hub
    targetHeightMeters: 1.43
    routeLevel: hub
    routeMechanismSetpoint: NET
    behindTargetMeters: 2.95
    lateralOffsetsMeters: [0.0, 0.45, -0.45]
    fieldMarginMeters: 0.28
```

Supported roles include `SCORE`, `TRANSFER_TO_SCORE`, `COLLECT`, `ENDGAME`, and `OTHER`.

Supported target kinds currently include `hub`, `alliance_side`, and field-coordinate fallbacks
using `blueXMeters` and `blueYMeters`.

## Shot Constraints

Constraints are per action. This lets a low transfer pass use a different shot envelope from a
high scoring arc.

```yaml
constraints:
  minLaunchSpeedMetersPerSecond: 0.0
  maxLaunchSpeedMetersPerSecond: 30.0
  minLaunchAngleDegrees: 60.0
  maxLaunchAngleDegrees: 90.0
  shotStyle: ARC
```

`shotStyle` must be `ANY`, `DIRECT`, or `ARC`.

## Moving Shot Tuning

Moving-shot tuning is also per action. A behaviour can keep driving while the solver predicts the
release pose, compensates the target for robot velocity, and decides whether the shot is safe to
release.

```yaml
movingShot:
  enabled: true
  releaseLatencySeconds: 0.08
  minFlightPredictionSeconds: 0.10
  maxFlightPredictionSeconds: 0.45
  defaultFlightPredictionSeconds: 0.18
  maxCompensatedSpeedMetersPerSecond: 4.5
  maxReleaseSpeedMetersPerSecond: 4.5
  yawToleranceDegrees: 13.0
  maxVerticalErrorMeters: 0.20
  iterations: 3
```

Tune `releaseLatencySeconds`, `yawToleranceDegrees`, `maxReleaseSpeedMetersPerSecond`, and
`maxVerticalErrorMeters` first from practice logs.

## Fallback Projectile Physics

Fallback physics are used if the deploy game-piece file cannot be loaded.

```yaml
fallbackGamePiece:
  massKg: 0.27
  crossSectionAreaM2: 0.014
  dragCoefficient: 0.95
```

## Runtime Telemetry

Moving-shot runtime outputs are logged under:

```text
Repulsor/MovingShot/...
Repulsor/Shuttle/...
Repulsor/ShuttleRecovery/...
```

Watch these with `/ShotSpeed`, `/ShotAngle`, and `/ShooterPassthrough` when tuning.
