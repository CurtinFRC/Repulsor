## Licensing

This repository is licensed under the MIT License (see `LICENSE.md`) **except** for:

- `src/main/org/curtinfrc/frc2026/util/Repulsor`

Repulsor is under the GNU GENERAL PUBLIC LICENSE, but is **copyright (c) 2026 Paul Hodges**
and includes its own license file at:

- `src/main/org/curtinfrc/frc2026/util/Repulsor/LICENSE.md`

## Subprojects

Repulsor is the autonomous planning and field-intelligence subsystem for this robot code.
See [https://github.com/CurtinFRC/Repulsor/blob/main/src/main/java/org/curtinfrc/frc2026/util/Repulsor/README.md](https://github.com/CurtinFRC/Repulsor/blob/main/src/main/java/org/curtinfrc/frc2026/util/Repulsor/README.md) for architecture and usage notes.

Fix the rank() for predicting fallback setpoints
Abstract the goal manager more allowing for more customisable waypointing (in which cases it waypoints and where it waypoints)
