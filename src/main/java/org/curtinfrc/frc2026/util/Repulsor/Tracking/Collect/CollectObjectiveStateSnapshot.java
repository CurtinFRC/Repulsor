package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import edu.wpi.first.math.geometry.Translation2d;

/** Immutable view of the collect objective loop's sticky/drive state. */
public record CollectObjectiveStateSnapshot(
    Translation2d stickyDriveTarget,
    int stickySide,
    int stickyHalfLock,
    Translation2d driveLastTarget,
    Translation2d forcedDriveCandidate,
    double noFuelSeconds,
    double stuckSeconds,
    double emptyDriveSeconds) {
  static CollectObjectiveStateSnapshot from(FieldTrackerCollectObjectiveLoop loop) {
    return new CollectObjectiveStateSnapshot(
        loop.collectStickyDriveTarget,
        loop.collectStickySide,
        loop.collectStickyHalfLock,
        loop.collectDriveLastTarget,
        loop.collectForcedDriveCand,
        loop.collectNoFuelSec,
        loop.collectStuckSec,
        loop.collectEmptyDriveSec);
  }
}
