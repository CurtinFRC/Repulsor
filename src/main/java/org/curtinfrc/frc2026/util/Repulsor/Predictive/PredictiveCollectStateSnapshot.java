package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import edu.wpi.first.math.geometry.Translation2d;

/** Immutable snapshot of the current predictive collect commitment state. */
public record PredictiveCollectStateSnapshot(
    Translation2d lastReturnedCollect,
    double lastReturnedCollectTs,
    Translation2d currentCollectTarget,
    double currentCollectChosenTs,
    double currentCollectScore,
    double currentCollectUnits,
    double currentCollectEta,
    double collectProgressLastTs,
    double collectProgressLastDist,
    double collectArrivalTs) {
  static PredictiveCollectStateSnapshot from(PredictiveFieldStateOps ops) {
    return new PredictiveCollectStateSnapshot(
        ops.lastReturnedCollect,
        ops.lastReturnedCollectTs,
        ops.currentCollectTarget,
        ops.currentCollectChosenTs,
        ops.currentCollectScore,
        ops.currentCollectUnits,
        ops.currentCollectEta,
        ops.collectProgressLastTs,
        ops.collectProgressLastDist,
        ops.collectArrivalTs);
  }
}
