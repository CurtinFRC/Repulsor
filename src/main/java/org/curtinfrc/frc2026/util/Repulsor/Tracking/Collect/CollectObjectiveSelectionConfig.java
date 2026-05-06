package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import org.curtinfrc.frc2026.util.Repulsor.Scoring.WeightedScoreBreakdown;
import org.curtinfrc.frc2026.util.Repulsor.Scoring.WeightedScoreTerm;

/**
 * Tunable objective-selection weights and hysteresis thresholds for choosing collect resources.
 *
 * <p>This layer only chooses the resource/drive objective. Reactive bypass, waypoint staging, and
 * path obstacle handling remain owned by FieldPlanner.
 */
public record CollectObjectiveSelectionConfig(
    double resourceUnitGain,
    double etaCost,
    double hubFrontTrapPenalty,
    double canonicalScoreDropLimit,
    double richerUnitsAbsGain,
    double richerUnitsRelGain,
    double richerEtaDeltaMaxSeconds,
    double richerScoreDropLimit,
    double liveFuelPreferScoreMargin,
    double hubFrontTrapEscapeScoreAllowDrop,
    double nearbyCentroidScoreDropLimit,
    double liveRelockScoreDropLimit,
    double stickyPreferRankedScoreMargin,
    double farSwitchLockDistanceMeters,
    double farSwitchForceMultiplier,
    double closeSwitchEasyDistanceMeters,
    double closeSwitchMarginScale) {

  public static CollectObjectiveSelectionConfig defaults() {
    return new CollectObjectiveSelectionConfig(
        1.00, 0.55, 0.52, 0.12, 0.07, 1.45, 0.95, 0.30, 0.02, 0.16, 0.08, 0.04, 0.06, 2.8, 2.1,
        1.35, 0.55);
  }

  public CollectObjectiveSelectionConfig {
    resourceUnitGain = nonNegative(resourceUnitGain);
    etaCost = nonNegative(etaCost);
    hubFrontTrapPenalty = nonNegative(hubFrontTrapPenalty);
    canonicalScoreDropLimit = nonNegative(canonicalScoreDropLimit);
    richerUnitsAbsGain = nonNegative(richerUnitsAbsGain);
    richerUnitsRelGain = nonNegative(richerUnitsRelGain);
    richerEtaDeltaMaxSeconds = nonNegative(richerEtaDeltaMaxSeconds);
    richerScoreDropLimit = nonNegative(richerScoreDropLimit);
    liveFuelPreferScoreMargin = nonNegative(liveFuelPreferScoreMargin);
    hubFrontTrapEscapeScoreAllowDrop = nonNegative(hubFrontTrapEscapeScoreAllowDrop);
    nearbyCentroidScoreDropLimit = nonNegative(nearbyCentroidScoreDropLimit);
    liveRelockScoreDropLimit = nonNegative(liveRelockScoreDropLimit);
    stickyPreferRankedScoreMargin = nonNegative(stickyPreferRankedScoreMargin);
    farSwitchLockDistanceMeters = nonNegative(farSwitchLockDistanceMeters);
    farSwitchForceMultiplier = nonNegative(farSwitchForceMultiplier);
    closeSwitchEasyDistanceMeters = nonNegative(closeSwitchEasyDistanceMeters);
    closeSwitchMarginScale = nonNegative(closeSwitchMarginScale);
  }

  public WeightedScoreBreakdown scoreBreakdown(
      double resourceUnits, double etaSeconds, boolean hubFrontTrap) {
    return WeightedScoreBreakdown.of(
        WeightedScoreTerm.gain("resourceUnits", resourceUnits, resourceUnitGain),
        WeightedScoreTerm.cost("etaSeconds", etaSeconds, etaCost),
        WeightedScoreTerm.cost("hubFrontTrap", hubFrontTrap ? 1.0 : 0.0, hubFrontTrapPenalty));
  }

  public double score(double resourceUnits, double etaSeconds, boolean hubFrontTrap) {
    return scoreBreakdown(resourceUnits, etaSeconds, hubFrontTrap).total();
  }

  private static double nonNegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
