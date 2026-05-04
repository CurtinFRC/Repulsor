package org.curtinfrc.frc2026.util.Repulsor.Predictive.Model;

import edu.wpi.first.math.geometry.Translation2d;

/** Detailed score terms for the latest predictive field-object ranking pass. */
public record PredictiveRankingBreakdown(
    String levelId,
    Translation2d target,
    double totalScore,
    double advantageTerm,
    double distanceTerm,
    double pressureTerm,
    double congestionTerm,
    double capacityTerm,
    double headingTerm,
    double hysteresisTerm) {}
