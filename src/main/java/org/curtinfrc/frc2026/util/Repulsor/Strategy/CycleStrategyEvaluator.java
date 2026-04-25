/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package org.curtinfrc.frc2026.util.Repulsor.Strategy;

import edu.wpi.first.math.MathUtil;

public final class CycleStrategyEvaluator {
  private CycleStrategyEvaluator() {}

  public enum Intent {
    TRANSFER_FOR_LATER_SCORE,
    SCORE_AVAILABLE_RESOURCES,
    FALLBACK
  }

  public record Tuning(
      double minDriveSpeedMetersPerSecond,
      double collectSecondsPerUnit,
      double scoreSecondsPerUnit,
      double transferSecondsPerUnit,
      double activeScoreValuePerUnit,
      double deferredScoreValuePerUnit,
      double trafficRiskWeight,
      double obstacleRiskWeight,
      double contestedAreaRisk,
      double inactiveTransferBias,
      double activeScoreBias,
      double deadlineMissPenalty,
      double hysteresisScore) {
    public Tuning {
      minDriveSpeedMetersPerSecond = finitePositive(minDriveSpeedMetersPerSecond, 0.25);
      collectSecondsPerUnit = finiteNonNegative(collectSecondsPerUnit);
      scoreSecondsPerUnit = finiteNonNegative(scoreSecondsPerUnit);
      transferSecondsPerUnit = finiteNonNegative(transferSecondsPerUnit);
      activeScoreValuePerUnit = finiteNonNegative(activeScoreValuePerUnit);
      deferredScoreValuePerUnit = finiteNonNegative(deferredScoreValuePerUnit);
      trafficRiskWeight = finiteNonNegative(trafficRiskWeight);
      obstacleRiskWeight = finiteNonNegative(obstacleRiskWeight);
      contestedAreaRisk = finiteNonNegative(contestedAreaRisk);
      inactiveTransferBias = finiteNonNegative(inactiveTransferBias);
      activeScoreBias = finiteNonNegative(activeScoreBias);
      deadlineMissPenalty = finiteNonNegative(deadlineMissPenalty);
      hysteresisScore = finiteNonNegative(hysteresisScore);
    }

    public static Tuning defaults() {
      return new Tuning(0.25, 0.08, 0.10, 0.12, 1.0, 0.85, 0.55, 0.35, 0.25, 0.35, 0.15, 1.2, 0.08);
    }
  }

  public record Inputs(
      boolean scoringWindowActive,
      double remainingScoringWindowSeconds,
      double driveSpeedMetersPerSecond,
      ResourceRegionSummary scoringSideResources,
      ResourceRegionSummary centerResources,
      double scoringSideTravelMeters,
      double centerTravelMeters,
      double centerReturnMeters,
      Intent currentIntent) {
    public Inputs {
      remainingScoringWindowSeconds = finiteNonNegative(remainingScoringWindowSeconds);
      driveSpeedMetersPerSecond = finitePositive(driveSpeedMetersPerSecond, 0.25);
      scoringSideResources =
          scoringSideResources == null
              ? ResourceRegionSummary.empty("scoringSide")
              : scoringSideResources;
      centerResources =
          centerResources == null ? ResourceRegionSummary.empty("center") : centerResources;
      scoringSideTravelMeters = finiteNonNegative(scoringSideTravelMeters);
      centerTravelMeters = finiteNonNegative(centerTravelMeters);
      centerReturnMeters = finiteNonNegative(centerReturnMeters);
      currentIntent = currentIntent == null ? Intent.FALLBACK : currentIntent;
    }
  }

  public record Option(
      Intent intent,
      ResourceRegionSummary resources,
      double expectedUnits,
      double cycleSeconds,
      double grossValue,
      double riskCost,
      double deadlineCost,
      double score) {
    public Option {
      expectedUnits = finiteNonNegative(expectedUnits);
      cycleSeconds = finitePositive(cycleSeconds, 0.001);
      grossValue = finiteNonNegative(grossValue);
      riskCost = finiteNonNegative(riskCost);
      deadlineCost = finiteNonNegative(deadlineCost);
      score = Double.isFinite(score) ? score : -1e18;
    }
  }

  public record Decision(
      Intent intent, Option scoringOption, Option transferOption, Option bestOption) {}

  public static Decision decide(Inputs inputs, Tuning tuning) {
    Tuning t = tuning == null ? Tuning.defaults() : tuning;
    Inputs in = inputs == null ? emptyInputs() : inputs;

    Option score = scoreOption(in, t);
    Option transfer = transferOption(in, t);

    Option best;
    if (!in.scoringWindowActive()) {
      best = transfer.score() >= score.score() - t.hysteresisScore() ? transfer : score;
    } else {
      best =
          score.score() + hysteresisBonus(in, Intent.SCORE_AVAILABLE_RESOURCES, t)
                  >= transfer.score() + hysteresisBonus(in, Intent.TRANSFER_FOR_LATER_SCORE, t)
              ? score
              : transfer;
    }

    if (!best.resources().hasResources()) {
      best =
          new Option(
              Intent.FALLBACK,
              ResourceRegionSummary.empty("fallback"),
              0.0,
              0.001,
              0.0,
              0.0,
              0.0,
              -1e18);
    }
    return new Decision(best.intent(), score, transfer, best);
  }

  private static Option scoreOption(Inputs in, Tuning t) {
    ResourceRegionSummary r = in.scoringSideResources();
    double units = r.resourceUnits();
    double cycleSeconds =
        in.scoringSideTravelMeters() / driveSpeed(in, t)
            + units * (t.collectSecondsPerUnit() + t.scoreSecondsPerUnit());
    double deadlineCost =
        in.scoringWindowActive() && cycleSeconds > in.remainingScoringWindowSeconds()
            ? (cycleSeconds - in.remainingScoringWindowSeconds()) * t.deadlineMissPenalty()
            : 0.0;
    double valuePerUnit =
        in.scoringWindowActive()
            ? t.activeScoreValuePerUnit()
            : t.deferredScoreValuePerUnit() * 0.35;
    double gross = units * valuePerUnit + (in.scoringWindowActive() ? t.activeScoreBias() : 0.0);
    double risk = risk(r, t);
    double score = valueRate(gross, cycleSeconds) - risk - deadlineCost;
    if (!r.hasResources()) score = -1e18;
    return new Option(
        Intent.SCORE_AVAILABLE_RESOURCES, r, units, cycleSeconds, gross, risk, deadlineCost, score);
  }

  private static Option transferOption(Inputs in, Tuning t) {
    ResourceRegionSummary r = in.centerResources();
    double units = r.resourceUnits();
    double cycleSeconds =
        (in.centerTravelMeters() + in.centerReturnMeters()) / driveSpeed(in, t)
            + units * (t.collectSecondsPerUnit() + t.transferSecondsPerUnit());
    double gross =
        units
                * (in.scoringWindowActive()
                    ? t.deferredScoreValuePerUnit()
                    : t.activeScoreValuePerUnit())
            + (!in.scoringWindowActive() ? t.inactiveTransferBias() : 0.0);
    double deadlineCost =
        in.scoringWindowActive() && cycleSeconds > in.remainingScoringWindowSeconds()
            ? 0.35 * (cycleSeconds - in.remainingScoringWindowSeconds()) * t.deadlineMissPenalty()
            : 0.0;
    double risk = risk(r, t) + t.contestedAreaRisk();
    double score = valueRate(gross, cycleSeconds) - risk - deadlineCost;
    if (!r.hasResources()) score = -1e18;
    return new Option(
        Intent.TRANSFER_FOR_LATER_SCORE, r, units, cycleSeconds, gross, risk, deadlineCost, score);
  }

  private static double driveSpeed(Inputs in, Tuning t) {
    return Math.max(t.minDriveSpeedMetersPerSecond(), in.driveSpeedMetersPerSecond());
  }

  private static double risk(ResourceRegionSummary r, Tuning t) {
    return r.trafficRisk() * t.trafficRiskWeight() + r.obstacleRisk() * t.obstacleRiskWeight();
  }

  private static double valueRate(double value, double seconds) {
    return value / Math.max(0.001, seconds);
  }

  private static double hysteresisBonus(Inputs in, Intent intent, Tuning t) {
    return in.currentIntent() == intent ? t.hysteresisScore() : 0.0;
  }

  private static Inputs emptyInputs() {
    return new Inputs(
        false,
        0.0,
        1.0,
        ResourceRegionSummary.empty("scoringSide"),
        ResourceRegionSummary.empty("center"),
        0.0,
        0.0,
        0.0,
        Intent.FALLBACK);
  }

  private static double finitePositive(double value, double fallback) {
    return Double.isFinite(value) && value > 0.0 ? value : fallback;
  }

  private static double finiteNonNegative(double value) {
    return Double.isFinite(value) ? MathUtil.clamp(value, 0.0, 1e9) : 0.0;
  }
}
