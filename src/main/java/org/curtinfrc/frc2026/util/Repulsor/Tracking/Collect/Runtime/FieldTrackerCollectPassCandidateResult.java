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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Function;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Scoring.WeightedScoreBreakdown;

/**
 * Immutable data record for field tracker collect pass candidate result values passed through the
 * Repulsor runtime helper layer shared by behaviours and planners. Use this type from robot code,
 * field profiles, or tests when integrating the corresponding Repulsor subsystem. Coordinates are
 * field-relative unless a method documents robot-relative motion.
 *
 * @param best component of the field tracker collect pass candidate result model
 * @param bestCandidate record component for the field tracker collect pass candidate result model
 * @param collectValid record component for the field tracker collect pass candidate result model
 * @param footprintHasFuel record component for the field tracker collect pass candidate result
 *     model
 * @param scoreResource record component for the field tracker collect pass candidate result model
 * @param immediatePose record component for the field tracker collect pass candidate result
 *     snapshot
 */
public record FieldTrackerCollectPassCandidateResult(
    PointCandidate best,
    Translation2d bestCandidate,
    Predicate<Translation2d> collectValid,
    Predicate<Translation2d> footprintHasFuel,
    Function<Translation2d, Double> scoreResource,
    Pose2d immediatePose,
    Function<Translation2d, WeightedScoreBreakdown> scoreBreakdown,
    boolean liveEvidenceRequired,
    boolean liveEvidenceFound,
    boolean staleObservationPresent,
    boolean canonicalized,
    boolean relockedToLiveEvidence,
    boolean trapPenaltyApplied,
    String selectionReason) {
  public FieldTrackerCollectPassCandidateResult(
      PointCandidate best,
      Translation2d bestCandidate,
      Predicate<Translation2d> collectValid,
      Predicate<Translation2d> footprintHasFuel,
      Function<Translation2d, Double> scoreResource,
      Pose2d immediatePose) {
    this(
        best,
        bestCandidate,
        collectValid,
        footprintHasFuel,
        scoreResource,
        immediatePose,
        ignored -> WeightedScoreBreakdown.empty(),
        false,
        false,
        false,
        false,
        false,
        false,
        "legacy");
  }

  public FieldTrackerCollectPassCandidateResult {
    if (scoreBreakdown == null) scoreBreakdown = ignored -> WeightedScoreBreakdown.empty();
    if (selectionReason == null || selectionReason.isBlank()) selectionReason = "unspecified";
  }
}
