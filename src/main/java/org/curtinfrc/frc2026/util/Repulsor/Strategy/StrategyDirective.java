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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.CycleStrategyEvaluator.Intent;

public record StrategyDirective(
    String source,
    Intent intent,
    String regionId,
    Translation2d resourceTarget,
    String actionRole,
    String actionId,
    double expectedUnits,
    double cycleSeconds,
    double deadlineSeconds,
    double score) {
  public StrategyDirective {
    source = source == null || source.isBlank() ? "strategy" : source.trim();
    intent = intent == null ? Intent.FALLBACK : intent;
    regionId = regionId == null || regionId.isBlank() ? "none" : regionId.trim();
    actionRole = actionRole == null || actionRole.isBlank() ? "none" : actionRole.trim();
    actionId = actionId == null || actionId.isBlank() ? "none" : actionId.trim();
    expectedUnits = finiteNonNegative(expectedUnits);
    cycleSeconds = finiteNonNegative(cycleSeconds);
    deadlineSeconds = finiteNonNegative(deadlineSeconds);
    score = Double.isFinite(score) ? score : -1e18;
  }

  public static StrategyDirective none() {
    return new StrategyDirective(
        "none", Intent.FALLBACK, "none", null, "none", "none", 0.0, 0.0, 0.0, -1e18);
  }

  public boolean hasResourceTarget() {
    return resourceTarget != null;
  }

  public Pose2d targetPose(Rotation2d fallbackRotation) {
    return new Pose2d(
        resourceTarget == null ? new Translation2d() : resourceTarget,
        fallbackRotation == null ? Rotation2d.kZero : fallbackRotation);
  }

  private static double finiteNonNegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
