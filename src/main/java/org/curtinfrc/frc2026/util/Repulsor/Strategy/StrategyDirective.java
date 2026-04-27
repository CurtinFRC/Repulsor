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

/**
 * Behaviour-facing directive produced by a reasoner after generic strategy evaluation. It carries
 * the selected intent, target resource, and profile action ID while keeping behaviours decoupled
 * from year-specific terms such as fuel, pipes, hub, or shuttle.
 *
 * @param source reasoner or subsystem that produced the directive
 * @param intent selected generic cycle intent
 * @param regionId strategic region that supplied the target resource
 * @param resourceTarget field-relative resource target in meters, or {@code null}
 * @param actionRole generic action role requested from the field action profile
 * @param actionId concrete profile action ID to execute
 * @param expectedUnits expected resource units for telemetry and arbitration
 * @param cycleSeconds estimated cycle duration in seconds
 * @param deadlineSeconds remaining relevant scoring deadline in seconds
 * @param score final option score used for comparison
 */
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

  /**
   * Creates an inert directive used when no strategic action is available.
   *
   * @return fallback directive with no target or action
   */
  public static StrategyDirective none() {
    return new StrategyDirective(
        "none", Intent.FALLBACK, "none", null, "none", "none", 0.0, 0.0, 0.0, -1e18);
  }

  /**
   * Reports whether this directive contains a field-relative resource target.
   *
   * @return true when {@link #resourceTarget()} is non-null
   */
  public boolean hasResourceTarget() {
    return resourceTarget != null;
  }

  /**
   * Converts the resource target to a {@link Pose2d} for planner and command code.
   *
   * @param fallbackRotation rotation to use because the directive only stores a translation
   * @return pose at the resource target, or the field origin when no target exists
   */
  public Pose2d targetPose(Rotation2d fallbackRotation) {
    return new Pose2d(
        resourceTarget == null ? new Translation2d() : resourceTarget,
        fallbackRotation == null ? Rotation2d.kZero : fallbackRotation);
  }

  private static double finiteNonNegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
