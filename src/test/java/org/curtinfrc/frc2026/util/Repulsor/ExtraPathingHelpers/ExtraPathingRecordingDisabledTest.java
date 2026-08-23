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

package org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.junit.jupiter.api.Test;

class ExtraPathingRecordingDisabledTest {
  @Test
  void telemetryIsDisabledByDefault() {
    assertFalse(ExtraPathingRecording.isTelemetryEnabled());
  }

  @Test
  void recordEntryPointsAreNoOpsWhenDisabled() {
    List<Obstacle> obstacles = List.of();
    Translation2d start = new Translation2d();
    Translation2d goal = new Translation2d(4.0, 4.0);

    long startNanos = System.nanoTime();
    for (int i = 0; i < 100; i++) {
      ExtraPathingRecording.recordForbiddenGrid(
          "Test/Forbidden", obstacles, 0.65, 0.65, 0.25, goal, 0.2);
      ExtraPathingRecording.recordPath("Test/Path", List.of(start, goal));
      ExtraPathingRecording.recordPoints("Test/Points", List.of(start, goal));
      ExtraPathingRecording.recordCircle("Test/Circle", goal, 0.5, 40);
      ExtraPathingRecording.recordEllipse("Test/Ellipse", goal, 0.5, 0.25, 60);
      ExtraPathingRecording.recordCorridor("Test/Corridor", start, goal, 0.45);
      ExtraPathingRecording.renderObstacles("Test/Obstacles", obstacles, 0.45);
    }
    long elapsedMs = (System.nanoTime() - startNanos) / 1_000_000L;

    assertTrue(
        elapsedMs < 1000,
        "disabled recording should skip payload building, took " + elapsedMs + "ms");
  }
}
