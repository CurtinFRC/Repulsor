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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import org.junit.jupiter.api.Test;

class CollinearOcclusionTest {
  private static final double EPS = 1e-9;

  @Test
  void collinearOverlapWithGateEdgeTriggersOccludingGateStage() {
    GatedAttractorObstacle gate = collinearEdgeGate();
    FieldPlannerGoalManager manager = manager(gate);

    Pose2d requested = new Pose2d(new Translation2d(9.5, 4.5), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(8.4, 3.4), List.of(gate)));
    FieldPlannerWaypointStatus status = manager.getWaypointStatus();
    assertEquals("occluding_gate_stage", status.transitionReason());
    assertTrue(status.activeStage());
    assertEquals(10.0, status.stagedAttractor().getX(), EPS);
    assertEquals(5.0, status.stagedAttractor().getY(), EPS);
  }

  @Test
  void parallelSegmentMissingTheGateStillGoesDirect() {
    GatedAttractorObstacle gate = collinearEdgeGate();
    FieldPlannerGoalManager manager = manager(gate);

    Pose2d requested = new Pose2d(new Translation2d(10.5, 3.5), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertTrue(manager.updateStagedGoal(new Translation2d(9.0, 2.0), List.of(gate)));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
    assertEquals("default_direct", manager.getWaypointStatus().transitionReason());
  }

  private static GatedAttractorObstacle collinearEdgeGate() {
    Translation2d[] gatePoly =
        new Translation2d[] {
          new Translation2d(8.0, 7.0),
          new Translation2d(10.0, 5.0),
          new Translation2d(9.0, 4.0),
          new Translation2d(8.0, 3.0),
          new Translation2d(6.0, 5.0)
        };
    return new GatedAttractorObstacle(
        new Translation2d(8.0, 5.0),
        1.0,
        5.0,
        gatePoly,
        new Translation2d(10.0, 5.0),
        1.0,
        5.0,
        true);
  }

  private static FieldPlannerGoalManager manager(GatedAttractorObstacle gate) {
    return new FieldPlannerGoalManager(
        List.of(gate), Constants.FIELD_LENGTH, Constants.FIELD_WIDTH, zeroPaddingConfig());
  }

  private static FieldPlannerWaypointConfig zeroPaddingConfig() {
    return new FieldPlannerWaypointConfig(
        true,
        true,
        true,
        3.648981,
        1.5,
        0.0,
        0.28,
        0.45,
        1.05,
        1.40,
        3.0,
        4.2,
        0.70,
        2.40,
        2.0,
        0.35,
        List.of(),
        FieldPlannerWaypointPlacement.defaults());
  }
}
