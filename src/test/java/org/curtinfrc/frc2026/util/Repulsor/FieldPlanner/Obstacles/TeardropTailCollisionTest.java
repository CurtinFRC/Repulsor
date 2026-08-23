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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers.ExtraPathingClearPath;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers.ExtraPathingCollision;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.junit.jupiter.api.Test;

class TeardropTailCollisionTest {
  private static final double EPS = 1e-9;

  @Test
  void tailAngleDefaultsToWorldPlusXUntilForced() {
    TeardropObstacle obstacle = teardrop();
    assertEquals(0.0, obstacle.tailAngle().getDegrees(), EPS);
    assertTrue(
        obstacle.intersectsRectangle(
            ExtraPathingCollision.rectCorners(new Translation2d(3.0, 2.0), 0.5, 0.5)));
  }

  @Test
  void footprintFollowsTailAlongPositiveY() {
    TeardropObstacle obstacle = teardrop();
    obstacle.getForceAtPosition(new Translation2d(2.0, 3.0), new Translation2d(2.0, 0.5));
    assertEquals(90.0, obstacle.tailAngle().getDegrees(), EPS);

    assertTrue(
        obstacle.intersectsRectangle(
            ExtraPathingCollision.rectCorners(new Translation2d(2.0, 3.0), 0.5, 0.5)));
    assertFalse(
        obstacle.intersectsRectangle(
            ExtraPathingCollision.rectCorners(new Translation2d(2.0, 1.0), 0.5, 0.5)));
  }

  @Test
  void clearPathAgreesWithRotatedFootprint() {
    TeardropObstacle obstacle = teardrop();
    obstacle.getForceAtPosition(new Translation2d(2.0, 3.0), new Translation2d(2.0, 0.5));
    List<Obstacle> obstacles = List.of(obstacle);

    assertFalse(
        ExtraPathingClearPath.isClearPath(
            "TeardropTail",
            new Translation2d(2.0, 3.0),
            new Translation2d(2.0, 4.0),
            obstacles,
            0.1,
            0.1,
            false));
    assertTrue(
        ExtraPathingClearPath.isClearPath(
            "TeardropTail",
            new Translation2d(1.0, 1.0),
            new Translation2d(3.0, 1.0),
            obstacles,
            0.1,
            0.1,
            false));
  }

  private static TeardropObstacle teardrop() {
    return new TeardropObstacle(new Translation2d(2.0, 2.0), 1.0, 0.5, 0.25, 1.0, 1.5);
  }
}
