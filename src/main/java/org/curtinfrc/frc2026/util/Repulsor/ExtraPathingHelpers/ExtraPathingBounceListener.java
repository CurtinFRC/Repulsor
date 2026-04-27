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

import edu.wpi.first.math.geometry.Pose2d;
import java.util.LinkedList;
import java.util.Queue;

/**
 * Provides extra pathing bounce listener functionality for the Repulsor extra pathing geometry and
 * collision helper layer. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class ExtraPathingBounceListener {
  private final double bounceDistanceThreshold;
  private final int bounceHistoryLimit;
  private final Queue<Pose2d> recentGoals = new LinkedList<>();
  private boolean isBouncing;

  /**
   * Returns the extra pathing bounce listener value maintained by this Repulsor component.
   *
   * @param bounceDistanceThreshold value used by this operation.
   * @param bounceHistoryLimit value used by this operation.
   */
  public ExtraPathingBounceListener(double bounceDistanceThreshold, int bounceHistoryLimit) {
    this.bounceDistanceThreshold = bounceDistanceThreshold;
    this.bounceHistoryLimit = bounceHistoryLimit;
  }

  /**
   * Updates update state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param currentGoal value used by this operation.
   */
  public void update(Pose2d currentGoal) {
    recentGoals.add(currentGoal);
    if (recentGoals.size() > bounceHistoryLimit) {
      recentGoals.poll();
    }

    isBouncing = checkBouncing();
  }

  private boolean checkBouncing() {
    if (recentGoals.size() < bounceHistoryLimit) return false;

    int similarCount = 0;
    Pose2d[] goals = recentGoals.toArray(new Pose2d[0]);
    for (int i = 0; i < goals.length - 1; i++) {
      for (int j = i + 1; j < goals.length; j++) {
        if (goals[i].getTranslation().getDistance(goals[j].getTranslation())
            < bounceDistanceThreshold) {
          similarCount++;
        }
      }
    }

    int totalPairs = (goals.length * (goals.length - 1)) / 2;
    return similarCount >= (totalPairs * 0.6);
  }

  /**
   * Updates clear history state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   */
  public void clearHistory() {
    recentGoals.clear();
    isBouncing = false;
  }

  /**
   * Returns the is bouncing value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isBouncing() {
    return isBouncing;
  }
}
