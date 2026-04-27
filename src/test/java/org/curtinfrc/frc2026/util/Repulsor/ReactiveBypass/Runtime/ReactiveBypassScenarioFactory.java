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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.curtinfrc.frc2026.util.Repulsor.Constants;

final class ReactiveBypassScenarioFactory {
  private static final double ROBOT_X = 0.85;
  private static final double ROBOT_Y = 0.85;
  private static final double FIELD_LEN_METERS = Constants.FIELD_LENGTH;
  private static final double FIELD_WID_METERS = Constants.FIELD_WIDTH;
  private static final int EARLY_FIXED_SCENARIO_WEIGHT = 3;
  private static final int MID_FIXED_SCENARIO_WEIGHT = 3;
  private static final int LATE_FIXED_SCENARIO_WEIGHT = 4;
  private static final int EARLY_RANDOM_SCENARIOS = 60;

  private ReactiveBypassScenarioFactory() {}

  static List<Scenario> buildCurriculumScenarios(
      List<Scenario> fixedScenarios,
      List<Scenario> randomTrainingScenarios,
      List<Scenario> hardRandomScenarios,
      List<Scenario> minedHardCases,
      int generation) {
    List<Scenario> scenarios = new ArrayList<>();

    int fixedWeight =
        generation <= 8
            ? EARLY_FIXED_SCENARIO_WEIGHT
            : generation <= 20 ? MID_FIXED_SCENARIO_WEIGHT : LATE_FIXED_SCENARIO_WEIGHT;
    for (int i = 0; i < fixedWeight; i++) {
      scenarios.addAll(fixedScenarios);
    }

    int randomLimit =
        generation <= 8
            ? Math.min(EARLY_RANDOM_SCENARIOS, randomTrainingScenarios.size())
            : randomTrainingScenarios.size();
    scenarios.addAll(randomTrainingScenarios.subList(0, randomLimit));

    if (generation >= 21) {
      scenarios.addAll(hardRandomScenarios);
    }

    scenarios.addAll(minedHardCases);
    return scenarios;
  }

  static List<Scenario> buildScenarios() {
    return List.of(
        new Scenario(
            "core-straight-single-block",
            new Pose2d(1.7, 4.0, Rotation2d.kZero),
            new Pose2d(14.4, 4.0, Rotation2d.kZero),
            List.of(new ObstacleSpec(7.7, 4.0, 0.48, 1.2)),
            8.0),
        new Scenario(
            "core-double-gap-top",
            new Pose2d(2.1, 3.4, Rotation2d.kZero),
            new Pose2d(14.2, 3.5, Rotation2d.kZero),
            List.of(new ObstacleSpec(7.1, 3.20, 0.45, 1.2), new ObstacleSpec(8.2, 4.25, 0.45, 1.2)),
            8.5),
        new Scenario(
            "core-double-gap-bottom",
            new Pose2d(2.1, 4.7, Rotation2d.kZero),
            new Pose2d(14.2, 4.6, Rotation2d.kZero),
            List.of(new ObstacleSpec(7.1, 4.85, 0.45, 1.2), new ObstacleSpec(8.2, 3.85, 0.45, 1.2)),
            8.5),
        new Scenario(
            "core-offset-left-bypass",
            new Pose2d(2.6, 2.1, Rotation2d.kZero),
            new Pose2d(13.0, 2.8, Rotation2d.kZero),
            List.of(new ObstacleSpec(7.0, 2.35, 0.58, 1.4)),
            8.0),
        new Scenario(
            "core-offset-right-bypass",
            new Pose2d(13.8, 5.8, Rotation2d.k180deg),
            new Pose2d(2.3, 5.0, Rotation2d.k180deg),
            List.of(new ObstacleSpec(8.4, 5.55, 0.58, 1.4)),
            8.5),
        new Scenario(
            "core-near-wall-pinned-low",
            new Pose2d(1.2, 1.0, Rotation2d.kZero),
            new Pose2d(6.2, 1.05, Rotation2d.kZero),
            List.of(new ObstacleSpec(3.0, 1.05, 0.52, 1.5)),
            7.0),
        new Scenario(
            "core-near-wall-pinned-high",
            new Pose2d(14.7, 7.0, Rotation2d.k180deg),
            new Pose2d(9.6, 6.95, Rotation2d.k180deg),
            List.of(new ObstacleSpec(12.8, 6.95, 0.52, 1.5)),
            7.0),
        new Scenario(
            "core-corner-release-low",
            new Pose2d(14.9, 1.0, Rotation2d.k180deg),
            new Pose2d(10.2, 2.3, Rotation2d.k180deg),
            List.of(
                new ObstacleSpec(13.7, 1.15, 0.45, 1.2), new ObstacleSpec(13.0, 1.75, 0.40, 1.2)),
            7.5),
        new Scenario(
            "core-corner-release-high",
            new Pose2d(1.0, 7.0, Rotation2d.kZero),
            new Pose2d(5.8, 5.7, Rotation2d.kZero),
            List.of(new ObstacleSpec(2.1, 6.85, 0.45, 1.2), new ObstacleSpec(2.9, 6.25, 0.40, 1.2)),
            7.5),
        new Scenario(
            "core-diagonal-crossing-down",
            new Pose2d(3.0, 6.4, Rotation2d.kZero),
            new Pose2d(13.5, 1.7, Rotation2d.kZero),
            List.of(new ObstacleSpec(7.0, 4.55, 0.46, 1.3), new ObstacleSpec(8.2, 3.80, 0.46, 1.3)),
            9.0),
        new Scenario(
            "core-diagonal-crossing-up",
            new Pose2d(13.5, 1.7, Rotation2d.k180deg),
            new Pose2d(3.0, 6.4, Rotation2d.k180deg),
            List.of(new ObstacleSpec(8.6, 3.45, 0.46, 1.3), new ObstacleSpec(7.4, 4.20, 0.46, 1.3)),
            9.0),
        new Scenario(
            "core-late-obstacle-near-goal",
            new Pose2d(2.0, 4.2, Rotation2d.kZero),
            new Pose2d(11.5, 4.2, Rotation2d.kZero),
            List.of(new ObstacleSpec(10.0, 4.2, 0.42, 1.2)),
            7.0),
        new Scenario(
            "core-start-blocker",
            new Pose2d(2.0, 4.0, Rotation2d.kZero),
            new Pose2d(13.8, 4.0, Rotation2d.kZero),
            List.of(new ObstacleSpec(3.0, 4.05, 0.48, 1.3)),
            8.5),
        new Scenario(
            "core-goal-behind-blocker",
            new Pose2d(2.0, 3.6, Rotation2d.kZero),
            new Pose2d(10.8, 3.6, Rotation2d.kZero),
            List.of(new ObstacleSpec(9.7, 3.6, 0.45, 1.4)),
            7.5),
        new Scenario(
            "core-narrow-corridor",
            new Pose2d(2.0, 4.0, Rotation2d.kZero),
            new Pose2d(14.0, 4.0, Rotation2d.kZero),
            List.of(new ObstacleSpec(7.4, 3.30, 0.55, 1.4), new ObstacleSpec(7.4, 4.70, 0.55, 1.4)),
            9.0),
        new Scenario(
            "core-side-switch-trap",
            new Pose2d(2.0, 4.0, Rotation2d.kZero),
            new Pose2d(14.0, 4.0, Rotation2d.kZero),
            List.of(
                new ObstacleSpec(5.4, 3.65, 0.42, 1.2),
                new ObstacleSpec(7.2, 4.35, 0.42, 1.2),
                new ObstacleSpec(9.0, 3.65, 0.42, 1.2)),
            9.0),
        new Scenario(
            "core-center-clutter",
            new Pose2d(1.8, 2.4, Rotation2d.kZero),
            new Pose2d(14.5, 5.7, Rotation2d.kZero),
            List.of(
                new ObstacleSpec(5.7, 3.1, 0.42, 1.1),
                new ObstacleSpec(7.3, 4.0, 0.48, 1.3),
                new ObstacleSpec(9.1, 4.7, 0.42, 1.1),
                new ObstacleSpec(10.4, 5.2, 0.35, 1.0)),
            10.0),
        new Scenario(
            "core-short-shuttle-blocked",
            new Pose2d(5.0, 2.0, Rotation2d.kZero),
            new Pose2d(8.8, 2.4, Rotation2d.kZero),
            List.of(new ObstacleSpec(6.8, 2.15, 0.42, 1.2)),
            5.5));
  }

  private enum ScenarioKind {
    STRAIGHT_BLOCKER,
    OFFSET_BLOCKER,
    DOUBLE_GAP,
    NEAR_WALL,
    CORNER_ESCAPE,
    DIAGONAL_TRAFFIC,
    LATE_GOAL_BLOCKER,
    START_BLOCKER,
    NARROW_CORRIDOR,
    SIDE_SWITCH_TRAP,
    CENTER_CLUTTER,
    SHORT_SHUTTLE
  }

  static List<Scenario> buildHardRandomScenarios(Random random, int count, String prefix) {
    List<Scenario> scenarios = new ArrayList<>();
    for (int i = 0; i < count; i++) {
      ScenarioKind kind = pickHardScenarioKind(random);
      scenarios.add(buildRandomScenario(random, kind, prefix + "-" + i));
    }
    return scenarios;
  }

  private static ScenarioKind pickHardScenarioKind(Random random) {
    double r = random.nextDouble();
    if (r < 0.18) return ScenarioKind.NEAR_WALL;
    if (r < 0.34) return ScenarioKind.CORNER_ESCAPE;
    if (r < 0.52) return ScenarioKind.NARROW_CORRIDOR;
    if (r < 0.68) return ScenarioKind.SIDE_SWITCH_TRAP;
    if (r < 0.84) return ScenarioKind.CENTER_CLUTTER;
    return ScenarioKind.LATE_GOAL_BLOCKER;
  }

  static List<Scenario> buildValidationScenarios(long seed, int count) {
    Random validationRandom = new Random(seed);
    List<Scenario> scenarios = new ArrayList<>(buildScenarios());
    scenarios.addAll(buildRandomScenarios(validationRandom, count, "validation"));
    return scenarios;
  }

  static List<Scenario> buildRandomScenarios(Random random, int count, String prefix) {
    List<Scenario> scenarios = new ArrayList<>();

    for (int i = 0; i < count; i++) {
      ScenarioKind kind = pickScenarioKind(random);
      scenarios.add(buildRandomScenario(random, kind, prefix + "-" + i));
    }

    return scenarios;
  }

  private static ScenarioKind pickScenarioKind(Random random) {
    double r = random.nextDouble();

    if (r < 0.12) return ScenarioKind.STRAIGHT_BLOCKER;
    if (r < 0.22) return ScenarioKind.OFFSET_BLOCKER;
    if (r < 0.34) return ScenarioKind.DOUBLE_GAP;
    if (r < 0.44) return ScenarioKind.NEAR_WALL;
    if (r < 0.52) return ScenarioKind.CORNER_ESCAPE;
    if (r < 0.64) return ScenarioKind.DIAGONAL_TRAFFIC;
    if (r < 0.72) return ScenarioKind.LATE_GOAL_BLOCKER;
    if (r < 0.78) return ScenarioKind.START_BLOCKER;
    if (r < 0.86) return ScenarioKind.NARROW_CORRIDOR;
    if (r < 0.92) return ScenarioKind.SIDE_SWITCH_TRAP;
    if (r < 0.97) return ScenarioKind.CENTER_CLUTTER;
    return ScenarioKind.SHORT_SHUTTLE;
  }

  private static Scenario buildRandomScenario(Random random, ScenarioKind kind, String name) {
    return switch (kind) {
      case STRAIGHT_BLOCKER -> randomStraightBlocker(random, name);
      case OFFSET_BLOCKER -> randomOffsetBlocker(random, name);
      case DOUBLE_GAP -> randomDoubleGap(random, name);
      case NEAR_WALL -> randomNearWall(random, name);
      case CORNER_ESCAPE -> randomCornerEscape(random, name);
      case DIAGONAL_TRAFFIC -> randomDiagonalTraffic(random, name);
      case LATE_GOAL_BLOCKER -> randomLateGoalBlocker(random, name);
      case START_BLOCKER -> randomStartBlocker(random, name);
      case NARROW_CORRIDOR -> randomNarrowCorridor(random, name);
      case SIDE_SWITCH_TRAP -> randomSideSwitchTrap(random, name);
      case CENTER_CLUTTER -> randomCenterClutter(random, name);
      case SHORT_SHUTTLE -> randomShortShuttle(random, name);
    };
  }

  private static Scenario randomStraightBlocker(Random random, String name) {
    double y = rand(random, 1.2, 7.0);
    boolean leftToRight = chance(random, 0.5);

    Translation2d start =
        new Translation2d(leftToRight ? rand(random, 1.0, 2.8) : rand(random, 13.7, 15.4), y);
    Translation2d goal =
        new Translation2d(
            leftToRight ? rand(random, 13.0, 15.2) : rand(random, 1.2, 3.0),
            y + rand(random, -0.25, 0.25));

    Translation2d obs =
        normalOffset(start, goal, rand(random, 0.40, 0.65), rand(random, -0.15, 0.15));

    return scenarioFromPoints(
        name + "-straight-blocker",
        start,
        goal,
        List.of(obstacleAt(obs, rand(random, 0.38, 0.62), rand(random, 1.0, 1.7))),
        2.5);
  }

  private static Scenario randomOffsetBlocker(Random random, String name) {
    double y = rand(random, 1.2, 7.0);
    boolean leftToRight = chance(random, 0.5);

    Translation2d start =
        new Translation2d(leftToRight ? rand(random, 1.0, 3.0) : rand(random, 13.5, 15.4), y);
    Translation2d goal =
        new Translation2d(
            leftToRight ? rand(random, 12.5, 15.2) : rand(random, 1.2, 3.2),
            y + rand(random, -0.8, 0.8));

    double side = chance(random, 0.5) ? 1.0 : -1.0;
    Translation2d obs =
        normalOffset(start, goal, rand(random, 0.35, 0.75), side * rand(random, 0.25, 0.85));

    return scenarioFromPoints(
        name + "-offset-blocker",
        start,
        goal,
        List.of(obstacleAt(obs, rand(random, 0.35, 0.65), rand(random, 0.9, 1.6))),
        2.5);
  }

  private static Scenario randomDoubleGap(Random random, String name) {
    boolean leftToRight = chance(random, 0.5);
    double y = rand(random, 2.0, 6.2);

    Translation2d start =
        new Translation2d(leftToRight ? rand(random, 1.0, 2.8) : rand(random, 13.6, 15.4), y);
    Translation2d goal =
        new Translation2d(
            leftToRight ? rand(random, 13.0, 15.2) : rand(random, 1.2, 3.0),
            y + rand(random, -0.45, 0.45));

    double along = rand(random, 0.40, 0.65);
    double gapHalf = rand(random, 0.55, 0.95);

    Translation2d obsA = normalOffset(start, goal, along, -gapHalf);
    Translation2d obsB = normalOffset(start, goal, along + rand(random, -0.05, 0.08), gapHalf);

    return scenarioFromPoints(
        name + "-double-gap",
        start,
        goal,
        List.of(
            obstacleAt(obsA, rand(random, 0.38, 0.58), rand(random, 1.0, 1.6)),
            obstacleAt(obsB, rand(random, 0.38, 0.58), rand(random, 1.0, 1.6))),
        3.0);
  }

  private static Scenario randomNearWall(Random random, String name) {
    boolean bottom = chance(random, 0.5);
    boolean leftToRight = chance(random, 0.5);

    double y = bottom ? rand(random, 0.75, 1.35) : rand(random, 6.85, 7.45);
    Translation2d start =
        new Translation2d(leftToRight ? rand(random, 0.9, 2.2) : rand(random, 14.2, 15.5), y);
    Translation2d goal =
        new Translation2d(
            leftToRight ? rand(random, 5.8, 8.8) : rand(random, 7.7, 10.7),
            y + rand(random, -0.15, 0.15));

    Translation2d obs =
        normalOffset(start, goal, rand(random, 0.32, 0.55), rand(random, -0.12, 0.12));

    return scenarioFromPoints(
        name + "-near-wall",
        start,
        goal,
        List.of(obstacleAt(obs, rand(random, 0.42, 0.65), rand(random, 1.2, 1.9))),
        3.0);
  }

  private static Scenario randomCornerEscape(Random random, String name) {
    boolean leftCorner = chance(random, 0.5);
    boolean bottomCorner = chance(random, 0.5);

    double startX = leftCorner ? rand(random, 0.75, 1.35) : rand(random, 15.15, 15.8);
    double startY = bottomCorner ? rand(random, 0.75, 1.35) : rand(random, 6.85, 7.45);

    double goalX = leftCorner ? rand(random, 4.5, 7.0) : rand(random, 9.5, 12.0);
    double goalY = bottomCorner ? rand(random, 1.8, 3.4) : rand(random, 4.8, 6.4);

    Translation2d start = new Translation2d(startX, startY);
    Translation2d goal = new Translation2d(goalX, goalY);

    List<ObstacleSpec> obstacles = new ArrayList<>();
    obstacles.add(
        obstacleAt(
            normalOffset(start, goal, rand(random, 0.20, 0.40), rand(random, -0.15, 0.15)),
            rand(random, 0.35, 0.55),
            rand(random, 1.1, 1.7)));
    obstacles.add(
        obstacleAt(
            normalOffset(start, goal, rand(random, 0.38, 0.60), rand(random, -0.45, 0.45)),
            rand(random, 0.32, 0.52),
            rand(random, 1.0, 1.6)));

    return scenarioFromPoints(name + "-corner-escape", start, goal, obstacles, 3.5);
  }

  private static Scenario randomDiagonalTraffic(Random random, String name) {
    boolean down = chance(random, 0.5);
    boolean leftToRight = chance(random, 0.5);

    Translation2d start =
        new Translation2d(
            leftToRight ? rand(random, 1.0, 3.2) : rand(random, 13.3, 15.4),
            down ? rand(random, 5.5, 7.2) : rand(random, 1.0, 2.7));

    Translation2d goal =
        new Translation2d(
            leftToRight ? rand(random, 12.8, 15.2) : rand(random, 1.2, 3.4),
            down ? rand(random, 1.0, 2.7) : rand(random, 5.5, 7.2));

    List<ObstacleSpec> obstacles = new ArrayList<>();
    int count = 2 + random.nextInt(3);

    for (int i = 0; i < count; i++) {
      obstacles.add(
          obstacleAt(
              normalOffset(start, goal, rand(random, 0.25, 0.82), rand(random, -0.9, 0.9)),
              rand(random, 0.32, 0.55),
              rand(random, 0.9, 1.6)));
    }

    return scenarioFromPoints(name + "-diagonal-traffic", start, goal, obstacles, 3.5);
  }

  private static Scenario randomLateGoalBlocker(Random random, String name) {
    boolean leftToRight = chance(random, 0.5);
    double y = rand(random, 1.3, 6.9);

    Translation2d start =
        new Translation2d(leftToRight ? rand(random, 1.0, 3.0) : rand(random, 13.5, 15.4), y);
    Translation2d goal =
        new Translation2d(
            leftToRight ? rand(random, 9.5, 13.0) : rand(random, 3.5, 7.0),
            y + rand(random, -0.4, 0.4));

    Translation2d obs =
        normalOffset(start, goal, rand(random, 0.80, 0.94), rand(random, -0.2, 0.2));

    return scenarioFromPoints(
        name + "-late-goal-blocker",
        start,
        goal,
        List.of(obstacleAt(obs, rand(random, 0.32, 0.52), rand(random, 1.0, 1.7))),
        2.8);
  }

  private static Scenario randomStartBlocker(Random random, String name) {
    boolean leftToRight = chance(random, 0.5);
    double y = rand(random, 1.2, 7.0);

    Translation2d start =
        new Translation2d(leftToRight ? rand(random, 1.0, 3.0) : rand(random, 13.5, 15.4), y);
    Translation2d goal =
        new Translation2d(
            leftToRight ? rand(random, 11.0, 15.2) : rand(random, 1.2, 5.4),
            y + rand(random, -0.6, 0.6));

    Translation2d obs =
        normalOffset(start, goal, rand(random, 0.08, 0.20), rand(random, -0.18, 0.18));

    return scenarioFromPoints(
        name + "-start-blocker",
        start,
        goal,
        List.of(obstacleAt(obs, rand(random, 0.35, 0.55), rand(random, 1.0, 1.6))),
        3.0);
  }

  private static Scenario randomNarrowCorridor(Random random, String name) {
    boolean leftToRight = chance(random, 0.5);
    double y = rand(random, 2.2, 6.0);

    Translation2d start =
        new Translation2d(leftToRight ? rand(random, 1.0, 2.8) : rand(random, 13.6, 15.4), y);
    Translation2d goal =
        new Translation2d(
            leftToRight ? rand(random, 13.0, 15.2) : rand(random, 1.2, 3.0),
            y + rand(random, -0.25, 0.25));

    double along = rand(random, 0.42, 0.65);
    double side = rand(random, 0.62, 0.86);

    Translation2d obsA = normalOffset(start, goal, along, -side);
    Translation2d obsB = normalOffset(start, goal, along, side);

    return scenarioFromPoints(
        name + "-narrow-corridor",
        start,
        goal,
        List.of(
            obstacleAt(obsA, rand(random, 0.48, 0.68), rand(random, 1.2, 1.9)),
            obstacleAt(obsB, rand(random, 0.48, 0.68), rand(random, 1.2, 1.9))),
        3.5);
  }

  private static Scenario randomSideSwitchTrap(Random random, String name) {
    boolean leftToRight = chance(random, 0.5);
    double y = rand(random, 2.0, 6.2);

    Translation2d start =
        new Translation2d(leftToRight ? rand(random, 1.0, 2.8) : rand(random, 13.6, 15.4), y);
    Translation2d goal =
        new Translation2d(
            leftToRight ? rand(random, 13.0, 15.2) : rand(random, 1.2, 3.0),
            y + rand(random, -0.2, 0.2));

    List<ObstacleSpec> obstacles = new ArrayList<>();
    double side = chance(random, 0.5) ? 1.0 : -1.0;

    obstacles.add(
        obstacleAt(
            normalOffset(start, goal, 0.30, side * rand(random, 0.25, 0.55)),
            rand(random, 0.35, 0.52),
            rand(random, 1.0, 1.5)));
    obstacles.add(
        obstacleAt(
            normalOffset(start, goal, 0.48, -side * rand(random, 0.25, 0.55)),
            rand(random, 0.35, 0.52),
            rand(random, 1.0, 1.5)));
    obstacles.add(
        obstacleAt(
            normalOffset(start, goal, 0.66, side * rand(random, 0.25, 0.55)),
            rand(random, 0.35, 0.52),
            rand(random, 1.0, 1.5)));

    return scenarioFromPoints(name + "-side-switch-trap", start, goal, obstacles, 3.5);
  }

  private static Scenario randomCenterClutter(Random random, String name) {
    boolean leftToRight = chance(random, 0.5);

    Translation2d start =
        new Translation2d(
            leftToRight ? rand(random, 1.0, 3.2) : rand(random, 13.3, 15.4),
            rand(random, 1.2, 7.0));

    Translation2d goal =
        new Translation2d(
            leftToRight ? rand(random, 13.0, 15.2) : rand(random, 1.2, 3.4),
            rand(random, 1.2, 7.0));

    List<ObstacleSpec> obstacles = new ArrayList<>();
    int count = 3 + random.nextInt(4);

    for (int i = 0; i < count; i++) {
      obstacles.add(
          obstacleAt(
              normalOffset(start, goal, rand(random, 0.25, 0.82), rand(random, -1.35, 1.35)),
              rand(random, 0.28, 0.55),
              rand(random, 0.8, 1.5)));
    }

    return scenarioFromPoints(name + "-center-clutter", start, goal, obstacles, 4.0);
  }

  private static Scenario randomShortShuttle(Random random, String name) {
    double startX = rand(random, 4.0, 9.5);
    double startY = rand(random, 1.2, 7.0);

    double angle = rand(random, -0.45, 0.45);
    double distance = rand(random, 3.0, 5.5);
    double direction = chance(random, 0.5) ? 1.0 : -1.0;

    Translation2d start = new Translation2d(startX, startY);
    Translation2d goal =
        clampPointToField(
            new Translation2d(
                startX + direction * Math.cos(angle) * distance,
                startY + Math.sin(angle) * distance));

    Translation2d obs =
        normalOffset(start, goal, rand(random, 0.35, 0.65), rand(random, -0.35, 0.35));

    return scenarioFromPoints(
        name + "-short-shuttle",
        start,
        goal,
        List.of(obstacleAt(obs, rand(random, 0.32, 0.52), rand(random, 1.0, 1.6))),
        2.2);
  }

  private static double rand(Random random, double min, double max) {
    return min + random.nextDouble() * (max - min);
  }

  private static boolean chance(Random random, double probability) {
    return random.nextDouble() < probability;
  }

  private static Translation2d lerp(Translation2d a, Translation2d b, double t) {
    return new Translation2d(
        a.getX() + (b.getX() - a.getX()) * t, a.getY() + (b.getY() - a.getY()) * t);
  }

  private static Translation2d normalOffset(
      Translation2d start, Translation2d goal, double along, double sideways) {
    Translation2d base = lerp(start, goal, along);
    Translation2d delta = goal.minus(start);
    double len = Math.max(1e-9, delta.getNorm());

    double nx = -delta.getY() / len;
    double ny = delta.getX() / len;

    return new Translation2d(base.getX() + nx * sideways, base.getY() + ny * sideways);
  }

  private static Translation2d clampPointToField(Translation2d point) {
    return new Translation2d(
        ReactiveBypassOptimizer.clamp(
            point.getX(), ROBOT_X * 0.5, FIELD_LEN_METERS - ROBOT_X * 0.5),
        ReactiveBypassOptimizer.clamp(
            point.getY(), ROBOT_Y * 0.5, FIELD_WID_METERS - ROBOT_Y * 0.5));
  }

  private static Pose2d poseFacing(Translation2d position, Translation2d target) {
    return new Pose2d(position, target.minus(position).getAngle());
  }

  private static Scenario scenarioFromPoints(
      String name,
      Translation2d start,
      Translation2d goal,
      List<ObstacleSpec> obstacles,
      double extraSeconds) {
    double direct = start.getDistance(goal);
    double maxSeconds = ReactiveBypassOptimizer.clamp(direct / 1.55 + extraSeconds, 4.5, 11.5);

    return new Scenario(
        name, poseFacing(start, goal), poseFacing(goal, start), obstacles, maxSeconds);
  }

  private static ObstacleSpec obstacleAt(Translation2d point, double radius, double strength) {
    Translation2d clamped = clampPointToField(point);
    return new ObstacleSpec(clamped.getX(), clamped.getY(), radius, strength);
  }
}
