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

import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassOptimizer.NORMAL_PROFILE;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassOptimizer.clamp;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassOptimizer.copyConfig;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassOptimizer.copyConfigInto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathing;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;
import org.curtinfrc.frc2026.util.Repulsor.HeadingGate;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DriveTuningHeat;

final class ReactiveBypassEvaluation {
  private static final double ROBOT_X = 0.85;
  private static final double ROBOT_Y = 0.85;
  private static final double DT_SECONDS = 0.02;
  private static final double MAX_SPEED_MPS = 3.7;
  private static final double GOAL_TOLERANCE_METERS = 0.22;
  private static final double CANDIDATE_CUTOFF_PENALTY = 100_000.0;
  private static final double BROAD_COLLISION_SCENARIO_PENALTY = 80_000.0;
  private static final int MAX_MINED_HARD_CASES = 80;

  private ReactiveBypassEvaluation() {}

  static Result simulate(ReactiveBypassConfig config, List<Scenario> scenarios) {
    return simulate(config, scenarios, NORMAL_PROFILE, Double.POSITIVE_INFINITY);
  }

  static Result simulate(
      ReactiveBypassConfig config, List<Scenario> scenarios, ScoreProfile profile) {
    return simulate(config, scenarios, profile, Double.POSITIVE_INFINITY);
  }

  static Result simulate(
      ReactiveBypassConfig config,
      List<Scenario> scenarios,
      ScoreProfile profile,
      double cutoffScore) {
    double score = 0.0;
    int successes = 0;
    int failedScenarios = 0;
    int collisionScenarios = 0;
    double duration = 0.0;
    double blocked = 0.0;
    double pinned = 0.0;
    int collisionSteps = 0;
    double pathLength = 0.0;
    double remaining = 0.0;

    for (Scenario scenario : scenarios) {
      EpisodeMetrics m = simulateScenario(config, scenario);
      successes += m.success() ? 1 : 0;
      if (!m.success()) failedScenarios++;
      if (m.collisionSteps() > 0) collisionScenarios++;

      duration += m.durationSeconds();
      blocked += m.blockedSeconds();
      pinned += m.pinnedSeconds();
      collisionSteps += m.collisionSteps();
      pathLength += m.pathLengthMeters();
      remaining += m.remainingMeters();

      double direct =
          scenario.start().getTranslation().getDistance(scenario.goal().getTranslation());
      double inefficiency = Math.max(0.0, m.pathLengthMeters() - direct);
      double progressLoss = Math.max(0.0, direct - m.forwardProgressMeters());

      score += scoreScenario(m, direct, inefficiency, progressLoss, profile);

      if (score > cutoffScore) {
        score += CANDIDATE_CUTOFF_PENALTY;
        break;
      }
    }

    score += collisionScenarios * BROAD_COLLISION_SCENARIO_PENALTY;

    return new Result(
        "",
        0,
        score,
        copyConfig(config),
        successes,
        failedScenarios,
        collisionScenarios,
        duration,
        blocked,
        pinned,
        collisionSteps,
        pathLength,
        remaining);
  }

  private static double scoreScenario(
      EpisodeMetrics m,
      double direct,
      double inefficiency,
      double progressLoss,
      ScoreProfile profile) {
    double score = 0.0;

    if (!m.success()) {
      score += profile.failureScenarioPenalty() + 1_500.0 * m.remainingMeters();
    }

    if (m.collisionSteps() > 0) {
      score += profile.collisionScenarioPenalty();
    }

    score += m.collisionSteps() * profile.collisionStepPenalty();
    score += m.pinnedSeconds() * profile.pinnedSecondPenalty();
    score += m.blockedSeconds() * profile.blockedSecondPenalty();
    score += m.durationSeconds() * profile.durationSecondPenalty();
    score += inefficiency * profile.inefficiencyPenalty();
    score += progressLoss * profile.progressLossPenalty();

    return score;
  }

  static List<ScenarioReportRow> evaluateScenarioRows(
      ReactiveBypassConfig config, List<Scenario> scenarios, ScoreProfile profile) {
    List<ScenarioReportRow> rows = new ArrayList<>();

    for (Scenario scenario : scenarios) {
      EpisodeMetrics m = simulateScenario(config, scenario);
      double direct =
          scenario.start().getTranslation().getDistance(scenario.goal().getTranslation());
      double inefficiency = Math.max(0.0, m.pathLengthMeters() - direct);
      double progressLoss = Math.max(0.0, direct - m.forwardProgressMeters());
      double contribution = scoreScenario(m, direct, inefficiency, progressLoss, profile);

      rows.add(
          new ScenarioReportRow(
              scenario.name(),
              m.success(),
              m.durationSeconds(),
              m.blockedSeconds(),
              m.pinnedSeconds(),
              m.collisionSteps(),
              m.pathLengthMeters(),
              m.remainingMeters(),
              contribution));
    }

    return rows;
  }

  static void writeScenarioReport(Path path, List<ScenarioReportRow> rows) throws IOException {
    StringBuilder out = new StringBuilder();
    out.append(
        "scenario,success,duration_s,blocked_s,pinned_s,collision_steps,path_m,remaining_m,score_contribution\n");
    for (ScenarioReportRow row : rows) {
      out.append(csv(row.scenario()))
          .append(',')
          .append(row.success())
          .append(',')
          .append(String.format(Locale.US, "%.6f", row.durationSeconds()))
          .append(',')
          .append(String.format(Locale.US, "%.6f", row.blockedSeconds()))
          .append(',')
          .append(String.format(Locale.US, "%.6f", row.pinnedSeconds()))
          .append(',')
          .append(row.collisionSteps())
          .append(',')
          .append(String.format(Locale.US, "%.6f", row.pathLengthMeters()))
          .append(',')
          .append(String.format(Locale.US, "%.6f", row.remainingMeters()))
          .append(',')
          .append(String.format(Locale.US, "%.6f", row.scoreContribution()))
          .append('\n');
    }
    Files.writeString(path, out.toString(), StandardCharsets.UTF_8);
  }

  private static String csv(String value) {
    String escaped = value.replace("\"", "\"\"");
    return "\"" + escaped + "\"";
  }

  static void printWorstScenarios(List<ScenarioReportRow> rows, int maxRows) {
    if (rows.isEmpty()) return;

    int printed = 0;
    for (ScenarioReportRow row : rows) {
      if (row.success() && row.collisionSteps() == 0) continue;
      if (printed == 0) {
        System.out.println("worst scenarios:");
      }
      System.out.printf(
          Locale.US,
          "  %d. %s success=%s collisions=%d remaining=%.3f blocked=%.3f score=%.1f%n",
          printed + 1,
          row.scenario(),
          row.success(),
          row.collisionSteps(),
          row.remainingMeters(),
          row.blockedSeconds(),
          row.scoreContribution());
      printed++;
      if (printed >= maxRows) break;
    }
  }

  private static boolean containsScenarioBaseName(List<Scenario> scenarios, String baseName) {
    for (Scenario scenario : scenarios) {
      if (stripDuplicateScenarioSuffix(scenario.name()).equals(baseName)) return true;
    }
    return false;
  }

  private static String stripDuplicateScenarioSuffix(String name) {
    return name;
  }

  static void addMinedHardCases(
      List<Scenario> minedHardCases,
      List<Scenario> activeScenarios,
      List<ScenarioReportRow> rows,
      int maxNewCases) {
    Map<String, Scenario> byName = new HashMap<>();
    for (Scenario scenario : activeScenarios) {
      byName.putIfAbsent(scenario.name(), scenario);
    }

    int added = 0;
    for (ScenarioReportRow row : rows) {
      if (row.success() && row.collisionSteps() == 0) continue;

      Scenario scenario = byName.get(row.scenario());
      if (scenario == null) continue;
      if (containsScenarioName(minedHardCases, scenario.name())) continue;

      String baseName = stripDuplicateScenarioSuffix(scenario.name());
      if (containsScenarioBaseName(minedHardCases, baseName)) continue;

      minedHardCases.add(scenario);
      added++;
      if (added >= maxNewCases) break;
    }

    while (minedHardCases.size() > MAX_MINED_HARD_CASES) {
      minedHardCases.remove(0);
    }
  }

  private static boolean containsScenarioName(List<Scenario> scenarios, String name) {
    for (Scenario scenario : scenarios) {
      if (scenario.name().equals(name)) return true;
    }
    return false;
  }

  static boolean safetyBetter(Result candidate, Result incumbent) {
    if (candidate.collisionScenarios() != incumbent.collisionScenarios()) {
      return candidate.collisionScenarios() < incumbent.collisionScenarios();
    }
    if (candidate.collisionSteps() != incumbent.collisionSteps()) {
      return candidate.collisionSteps() < incumbent.collisionSteps();
    }
    if (candidate.successes() != incumbent.successes()) {
      return candidate.successes() > incumbent.successes();
    }
    if (candidate.failedScenarios() != incumbent.failedScenarios()) {
      return candidate.failedScenarios() < incumbent.failedScenarios();
    }
    return candidate.score() < incumbent.score();
  }

  private static EpisodeMetrics simulateScenario(ReactiveBypassConfig cfg, Scenario scenario) {
    ReactiveBypassRuntime runtime = new ReactiveBypassRuntime(new ReactiveBypassConfig());
    runtime.setConfig(target -> copyConfigInto(cfg, target));

    PoseHolder poseHolder = new PoseHolder(scenario.start());
    DriveTuningHeat driveTuning = new DriveTuningHeat(() -> poseHolder.pose, Constants.FIELD);
    driveTuning.applyDefaults();
    HeadingGate headingGate = new HeadingGate();
    headingGate.reset(scenario.start().getRotation());

    List<Obstacle> obstacles = scenario.newObstacles();
    Function<Translation2d[], Boolean> intersectsDynamicOnly =
        rect -> {
          for (Obstacle obstacle : obstacles) {
            if (obstacle.intersectsRectangle(rect)) return true;
          }
          return false;
        };

    Pose2d pose = scenario.start();
    Translation2d startTranslation = pose.getTranslation();
    Translation2d finalGoal = scenario.goal().getTranslation();
    Translation2d last = pose.getTranslation();
    double duration = 0.0;
    double blockedSeconds = 0.0;
    double pinnedSeconds = 0.0;
    int collisionSteps = 0;
    boolean lastCollision = false;
    double pathLength = 0.0;
    double forwardProgress = 0.0;
    double bestRemaining = pose.getTranslation().getDistance(finalGoal);

    int steps = (int) Math.ceil(scenario.maxSeconds() / driveTuning.dtSeconds());
    for (int i = 0; i < steps; i++) {
      poseHolder.pose = pose;
      Force headingForce = syntheticForce(pose.getTranslation(), finalGoal, obstacles, cfg);
      Rotation2d heading =
          headingForce.getNorm() > 1e-9
              ? headingForce.getAngle()
              : finalGoal.minus(pose.getTranslation()).getAngle();

      boolean blockedNow =
          !ExtraPathing.isClearPath(
              "ReactiveBypassOptimizer/Blocked",
              pose.getTranslation(),
              finalGoal,
              obstacles,
              ROBOT_X,
              ROBOT_Y,
              false);
      if (blockedNow) blockedSeconds += driveTuning.dtSeconds();

      Translation2d currentTranslation = pose.getTranslation();
      Optional<Pose2d> bypass =
          runtime.update(
              pose,
              scenario.goal(),
              heading,
              driveTuning.dtSeconds(),
              ROBOT_X,
              ROBOT_Y,
              obstacles,
              intersectsDynamicOnly,
              tag ->
                  ExtraPathing.isClearPath(
                      "ReactiveBypassOptimizer/Rejoin",
                      currentTranslation,
                      finalGoal,
                      obstacles,
                      ROBOT_X,
                      ROBOT_Y,
                      false));

      if (runtime.isPinnedMode()) pinnedSeconds += driveTuning.dtSeconds();

      Pose2d effectiveGoal = bypass.orElse(scenario.goal());
      Translation2d target = effectiveGoal.getTranslation();
      Force obstacleForce = syntheticObstacleForce(pose.getTranslation(), target, obstacles, cfg);
      Force driveForce = getGoalForce(pose.getTranslation(), target).plus(obstacleForce);
      Rotation2d driveHeading =
          driveForce.getNorm() > 1e-9
              ? driveForce.getAngle()
              : target.minus(pose.getTranslation()).getAngle();

      double distanceToTarget = pose.getTranslation().getDistance(target);
      double stepMeters =
          driveTuning.stepSizeMeters(distanceToTarget, obstacleForce.getNorm(), false, blockedNow);
      Rotation2d desiredHeading =
          headingGate.filter(pose.getRotation(), driveHeading, driveTuning.dtSeconds());
      Translation2d step = new Translation2d(stepMeters, driveHeading);
      Translation2d next = clampToField(pose.getTranslation().plus(step), cfg);

      if (next.getDistance(pose.getTranslation()) < 1e-6
          && distanceToTarget > GOAL_TOLERANCE_METERS) {
        next = pose.getTranslation();
      }

      boolean collision = ExtraPathing.robotIntersects(next, ROBOT_X, ROBOT_Y, obstacles);
      if (collision) {
        collisionSteps++;

        Translation2d halfNext = clampToField(pose.getTranslation().plus(step.times(0.5)), cfg);
        boolean halfCollision = ExtraPathing.robotIntersects(halfNext, ROBOT_X, ROBOT_Y, obstacles);

        if (!halfCollision) {
          next = halfNext;
        } else {
          Translation2d quarterNext =
              clampToField(pose.getTranslation().plus(step.times(0.25)), cfg);
          boolean quarterCollision =
              ExtraPathing.robotIntersects(quarterNext, ROBOT_X, ROBOT_Y, obstacles);

          if (!quarterCollision) {
            next = quarterNext;
          } else {
            next = pose.getTranslation();
          }
        }
      }

      lastCollision = collision;
      pathLength += next.getDistance(last);
      pose = new Pose2d(next, desiredHeading);
      last = next;
      duration += driveTuning.dtSeconds();

      Translation2d startToGoal = finalGoal.minus(startTranslation);
      double denom = Math.max(startToGoal.getNorm(), 1e-9);
      Translation2d startToPose = next.minus(startTranslation);
      forwardProgress =
          Math.max(
              forwardProgress,
              (startToPose.getX() * startToGoal.getX() + startToPose.getY() * startToGoal.getY())
                  / denom);
      bestRemaining = Math.min(bestRemaining, next.getDistance(finalGoal));

      if (next.getDistance(finalGoal) <= GOAL_TOLERANCE_METERS) {
        return new EpisodeMetrics(
            true,
            duration,
            blockedSeconds,
            pinnedSeconds,
            collisionSteps,
            pathLength,
            forwardProgress,
            next.getDistance(finalGoal));
      }
    }

    return new EpisodeMetrics(
        false,
        duration,
        blockedSeconds,
        pinnedSeconds,
        collisionSteps,
        pathLength,
        forwardProgress,
        bestRemaining);
  }

  private static Translation2d clampToField(Translation2d point, ReactiveBypassConfig cfg) {
    return new Translation2d(
        clamp(point.getX(), ROBOT_X * 0.5, cfg.fieldLen - ROBOT_X * 0.5),
        clamp(point.getY(), ROBOT_Y * 0.5, cfg.fieldWid - ROBOT_Y * 0.5));
  }

  private static Force syntheticForce(
      Translation2d current,
      Translation2d target,
      List<? extends Obstacle> dynamicObstacles,
      ReactiveBypassConfig cfg) {
    return getGoalForce(current, target)
        .plus(syntheticObstacleForce(current, target, dynamicObstacles, cfg));
  }

  private static Force syntheticObstacleForce(
      Translation2d current,
      Translation2d target,
      List<? extends Obstacle> dynamicObstacles,
      ReactiveBypassConfig cfg) {
    Force force = Force.kZero;

    for (Obstacle obstacle : dynamicObstacles) {
      force = force.plus(obstacle.getForceAtPosition(current, target));
    }

    force = force.plus(wallForce(current.getX(), 0.0, Rotation2d.kZero));
    force = force.plus(wallForce(cfg.fieldLen - current.getX(), 0.0, Rotation2d.k180deg));
    force = force.plus(wallForce(current.getY(), 0.0, Rotation2d.kCCW_90deg));
    force = force.plus(wallForce(cfg.fieldWid - current.getY(), 0.0, Rotation2d.kCW_90deg));

    return force;
  }

  private static Force getGoalForce(Translation2d current, Translation2d target) {
    Translation2d toTarget = target.minus(current);
    return toTarget.getNorm() > 1e-9
        ? new Force(1.0 + 1.0 / (0.2 + toTarget.getNorm()), toTarget.getAngle())
        : Force.kZero;
  }

  private static Force wallForce(double distance, double deadband, Rotation2d awayFromWall) {
    double clear = distance - deadband;
    if (clear >= 1.1) return Force.kZero;
    double mag = 0.55 / Math.max(0.05, clear * clear);
    return new Force(mag, awayFromWall);
  }
}
