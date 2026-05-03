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
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
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
  private static final double ROBOT_EFFECTIVE_RADIUS = Math.max(ROBOT_X, ROBOT_Y) * 0.5;
  private static final double CLEARANCE_BUFFER_METERS = 0.20;
  private static final double CANDIDATE_CUTOFF_PENALTY = 100_000.0;
  private static final double BROAD_COLLISION_SCENARIO_PENALTY = 80_000.0;
  private static final int MAX_COLLISION_STEPS_PER_SCENARIO = 20;
  private static final int MAX_CONSECUTIVE_BLOCKED_COLLISIONS = 8;
  private static final int MAX_MINED_HARD_CASES = 80;
  private static final int MAX_MINED_CASES_PER_TYPE = 3;

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
    return simulate(config, scenarios, profile, cutoffScore, ProgressReporter.disabled());
  }

  static Result simulate(
      ReactiveBypassConfig config,
      List<Scenario> scenarios,
      ScoreProfile profile,
      double cutoffScore,
      ProgressReporter progress) {
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

    int evaluatedScenarios = 0;
    for (Scenario scenario : scenarios) {
      EpisodeMetrics m = simulateScenario(config, scenario);
      evaluatedScenarios++;
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
        int unevaluated = scenarios.size() - evaluatedScenarios;
        failedScenarios += unevaluated;
        collisionScenarios += unevaluated;
        collisionSteps += unevaluated * MAX_COLLISION_STEPS_PER_SCENARIO;
        remaining += unevaluated;
        score += CANDIDATE_CUTOFF_PENALTY;
        score += unevaluated * profile.failureScenarioPenalty();
        score += unevaluated * profile.collisionScenarioPenalty();
        score += unevaluated * MAX_COLLISION_STEPS_PER_SCENARIO * profile.collisionStepPenalty();
        progress.finish();
        break;
      }

      progress.step();
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
    score += m.clearanceRisk() * profile.clearanceRiskPenalty();

    return score;
  }

  static List<ScenarioReportRow> evaluateScenarioRows(
      ReactiveBypassConfig config, List<Scenario> scenarios, ScoreProfile profile) {
    return evaluateScenarioRows(config, scenarios, profile, ProgressReporter.disabled());
  }

  static List<ScenarioReportRow> evaluateScenarioRows(
      ReactiveBypassConfig config,
      List<Scenario> scenarios,
      ScoreProfile profile,
      ProgressReporter progress) {
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
              m.minClearanceMeters(),
              m.clearanceRisk(),
              m.pathLengthMeters(),
              m.remainingMeters(),
              contribution));
      progress.step();
    }

    return rows;
  }

  static void writeScenarioReport(Path path, List<ScenarioReportRow> rows) throws IOException {
    StringBuilder out = new StringBuilder();
    out.append(
        "scenario,success,duration_s,blocked_s,pinned_s,collision_steps,min_clearance_m,clearance_risk,path_m,remaining_m,score_contribution\n");
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
          .append(String.format(Locale.US, "%.6f", row.minClearanceMeters()))
          .append(',')
          .append(String.format(Locale.US, "%.6f", row.clearanceRisk()))
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

  static EpisodeMetrics writeScenarioTrace(
      Path path, ReactiveBypassConfig config, Scenario scenario) throws IOException {
    StringBuilder out = new StringBuilder();
    out.append(
        "step,time_s,pose_x,pose_y,pose_deg,final_goal_x,final_goal_y,effective_goal_x,effective_goal_y,bypass_present,drive_heading_deg,desired_heading_deg,step_m,next_x,next_y,remaining_m,blocked_now,min_clearance,pinned,subgoal_x,subgoal_y,last_occ,preferred_side,consecutive_bypass_failures\n");
    EpisodeMetrics metrics = simulateScenario(config, scenario, out);
    Files.writeString(path, out.toString(), StandardCharsets.UTF_8);
    return metrics;
  }

  private static String csv(String value) {
    String escaped = value.replace("\"", "\"\"");
    return "\"" + escaped + "\"";
  }

  static void printWorstScenarios(List<ScenarioReportRow> rows, int maxRows) {
    if (rows.isEmpty()) return;

    Set<String> printedScenarios = new HashSet<>();
    int printed = 0;
    for (ScenarioReportRow row : rows) {
      if (row.success() && row.collisionSteps() == 0) continue;
      if (!printedScenarios.add(row.scenario())) continue;
      if (printed == 0) {
        System.out.println("worst scenarios:");
      }
      System.out.printf(
          Locale.US,
          "  %d. %s success=%s collisions=%d minClearance=%.3f risk=%.3f remaining=%.3f blocked=%.3f score=%.1f%n",
          printed + 1,
          row.scenario(),
          row.success(),
          row.collisionSteps(),
          row.minClearanceMeters(),
          row.clearanceRisk(),
          row.remainingMeters(),
          row.blockedSeconds(),
          row.scoreContribution());
      printed++;
      if (printed >= maxRows) break;
    }
  }

  static void printScenarioTypeSummary(List<ScenarioReportRow> rows, int maxTypes) {
    if (rows.isEmpty()) return;

    Map<String, ScenarioTypeStats> statsByType = new HashMap<>();
    for (ScenarioReportRow row : rows) {
      ScenarioTypeStats stats =
          statsByType.computeIfAbsent(
              scenarioKind(row.scenario()), ignored -> new ScenarioTypeStats());
      stats.count++;
      if (!row.success()) stats.failed++;
      if (row.collisionSteps() > 0) stats.collisionScenarios++;
      stats.collisionSteps += row.collisionSteps();
      stats.minClearanceMeters = Math.min(stats.minClearanceMeters, row.minClearanceMeters());
      stats.clearanceRisk += row.clearanceRisk();
      stats.score += row.scoreContribution();
    }

    List<Map.Entry<String, ScenarioTypeStats>> entries = new ArrayList<>(statsByType.entrySet());
    entries.sort(
        (a, b) -> {
          int cmp =
              Integer.compare(b.getValue().collisionScenarios, a.getValue().collisionScenarios);
          if (cmp != 0) return cmp;
          cmp = Integer.compare(b.getValue().collisionSteps, a.getValue().collisionSteps);
          if (cmp != 0) return cmp;
          cmp = Double.compare(b.getValue().clearanceRisk, a.getValue().clearanceRisk);
          if (cmp != 0) return cmp;
          return a.getKey().compareTo(b.getKey());
        });

    int limit = Math.min(maxTypes, entries.size());
    if (limit <= 0) return;

    System.out.println("scenario type pressure:");
    for (int i = 0; i < limit; i++) {
      Map.Entry<String, ScenarioTypeStats> entry = entries.get(i);
      ScenarioTypeStats stats = entry.getValue();
      System.out.printf(
          Locale.US,
          "  %d. %s count=%d failed=%d collisionScenarios=%d collisionSteps=%d minClearance=%.3f risk=%.3f score=%.1f%n",
          i + 1,
          entry.getKey(),
          stats.count,
          stats.failed,
          stats.collisionScenarios,
          stats.collisionSteps,
          stats.minClearanceMeters,
          stats.clearanceRisk,
          stats.score);
    }
  }

  private static String scenarioKind(String name) {
    String base = stripDuplicateScenarioSuffix(name);
    if (base.contains("narrow-corridor") || base.contains("corridor")) return "narrow-corridor";
    if (base.contains("double-gap")) return "double-gap";
    if (base.contains("corner")) return "corner-escape";
    if (base.contains("near-wall")) return "near-wall";
    if (base.contains("late-goal")) return "late-goal-blocker";
    if (base.contains("start-blocker")) return "start-blocker";
    if (base.contains("side-switch")) return "side-switch-trap";
    if (base.contains("center-clutter")) return "center-clutter";
    if (base.contains("diagonal")) return "diagonal-traffic";
    if (base.contains("offset")) return "offset-blocker";
    if (base.contains("short-shuttle")) return "short-shuttle";
    if (base.contains("straight")) return "straight-blocker";
    return base;
  }

  private static final class ScenarioTypeStats {
    int count;
    int failed;
    int collisionScenarios;
    int collisionSteps;
    double minClearanceMeters = Double.POSITIVE_INFINITY;
    double clearanceRisk;
    double score;
  }

  private static String stripDuplicateScenarioSuffix(String name) {
    int firstDash = name.indexOf('-');
    if (firstDash < 0) return name;

    String prefix = name.substring(0, firstDash);
    if (!prefix.equals("train") && !prefix.equals("hard") && !prefix.equals("validation")) {
      return name;
    }

    int secondDash = name.indexOf('-', firstDash + 1);
    if (secondDash < 0 || secondDash + 1 >= name.length()) return name;

    return name.substring(secondDash + 1);
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

    Map<String, Integer> minedTypeCounts = new HashMap<>();
    for (Scenario scenario : minedHardCases) {
      minedTypeCounts.merge(stripDuplicateScenarioSuffix(scenario.name()), 1, Integer::sum);
    }

    int added = 0;
    for (ScenarioReportRow row : rows) {
      if (row.success() && row.collisionSteps() == 0) continue;

      Scenario scenario = byName.get(row.scenario());
      if (scenario == null) continue;
      if (containsScenarioName(minedHardCases, scenario.name())) continue;

      String baseName = stripDuplicateScenarioSuffix(scenario.name());
      if (minedTypeCounts.getOrDefault(baseName, 0) >= MAX_MINED_CASES_PER_TYPE) continue;

      minedHardCases.add(scenario);
      minedTypeCounts.merge(baseName, 1, Integer::sum);
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

  private static double clearanceAt(
      Translation2d point, Rotation2d heading, List<ObstacleSpec> obstacles) {
    if (obstacles.isEmpty()) return Double.POSITIVE_INFINITY;

    double minClearance = Double.POSITIVE_INFINITY;
    for (ObstacleSpec obstacle : obstacles) {
      double clearance = circleToRobotRectClearance(point, heading, obstacle);
      minClearance = Math.min(minClearance, clearance);
    }
    return minClearance;
  }

  private static double circleToRobotRectClearance(
      Translation2d center, Rotation2d heading, ObstacleSpec obstacle) {
    double dx = obstacle.x() - center.getX();
    double dy = obstacle.y() - center.getY();
    double cos = Math.cos(-heading.getRadians());
    double sin = Math.sin(-heading.getRadians());
    double localX = dx * cos - dy * sin;
    double localY = dx * sin + dy * cos;
    double outsideX = Math.max(Math.abs(localX) - ROBOT_X * 0.5, 0.0);
    double outsideY = Math.max(Math.abs(localY) - ROBOT_Y * 0.5, 0.0);
    double outsideDistance = Math.hypot(outsideX, outsideY);
    if (outsideDistance > 0.0) {
      return outsideDistance - obstacle.radius();
    }

    double insideDistance =
        Math.min(ROBOT_X * 0.5 - Math.abs(localX), ROBOT_Y * 0.5 - Math.abs(localY));
    return -obstacle.radius() - insideDistance;
  }

  private static double clearanceRisk(double clearance, double dtSeconds) {
    double deficit = Math.max(0.0, CLEARANCE_BUFFER_METERS - clearance);
    return deficit * deficit * dtSeconds;
  }

  private static EpisodeMetrics simulateScenario(ReactiveBypassConfig cfg, Scenario scenario) {
    return simulateScenario(cfg, scenario, null);
  }

  private static EpisodeMetrics simulateScenario(
      ReactiveBypassConfig cfg, Scenario scenario, StringBuilder trace) {
    ReactiveBypassRuntime runtime = new ReactiveBypassRuntime(new ReactiveBypassConfig());
    runtime.setConfig(target -> copyConfigInto(cfg, target));

    PoseHolder poseHolder = new PoseHolder(scenario.start());
    DriveTuningHeat driveTuning = new DriveTuningHeat(() -> poseHolder.pose, Constants.FIELD);
    driveTuning.applyDefaults();
    HeadingGate headingGate = new HeadingGate();
    headingGate.reset(scenario.start().getRotation());

    List<ObstacleSpec> obstacleSpecs = scenario.obstacles();
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
    int consecutiveBlockedCollisions = 0;
    boolean lastCollision = false;
    double minClearance = clearanceAt(pose.getTranslation(), pose.getRotation(), obstacleSpecs);
    double clearanceRisk = 0.0;
    double pathLength = 0.0;
    double forwardProgress = 0.0;
    double bestRemaining = pose.getTranslation().getDistance(finalGoal);

    int steps = (int) Math.ceil(scenario.maxSeconds() / driveTuning.dtSeconds());
    for (int i = 0; i < steps; i++) {
      poseHolder.pose = pose;
      Force headingForce =
          syntheticForce(pose.getTranslation(), finalGoal, obstacles, obstacleSpecs, cfg);
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
      Force obstacleForce =
          syntheticObstacleForce(
              pose.getTranslation(), target, obstacles, scenario.obstacles(), cfg);
      boolean clearPathToTarget =
          ExtraPathing.isClearPath(
              "ReactiveBypassOptimizer/ClearTarget",
              pose.getTranslation(),
              target,
              obstacles,
              ROBOT_X,
              ROBOT_Y,
              false);
      obstacleForce =
          removeBackwardObstacleForceWhenClear(
              obstacleForce, pose.getTranslation(), target, clearPathToTarget);
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
      Translation2d attemptedNext = next;

      if (next.getDistance(pose.getTranslation()) < 1e-6
          && distanceToTarget > GOAL_TOLERANCE_METERS) {
        next = pose.getTranslation();
        attemptedNext = next;
      }

      boolean collision =
          ExtraPathing.robotIntersects(attemptedNext, desiredHeading, ROBOT_X, ROBOT_Y, obstacles);
      if (collision) {
        collisionSteps++;

        Translation2d halfNext = clampToField(pose.getTranslation().plus(step.times(0.5)), cfg);
        boolean halfCollision =
            ExtraPathing.robotIntersects(halfNext, desiredHeading, ROBOT_X, ROBOT_Y, obstacles);

        if (!halfCollision) {
          next = halfNext;
          consecutiveBlockedCollisions = 0;
        } else {
          Translation2d quarterNext =
              clampToField(pose.getTranslation().plus(step.times(0.25)), cfg);
          boolean quarterCollision =
              ExtraPathing.robotIntersects(
                  quarterNext, desiredHeading, ROBOT_X, ROBOT_Y, obstacles);

          if (!quarterCollision) {
            next = quarterNext;
            consecutiveBlockedCollisions = 0;
          } else {
            next = pose.getTranslation();
            consecutiveBlockedCollisions++;
          }
        }
      } else {
        consecutiveBlockedCollisions = 0;
      }

      double stepClearance =
          Math.min(
              clearanceAt(attemptedNext, desiredHeading, obstacleSpecs),
              clearanceAt(next, desiredHeading, obstacleSpecs));
      minClearance = Math.min(minClearance, stepClearance);
      clearanceRisk += clearanceRisk(stepClearance, driveTuning.dtSeconds());

      if (trace != null) {
        Pose2d subgoal = runtime.debugLatchedSubgoal();
        trace
            .append(i)
            .append(',')
            .append(String.format(Locale.US, "%.6f", duration))
            .append(',')
            .append(String.format(Locale.US, "%.6f", pose.getX()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", pose.getY()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", pose.getRotation().getDegrees()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", finalGoal.getX()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", finalGoal.getY()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", target.getX()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", target.getY()))
            .append(',')
            .append(bypass.isPresent())
            .append(',')
            .append(String.format(Locale.US, "%.6f", driveHeading.getDegrees()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", desiredHeading.getDegrees()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", stepMeters))
            .append(',')
            .append(String.format(Locale.US, "%.6f", next.getX()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", next.getY()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", next.getDistance(finalGoal)))
            .append(',')
            .append(blockedNow)
            .append(',')
            .append(String.format(Locale.US, "%.6f", stepClearance))
            .append(',')
            .append(runtime.isPinnedMode())
            .append(',')
            .append(subgoal == null ? "" : String.format(Locale.US, "%.6f", subgoal.getX()))
            .append(',')
            .append(subgoal == null ? "" : String.format(Locale.US, "%.6f", subgoal.getY()))
            .append(',')
            .append(String.format(Locale.US, "%.6f", runtime.debugLastOcc()))
            .append(',')
            .append(runtime.debugPreferredSide())
            .append(',')
            .append(runtime.debugConsecutiveBypassFailures())
            .append('\n');
      }

      if (collisionSteps >= MAX_COLLISION_STEPS_PER_SCENARIO
          || consecutiveBlockedCollisions >= MAX_CONSECUTIVE_BLOCKED_COLLISIONS) {
        return new EpisodeMetrics(
            false,
            duration,
            blockedSeconds,
            pinnedSeconds,
            collisionSteps,
            minClearance,
            clearanceRisk,
            pathLength,
            forwardProgress,
            bestRemaining);
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
            minClearance,
            clearanceRisk,
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
        minClearance,
        clearanceRisk,
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
      List<ObstacleSpec> obstacleSpecs,
      ReactiveBypassConfig cfg) {
    return getGoalForce(current, target)
        .plus(syntheticObstacleForce(current, target, dynamicObstacles, obstacleSpecs, cfg));
  }

  private static Force syntheticObstacleForce(
      Translation2d current,
      Translation2d target,
      List<? extends Obstacle> dynamicObstacles,
      List<ObstacleSpec> obstacleSpecs,
      ReactiveBypassConfig cfg) {
    Force force = Force.kZero;

    for (Obstacle obstacle : dynamicObstacles) {
      force = force.plus(obstacle.getForceAtPosition(current, target));
    }

    for (ObstacleSpec obstacle : obstacleSpecs) {
      Translation2d center = new Translation2d(obstacle.x(), obstacle.y());
      Translation2d away = current.minus(center);
      double distance = away.getNorm();
      if (distance < 1e-9 || distance > 4.0) continue;
      double clearance =
          distance - obstacle.radius() - ROBOT_EFFECTIVE_RADIUS - cfg.inflationMeters;
      double activeRange = CLEARANCE_BUFFER_METERS;
      if (clearance >= activeRange) continue;
      double mag =
          obstacle.strength()
              * Math.pow((activeRange - clearance) / activeRange, 2.0)
              * (1.0 + 1.0 / Math.max(0.08, clearance + activeRange));
      force = force.plus(new Force(mag, away.getAngle()));
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

  private static Force removeBackwardObstacleForceWhenClear(
      Force obstacleForce, Translation2d current, Translation2d target, boolean clearPath) {
    if (!clearPath || obstacleForce.getNorm() < 1e-9) return obstacleForce;

    Translation2d toTarget = target.minus(current);
    double distance = toTarget.getNorm();
    if (distance < 1e-9) return obstacleForce;

    double ux = toTarget.getX() / distance;
    double uy = toTarget.getY() / distance;
    double along = obstacleForce.getX() * ux + obstacleForce.getY() * uy;
    if (along >= 0.0) return obstacleForce;

    return new Force(obstacleForce.getX() - along * ux, obstacleForce.getY() - along * uy);
  }

  private static Force wallForce(double distance, double deadband, Rotation2d awayFromWall) {
    double clear = distance - deadband;
    if (clear >= 1.1) return Force.kZero;
    double mag = 0.55 / Math.max(0.05, clear * clear);
    return new Force(mag, awayFromWall);
  }
}
