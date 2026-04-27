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
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathing;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PointObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

/** Offline optimizer for {@code ReactiveBypassConfig.yaml}. */
public final class ReactiveBypassOptimizer {
  private static final double ROBOT_X = 0.85;
  private static final double ROBOT_Y = 0.85;
  private static final double DT_SECONDS = 0.02;
  private static final double MAX_SPEED_MPS = 3.7;
  private static final double GOAL_TOLERANCE_METERS = 0.22;

  private static final List<Tunable> TUNABLES =
      List.of(
          d("triggerAheadMeters", 0.60, 2.50),
          d("triggerWidthMeters", 0.30, 1.25),
          i("corridorSamples", 3, 21),
          d("occHigh", 0.12, 0.85),
          d("occLow", 0.01, 0.25),
          d("lateralMeters", 0.25, 1.45),
          d("lateralMaxMeters", 1.10, 3.20),
          d("minLateralMeters", 0.12, 0.85),
          d("forwardMeters", 0.70, 2.80),
          d("forwardMaxMeters", 1.40, 3.60),
          d("holdMinSeconds", 0.05, 0.85),
          d("recalcSeconds", 0.02, 0.55),
          d("subgoalMaxSeconds", 0.45, 3.50),
          d("minHeadingErrToTriggerDeg", 0.0, 28.0),
          d("releaseAheadMeters", 0.70, 2.70),
          d("angleCostWeight", 0.0, 1.80),
          d("curvatureWeight", 0.0, 1.80),
          d("wallPenaltyGain", 0.0, 1.50),
          d("occCostGain", 0.05, 4.00),
          d("sideSwitchPenalty", 0.0, 2.50),
          d("headingDeadbandDeg", 0.0, 12.0),
          d("probeOccLegStep", 0.08, 0.65),
          d("sideBiasProbeAngleDeg", 5.0, 38.0),
          d("sideStickSeconds", 0.05, 1.20),
          d("subgoalSlewRateMps", 1.00, 6.00),
          d("subgoalJitterMeters", 0.0, 0.18),
          d("arcAngleDeg", 10.0, 50.0),
          d("arcRMin", 0.25, 1.30),
          d("arcRMax", 0.80, 2.40),
          i("arcRadialSteps", 2, 5),
          i("arcAngularStepsPerSide", 1, 4),
          d("relatchImproveFrac", 0.02, 0.35),
          d("zzzMaxYawDeltaDeg", 12.0, 55.0),
          d("zzzPenalty", 0.0, 1.20),
          d("vibWindowS", 0.18, 1.10),
          d("vibMinDisp", 0.04, 0.35),
          i("vibMaxDirFlips", 3, 12),
          d("escapeForward", 1.00, 2.80),
          d("escapeLateral", 0.80, 2.80),
          d("escapeHoldS", 0.10, 1.20),
          d("escapeOccBoost", 0.0, 0.85),
          d("progressCostWeight", 0.0, 0.80),
          d("minForwardProgressMeters", 0.05, 0.90),
          d("nearGoalDistMeters", 0.70, 4.00),
          d("nearGoalForwardScale", 0.15, 1.00),
          d("nearGoalLateralScale", 0.25, 1.25),
          d("stuckMinForwardProgress", 0.05, 0.60),
          d("stuckOccMin", 0.12, 0.80),
          d("stuckLookbackFrac", 0.35, 0.90),
          d("sideSwitchStuckPenaltyScale", 0.0, 1.10),
          d("pinnedOccMin", 0.15, 0.85),
          d("pinnedMinTimeSeconds", 0.10, 1.00),
          d("pinnedMaxTimeSeconds", 1.20, 5.50),
          d("pinnedCooldownSeconds", 0.20, 2.00),
          d("cornerWallThresh", 1.50, 7.00),
          d("cornerEscapeLatBoost", 1.00, 3.60),
          d("cornerEscapeFwdBoost", 0.80, 2.60),
          d("cornerEscapeBlend", 0.20, 1.00),
          d("cornerRewardGain", 0.0, 1.40));

  private ReactiveBypassOptimizer() {}

  public static void main(String[] args) throws Exception {
    Options options = Options.parse(args);
    Path outputDir = options.outputDir();
    Files.createDirectories(outputDir);

    if (System.getProperty("repulsor.deploy.dir") == null) {
      System.setProperty("repulsor.deploy.dir", options.configPath().getParent().toString());
    }

    ReactiveBypassConfig base = new ReactiveBypassConfig();
    ReactiveBypassConfigLoader.loadConfigFromYaml(base, ReactiveBypassConfig.class);

    List<Scenario> scenarios = buildScenarios();
    Random random = new Random(options.seed());
    OptimizerState optimizer = new OptimizerState(base, scenarios, random);

    List<Result> history = new ArrayList<>();
    Result baseline = optimizer.evaluate(optimizer.currentVector(), "baseline", 0);
    history.add(baseline);
    Result best = baseline;
    System.out.println("ReactiveBypass optimizer");
    System.out.printf(
        Locale.US,
        "baseline score=%.3f success=%d/%d duration=%.3f collisionSteps=%d%n",
        baseline.score(),
        baseline.successes(),
        scenarios.size(),
        baseline.durationSeconds(),
        baseline.collisionSteps());

    int generations = Math.max(5, Math.min(30, options.iterations() / 16));
    int population = Math.max(8, options.iterations() / generations);
    double[] mean = optimizer.currentVector();
    double[] sigma = optimizer.initialSigma();
    int evaluations = 0;

    for (int generation = 1;
        generation <= generations && evaluations < options.iterations();
        generation++) {
      List<Result> generationResults = new ArrayList<>();
      generationResults.add(optimizer.evaluate(mean, "mean", generation));
      while (generationResults.size() < population && evaluations < options.iterations()) {
        double[] sample = optimizer.sample(mean, sigma);
        Result result = optimizer.evaluate(sample, "sample", generation);
        generationResults.add(result);
        evaluations++;
      }
      generationResults.sort(Comparator.comparingDouble(Result::score));
      Result generationBest = generationResults.get(0);
      if (generationBest.score() < best.score()) {
        best = generationBest;
      }
      history.addAll(generationResults);

      int eliteCount =
          Math.min(generationResults.size(), Math.max(1, generationResults.size() / 5));
      optimizer.updateDistribution(generationResults.subList(0, eliteCount), mean, sigma);
      System.out.printf(
          Locale.US,
          "gen=%02d best=%.3f overall=%.3f success=%d/%d sigma=%.4f%n",
          generation,
          generationBest.score(),
          best.score(),
          best.successes(),
          scenarios.size(),
          average(sigma));
    }

    best = optimizer.polish(best, mean, sigma, history);

    Path resultsPath = outputDir.resolve("results.csv");
    writeResults(resultsPath, history);
    Path bestPath = outputDir.resolve("best.yaml");
    writeYaml(bestPath, best.config());

    System.out.printf(
        Locale.US,
        "best score=%.3f success=%d/%d duration=%.3f blocked=%.3f pinned=%.3f collisionSteps=%d%n",
        best.score(),
        best.successes(),
        scenarios.size(),
        best.durationSeconds(),
        best.blockedSeconds(),
        best.pinnedSeconds(),
        best.collisionSteps());
    System.out.println("wrote " + bestPath.toAbsolutePath());
    System.out.println("wrote " + resultsPath.toAbsolutePath());

    if (options.apply()) {
      writeYaml(options.configPath(), best.config());
      System.out.println("updated " + options.configPath().toAbsolutePath());
    } else {
      System.out.println("deploy YAML left unchanged; rerun with -PoptimizerApply=true to apply");
    }
  }

  private static List<Scenario> buildScenarios() {
    return List.of(
        new Scenario(
            "center-single-block",
            new Pose2d(1.7, 4.0, Rotation2d.kZero),
            new Pose2d(14.4, 4.0, Rotation2d.kZero),
            List.of(new ObstacleSpec(7.7, 4.0, 0.48, 1.2)),
            8.0),
        new Scenario(
            "center-double-gap-top",
            new Pose2d(2.1, 3.4, Rotation2d.kZero),
            new Pose2d(14.2, 3.5, Rotation2d.kZero),
            List.of(new ObstacleSpec(7.1, 3.25, 0.45, 1.2), new ObstacleSpec(8.2, 4.25, 0.45, 1.2)),
            8.5),
        new Scenario(
            "offset-left-bypass",
            new Pose2d(2.6, 2.1, Rotation2d.kZero),
            new Pose2d(13.0, 2.8, Rotation2d.kZero),
            List.of(new ObstacleSpec(7.0, 2.35, 0.58, 1.4)),
            8.0),
        new Scenario(
            "offset-right-bypass",
            new Pose2d(13.8, 5.8, Rotation2d.k180deg),
            new Pose2d(2.3, 5.0, Rotation2d.k180deg),
            List.of(new ObstacleSpec(8.4, 5.55, 0.58, 1.4)),
            8.5),
        new Scenario(
            "near-wall-pinned",
            new Pose2d(1.2, 1.0, Rotation2d.kZero),
            new Pose2d(6.2, 1.05, Rotation2d.kZero),
            List.of(new ObstacleSpec(3.0, 1.05, 0.52, 1.5)),
            7.0),
        new Scenario(
            "corner-release",
            new Pose2d(14.9, 1.0, Rotation2d.k180deg),
            new Pose2d(10.2, 2.3, Rotation2d.k180deg),
            List.of(
                new ObstacleSpec(13.7, 1.15, 0.45, 1.2), new ObstacleSpec(13.0, 1.75, 0.40, 1.2)),
            7.5),
        new Scenario(
            "diagonal-crossing",
            new Pose2d(3.0, 6.4, Rotation2d.kZero),
            new Pose2d(13.5, 1.7, Rotation2d.kZero),
            List.of(new ObstacleSpec(7.0, 4.55, 0.46, 1.3), new ObstacleSpec(8.2, 3.80, 0.46, 1.3)),
            9.0),
        new Scenario(
            "late-obstacle-near-goal",
            new Pose2d(2.0, 4.2, Rotation2d.kZero),
            new Pose2d(11.5, 4.2, Rotation2d.kZero),
            List.of(new ObstacleSpec(10.0, 4.2, 0.42, 1.2)),
            7.0));
  }

  private static Result simulate(ReactiveBypassConfig config, List<Scenario> scenarios) {
    double score = 0.0;
    int successes = 0;
    double duration = 0.0;
    double blocked = 0.0;
    double pinned = 0.0;
    int collisionSteps = 0;
    double pathLength = 0.0;
    double remaining = 0.0;

    for (Scenario scenario : scenarios) {
      EpisodeMetrics m = simulateScenario(config, scenario);
      successes += m.success() ? 1 : 0;
      duration += m.durationSeconds();
      blocked += m.blockedSeconds();
      pinned += m.pinnedSeconds();
      collisionSteps += m.collisionSteps();
      pathLength += m.pathLengthMeters();
      remaining += m.remainingMeters();

      double direct =
          scenario.start().getTranslation().getDistance(scenario.goal().getTranslation());
      double inefficiency = Math.max(0.0, m.pathLengthMeters() - direct);
      score += m.success() ? 0.0 : 15_000.0 + 1_200.0 * m.remainingMeters();
      score += m.collisionSteps() * 900.0;
      score += m.durationSeconds() * 8.0;
      score += m.blockedSeconds() * 35.0;
      score += m.pinnedSeconds() * 42.0;
      score += inefficiency * 28.0;
      score += Math.max(0.0, direct - m.forwardProgressMeters()) * 140.0;
    }

    return new Result(
        "",
        0,
        score,
        copyConfig(config),
        successes,
        duration,
        blocked,
        pinned,
        collisionSteps,
        pathLength,
        remaining);
  }

  private static EpisodeMetrics simulateScenario(ReactiveBypassConfig cfg, Scenario scenario) {
    ReactiveBypassRuntime runtime = new ReactiveBypassRuntime(new ReactiveBypassConfig());
    runtime.setConfig(target -> copyConfigInto(cfg, target));

    List<Obstacle> obstacles = scenario.newObstacles();
    Function<Translation2d[], Boolean> intersectsDynamicOnly =
        rect -> {
          for (Obstacle obstacle : obstacles) {
            if (obstacle.intersectsRectangle(rect)) return true;
          }
          return false;
        };

    Pose2d pose = scenario.start();
    Translation2d start = pose.getTranslation();
    Translation2d goal = scenario.goal().getTranslation();
    Translation2d last = pose.getTranslation();
    double duration = 0.0;
    double blockedSeconds = 0.0;
    double pinnedSeconds = 0.0;
    int collisionSteps = 0;
    boolean lastCollision = false;
    double pathLength = 0.0;
    double forwardProgress = 0.0;
    double bestRemaining = pose.getTranslation().getDistance(goal);

    int steps = (int) Math.ceil(scenario.maxSeconds() / DT_SECONDS);
    for (int i = 0; i < steps; i++) {
      Force headingForce = syntheticForce(pose.getTranslation(), goal, obstacles, cfg);
      Rotation2d heading =
          headingForce.getNorm() > 1e-9
              ? headingForce.getAngle()
              : goal.minus(pose.getTranslation()).getAngle();

      boolean blockedNow =
          !ExtraPathing.isClearPath(
              "ReactiveBypassOptimizer/Blocked",
              pose.getTranslation(),
              goal,
              obstacles,
              ROBOT_X,
              ROBOT_Y,
              false);
      if (blockedNow) blockedSeconds += DT_SECONDS;

      Translation2d currentTranslation = pose.getTranslation();
      Optional<Pose2d> bypass =
          runtime.update(
              pose,
              scenario.goal(),
              heading,
              DT_SECONDS,
              ROBOT_X,
              ROBOT_Y,
              obstacles,
              intersectsDynamicOnly,
              tag ->
                  ExtraPathing.isClearPath(
                      "ReactiveBypassOptimizer/Rejoin",
                      currentTranslation,
                      goal,
                      obstacles,
                      ROBOT_X,
                      ROBOT_Y,
                      false));

      if (runtime.isPinnedMode()) pinnedSeconds += DT_SECONDS;

      Translation2d target = bypass.orElse(scenario.goal()).getTranslation();
      Force driveForce = syntheticForce(pose.getTranslation(), target, obstacles, cfg);
      Rotation2d driveHeading =
          driveForce.getNorm() > 1e-9
              ? driveForce.getAngle()
              : target.minus(pose.getTranslation()).getAngle();
      double distanceToTarget = pose.getTranslation().getDistance(target);
      double stepMeters = Math.min(MAX_SPEED_MPS * DT_SECONDS, distanceToTarget);
      Translation2d step = new Translation2d(stepMeters, driveHeading);
      Translation2d next = clampToField(pose.getTranslation().plus(step), cfg);
      if (next.getDistance(pose.getTranslation()) < 1e-6
          && distanceToTarget > GOAL_TOLERANCE_METERS) {
        next = pose.getTranslation();
      }

      boolean collision = ExtraPathing.robotIntersects(next, ROBOT_X, ROBOT_Y, obstacles);
      if (collision) {
        collisionSteps++;
        if (lastCollision) {
          next = pose.getTranslation();
        }
      }
      lastCollision = collision;

      pathLength += next.getDistance(last);
      pose = new Pose2d(next, driveHeading);
      last = next;
      duration += DT_SECONDS;

      Translation2d startToGoal = goal.minus(start);
      double denom = Math.max(startToGoal.getNorm(), 1e-9);
      Translation2d startToPose = next.minus(start);
      forwardProgress =
          Math.max(
              forwardProgress,
              (startToPose.getX() * startToGoal.getX() + startToPose.getY() * startToGoal.getY())
                  / denom);
      bestRemaining = Math.min(bestRemaining, next.getDistance(goal));

      if (next.getDistance(goal) <= GOAL_TOLERANCE_METERS) {
        return new EpisodeMetrics(
            true,
            duration,
            blockedSeconds,
            pinnedSeconds,
            collisionSteps,
            pathLength,
            forwardProgress,
            next.getDistance(goal));
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
    Translation2d toTarget = target.minus(current);
    Force force =
        toTarget.getNorm() > 1e-9
            ? new Force(1.0 + 1.0 / (0.2 + toTarget.getNorm()), toTarget.getAngle())
            : Force.kZero;

    for (Obstacle obstacle : dynamicObstacles) {
      force = force.plus(obstacle.getForceAtPosition(current, target));
    }

    force = force.plus(wallForce(current.getX(), 0.0, Rotation2d.kZero));
    force = force.plus(wallForce(cfg.fieldLen - current.getX(), 0.0, Rotation2d.k180deg));
    force = force.plus(wallForce(current.getY(), 0.0, Rotation2d.kCCW_90deg));
    force = force.plus(wallForce(cfg.fieldWid - current.getY(), 0.0, Rotation2d.kCW_90deg));
    return force;
  }

  private static Force wallForce(double distance, double deadband, Rotation2d awayFromWall) {
    double clear = distance - deadband;
    if (clear >= 1.1) return Force.kZero;
    double mag = 0.55 / Math.max(0.05, clear * clear);
    return new Force(mag, awayFromWall);
  }

  private static void writeResults(Path path, List<Result> results) throws IOException {
    StringBuilder out = new StringBuilder();
    out.append(
        "label,generation,score,successes,duration_s,blocked_s,pinned_s,collision_steps,path_m,remaining_m\n");
    for (Result result : results) {
      out.append(
          String.format(
              Locale.US,
              "%s,%d,%.6f,%d,%.6f,%.6f,%.6f,%d,%.6f,%.6f%n",
              result.label(),
              result.generation(),
              result.score(),
              result.successes(),
              result.durationSeconds(),
              result.blockedSeconds(),
              result.pinnedSeconds(),
              result.collisionSteps(),
              result.pathLengthMeters(),
              result.remainingMeters()));
    }
    Files.writeString(path, out.toString(), StandardCharsets.UTF_8);
  }

  private static void writeYaml(Path path, ReactiveBypassConfig cfg) throws IOException {
    StringBuilder out = new StringBuilder();
    for (Field field : ReactiveBypassConfig.class.getFields()) {
      try {
        if (field.getType() == int.class) {
          out.append(field.getName()).append(": ").append(field.getInt(cfg)).append('\n');
        } else if (field.getType() == double.class) {
          out.append(field.getName())
              .append(": ")
              .append(String.format(Locale.US, "%.15g", field.getDouble(cfg)))
              .append('\n');
        } else if (field.getType() == boolean.class) {
          out.append(field.getName()).append(": ").append(field.getBoolean(cfg)).append('\n');
        }
      } catch (IllegalAccessException e) {
        throw new IOException("Failed to read config field " + field.getName(), e);
      }
    }
    Files.writeString(path, out.toString(), StandardCharsets.UTF_8);
  }

  private static double average(double[] values) {
    double sum = 0.0;
    for (double value : values) sum += value;
    return values.length == 0 ? 0.0 : sum / values.length;
  }

  private static ReactiveBypassConfig copyConfig(ReactiveBypassConfig source) {
    ReactiveBypassConfig copy = new ReactiveBypassConfig();
    copyConfigInto(source, copy);
    return copy;
  }

  private static void copyConfigInto(ReactiveBypassConfig source, ReactiveBypassConfig target) {
    for (Field field : ReactiveBypassConfig.class.getFields()) {
      try {
        if (field.getType() == int.class) {
          field.setInt(target, field.getInt(source));
        } else if (field.getType() == double.class) {
          field.setDouble(target, field.getDouble(source));
        } else if (field.getType() == boolean.class) {
          field.setBoolean(target, field.getBoolean(source));
        }
      } catch (IllegalAccessException e) {
        throw new IllegalStateException("Failed to copy config field " + field.getName(), e);
      }
    }
  }

  private static double readDouble(ReactiveBypassConfig cfg, String name) {
    try {
      return ReactiveBypassConfig.class.getField(name).getDouble(cfg);
    } catch (ReflectiveOperationException e) {
      throw new IllegalArgumentException("Unknown double config field " + name, e);
    }
  }

  private static int readInt(ReactiveBypassConfig cfg, String name) {
    try {
      return ReactiveBypassConfig.class.getField(name).getInt(cfg);
    } catch (ReflectiveOperationException e) {
      throw new IllegalArgumentException("Unknown int config field " + name, e);
    }
  }

  private static void writeValue(ReactiveBypassConfig cfg, Tunable tunable, double value) {
    try {
      Field field = ReactiveBypassConfig.class.getField(tunable.name());
      if (tunable.integer()) {
        field.setInt(cfg, (int) Math.round(value));
      } else {
        field.setDouble(cfg, value);
      }
    } catch (ReflectiveOperationException e) {
      throw new IllegalArgumentException("Unknown config field " + tunable.name(), e);
    }
  }

  private static void repair(ReactiveBypassConfig cfg) {
    cfg.occLow = Math.min(cfg.occLow, cfg.occHigh * 0.75);
    cfg.occHigh = Math.max(cfg.occHigh, cfg.occLow + 0.03);
    cfg.minLateralMeters = Math.min(cfg.minLateralMeters, cfg.lateralMaxMeters);
    cfg.lateralMeters = clamp(cfg.lateralMeters, cfg.minLateralMeters, cfg.lateralMaxMeters);
    cfg.forwardMaxMeters = Math.max(cfg.forwardMaxMeters, cfg.forwardMeters + 0.05);
    cfg.arcRMax = Math.max(cfg.arcRMax, cfg.arcRMin + 0.05);
    cfg.pinnedMaxTimeSeconds = Math.max(cfg.pinnedMaxTimeSeconds, cfg.pinnedMinTimeSeconds + 0.25);
    cfg.nearGoalForwardScale = clamp(cfg.nearGoalForwardScale, 0.05, 1.25);
    cfg.nearGoalLateralScale = clamp(cfg.nearGoalLateralScale, 0.05, 1.50);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static Tunable d(String name, double min, double max) {
    return new Tunable(name, min, max, false);
  }

  private static Tunable i(String name, double min, double max) {
    return new Tunable(name, min, max, true);
  }

  private record Tunable(String name, double min, double max, boolean integer) {}

  private record ObstacleSpec(double x, double y, double radius, double strength) {
    Obstacle newObstacle() {
      PointObstacle obstacle = new PointObstacle(new Translation2d(x, y), strength, true);
      obstacle.radius = radius;
      return obstacle;
    }
  }

  private record Scenario(
      String name, Pose2d start, Pose2d goal, List<ObstacleSpec> obstacles, double maxSeconds) {
    List<Obstacle> newObstacles() {
      List<Obstacle> out = new ArrayList<>();
      for (ObstacleSpec spec : obstacles) out.add(spec.newObstacle());
      return out;
    }
  }

  private record EpisodeMetrics(
      boolean success,
      double durationSeconds,
      double blockedSeconds,
      double pinnedSeconds,
      int collisionSteps,
      double pathLengthMeters,
      double forwardProgressMeters,
      double remainingMeters) {}

  private record Result(
      String label,
      int generation,
      double score,
      ReactiveBypassConfig config,
      int successes,
      double durationSeconds,
      double blockedSeconds,
      double pinnedSeconds,
      int collisionSteps,
      double pathLengthMeters,
      double remainingMeters) {
    Result withLabel(String nextLabel, int nextGeneration) {
      return new Result(
          nextLabel,
          nextGeneration,
          score,
          config,
          successes,
          durationSeconds,
          blockedSeconds,
          pinnedSeconds,
          collisionSteps,
          pathLengthMeters,
          remainingMeters);
    }
  }

  private static final class OptimizerState {
    private final ReactiveBypassConfig base;
    private final List<Scenario> scenarios;
    private final Random random;
    private final Map<String, Integer> indexByName = new HashMap<>();

    OptimizerState(ReactiveBypassConfig base, List<Scenario> scenarios, Random random) {
      this.base = copyConfig(base);
      this.scenarios = scenarios;
      this.random = random;
      for (int i = 0; i < TUNABLES.size(); i++) {
        indexByName.put(TUNABLES.get(i).name(), i);
      }
    }

    double[] currentVector() {
      double[] out = new double[TUNABLES.size()];
      for (int i = 0; i < TUNABLES.size(); i++) {
        Tunable tunable = TUNABLES.get(i);
        out[i] =
            tunable.integer() ? readInt(base, tunable.name()) : readDouble(base, tunable.name());
        out[i] = clamp(out[i], tunable.min(), tunable.max());
      }
      return out;
    }

    double[] initialSigma() {
      double[] out = new double[TUNABLES.size()];
      for (int i = 0; i < TUNABLES.size(); i++) {
        Tunable tunable = TUNABLES.get(i);
        out[i] = Math.max((tunable.max() - tunable.min()) * 0.22, tunable.integer() ? 1.0 : 1e-3);
      }
      return out;
    }

    double[] sample(double[] mean, double[] sigma) {
      double[] out = Arrays.copyOf(mean, mean.length);
      for (int i = 0; i < out.length; i++) {
        Tunable tunable = TUNABLES.get(i);
        double value = mean[i] + random.nextGaussian() * sigma[i];
        if (random.nextDouble() < 0.06) {
          value = tunable.min() + random.nextDouble() * (tunable.max() - tunable.min());
        }
        out[i] = clamp(value, tunable.min(), tunable.max());
        if (tunable.integer()) out[i] = Math.round(out[i]);
      }
      repairVector(out);
      return out;
    }

    Result evaluate(double[] vector, String label, int generation) {
      ReactiveBypassConfig cfg = toConfig(vector);
      Result result = simulate(cfg, scenarios);
      return result.withLabel(label, generation);
    }

    ReactiveBypassConfig toConfig(double[] vector) {
      ReactiveBypassConfig cfg = copyConfig(base);
      double[] repaired = Arrays.copyOf(vector, vector.length);
      repairVector(repaired);
      for (int i = 0; i < TUNABLES.size(); i++) {
        writeValue(cfg, TUNABLES.get(i), repaired[i]);
      }
      repair(cfg);
      return cfg;
    }

    void updateDistribution(List<Result> elite, double[] mean, double[] sigma) {
      double[] nextMean = new double[mean.length];
      for (Result result : elite) {
        double[] vector = vectorFromConfig(result.config());
        for (int i = 0; i < nextMean.length; i++) nextMean[i] += vector[i];
      }
      for (int i = 0; i < nextMean.length; i++) nextMean[i] /= elite.size();

      double[] nextSigma = new double[sigma.length];
      for (Result result : elite) {
        double[] vector = vectorFromConfig(result.config());
        for (int i = 0; i < nextSigma.length; i++) {
          double delta = vector[i] - nextMean[i];
          nextSigma[i] += delta * delta;
        }
      }
      for (int i = 0; i < nextSigma.length; i++) {
        Tunable tunable = TUNABLES.get(i);
        double floor = tunable.integer() ? 0.45 : (tunable.max() - tunable.min()) * 0.015;
        double sampled = Math.sqrt(nextSigma[i] / elite.size());
        sigma[i] =
            clamp(0.72 * sampled + 0.28 * sigma[i] * 0.82, floor, tunable.max() - tunable.min());
        mean[i] = clamp(nextMean[i], tunable.min(), tunable.max());
        if (tunable.integer()) mean[i] = Math.round(mean[i]);
      }
      repairVector(mean);
    }

    Result polish(Result best, double[] mean, double[] sigma, List<Result> history) {
      Result current = best;
      double[] vector = vectorFromConfig(best.config());
      for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < vector.length; i++) {
          double step = Math.max(sigma[i] * 0.55, TUNABLES.get(i).integer() ? 1.0 : 1e-4);
          for (double sign : new double[] {-1.0, 1.0}) {
            double[] trial = Arrays.copyOf(vector, vector.length);
            trial[i] = clamp(trial[i] + sign * step, TUNABLES.get(i).min(), TUNABLES.get(i).max());
            if (TUNABLES.get(i).integer()) trial[i] = Math.round(trial[i]);
            repairVector(trial);
            Result result = evaluate(trial, "polish", 100 + pass);
            history.add(result);
            if (result.score() < current.score()) {
              current = result;
              vector = vectorFromConfig(result.config());
            }
          }
        }
      }
      return current;
    }

    private double[] vectorFromConfig(ReactiveBypassConfig cfg) {
      double[] out = new double[TUNABLES.size()];
      for (int i = 0; i < TUNABLES.size(); i++) {
        Tunable tunable = TUNABLES.get(i);
        out[i] = tunable.integer() ? readInt(cfg, tunable.name()) : readDouble(cfg, tunable.name());
      }
      repairVector(out);
      return out;
    }

    private void repairVector(double[] values) {
      set(values, "occLow", Math.min(get(values, "occLow"), get(values, "occHigh") * 0.75));
      set(values, "occHigh", Math.max(get(values, "occHigh"), get(values, "occLow") + 0.03));
      set(
          values,
          "minLateralMeters",
          Math.min(get(values, "minLateralMeters"), get(values, "lateralMaxMeters")));
      set(
          values,
          "lateralMeters",
          clamp(
              get(values, "lateralMeters"),
              get(values, "minLateralMeters"),
              get(values, "lateralMaxMeters")));
      set(
          values,
          "forwardMaxMeters",
          Math.max(get(values, "forwardMaxMeters"), get(values, "forwardMeters") + 0.05));
      set(values, "arcRMax", Math.max(get(values, "arcRMax"), get(values, "arcRMin") + 0.05));
      set(
          values,
          "pinnedMaxTimeSeconds",
          Math.max(
              get(values, "pinnedMaxTimeSeconds"), get(values, "pinnedMinTimeSeconds") + 0.25));
    }

    private double get(double[] values, String name) {
      return values[indexByName.get(name)];
    }

    private void set(double[] values, String name, double value) {
      int idx = indexByName.get(name);
      Tunable tunable = TUNABLES.get(idx);
      values[idx] = clamp(value, tunable.min(), tunable.max());
      if (tunable.integer()) values[idx] = Math.round(values[idx]);
    }
  }

  private record Options(
      int iterations, long seed, Path outputDir, Path configPath, boolean apply) {
    static Options parse(String[] args) {
      Map<String, String> parsed = new HashMap<>();
      for (int i = 0; i < args.length; i++) {
        String arg = args[i];
        if (!arg.startsWith("--")) continue;
        String key = arg.substring(2);
        String value = "true";
        if (i + 1 < args.length && !args[i + 1].startsWith("--")) {
          value = args[++i];
        }
        parsed.put(key, value);
      }
      return new Options(
          Integer.parseInt(parsed.getOrDefault("iterations", "320")),
          Long.parseLong(parsed.getOrDefault("seed", "4788")),
          Path.of(parsed.getOrDefault("output-dir", "build/reactive-bypass-optimizer")),
          Path.of(parsed.getOrDefault("config", "src/main/deploy/ReactiveBypassConfig.yaml")),
          Boolean.parseBoolean(parsed.getOrDefault("apply", "false")));
    }
  }
}
