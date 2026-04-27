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
import edu.wpi.first.math.geometry.Translation2d;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PointObstacle;

record Tunable(String name, double min, double max, boolean integer) {}

record Candidate(double[] vector, String label, int generation, double cutoffScore) {}

final class PoseHolder {
  Pose2d pose;

  PoseHolder(Pose2d pose) {
    this.pose = pose;
  }
}

record ObstacleSpec(double x, double y, double radius, double strength) {
  Obstacle newObstacle() {
    PointObstacle obstacle = new PointObstacle(new Translation2d(x, y), strength, true);
    obstacle.radius = radius;
    return obstacle;
  }
}

record Scenario(
    String name, Pose2d start, Pose2d goal, List<ObstacleSpec> obstacles, double maxSeconds) {
  List<Obstacle> newObstacles() {
    List<Obstacle> out = new ArrayList<>();
    for (ObstacleSpec spec : obstacles) out.add(spec.newObstacle());
    return out;
  }
}

record EpisodeMetrics(
    boolean success,
    double durationSeconds,
    double blockedSeconds,
    double pinnedSeconds,
    int collisionSteps,
    double pathLengthMeters,
    double forwardProgressMeters,
    double remainingMeters) {}

record ScoreProfile(
    String name,
    double failureScenarioPenalty,
    double collisionScenarioPenalty,
    double collisionStepPenalty,
    double pinnedSecondPenalty,
    double blockedSecondPenalty,
    double durationSecondPenalty,
    double inefficiencyPenalty,
    double progressLossPenalty) {}

record ScenarioReportRow(
    String scenario,
    boolean success,
    double durationSeconds,
    double blockedSeconds,
    double pinnedSeconds,
    int collisionSteps,
    double pathLengthMeters,
    double remainingMeters,
    double scoreContribution) {
  double severity() {
    double failure = success ? 0.0 : 1_000_000.0;
    return failure + scoreContribution + collisionSteps * 100_000.0 + remainingMeters * 10_000.0;
  }
}

record Result(
    String label,
    int generation,
    double score,
    ReactiveBypassConfig config,
    int successes,
    int failedScenarios,
    int collisionScenarios,
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
        failedScenarios,
        collisionScenarios,
        durationSeconds,
        blockedSeconds,
        pinnedSeconds,
        collisionSteps,
        pathLengthMeters,
        remainingMeters);
  }
}

record Options(
    int iterations,
    int randomScenarios,
    int validationScenarios,
    int threads,
    boolean progress,
    boolean reportScenarios,
    long seed,
    Path outputDir,
    Path configPath,
    boolean apply) {
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

    long seed = parsed.containsKey("seed") ? Long.parseLong(parsed.get("seed")) : System.nanoTime();

    return new Options(
        Integer.parseInt(parsed.getOrDefault("iterations", "320")),
        Integer.parseInt(parsed.getOrDefault("random-scenarios", "80")),
        Integer.parseInt(parsed.getOrDefault("validation-scenarios", "300")),
        Integer.parseInt(parsed.getOrDefault("threads", "0")),
        Boolean.parseBoolean(parsed.getOrDefault("progress", "true")),
        Boolean.parseBoolean(parsed.getOrDefault("report-scenarios", "true")),
        seed,
        Path.of(parsed.getOrDefault("output-dir", "build/reactive-bypass-optimizer")),
        Path.of(parsed.getOrDefault("config", "src/main/deploy/ReactiveBypassConfig.yaml")),
        Boolean.parseBoolean(parsed.getOrDefault("apply", "false")));
  }
}
