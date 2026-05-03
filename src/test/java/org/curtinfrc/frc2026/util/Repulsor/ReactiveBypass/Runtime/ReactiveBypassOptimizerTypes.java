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
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
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
    double minClearanceMeters,
    double clearanceRisk,
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
    double progressLossPenalty,
    double clearanceRiskPenalty) {}

record ScenarioReportRow(
    String scenario,
    boolean success,
    double durationSeconds,
    double blockedSeconds,
    double pinnedSeconds,
    int collisionSteps,
    double minClearanceMeters,
    double clearanceRisk,
    double pathLengthMeters,
    double remainingMeters,
    double scoreContribution) {
  double severity() {
    double failure = success ? 0.0 : 1_000_000.0;
    double clearance = Math.max(0.0, 0.20 - minClearanceMeters) * 100_000.0;
    return failure
        + scoreContribution
        + collisionSteps * 100_000.0
        + clearance
        + remainingMeters * 10_000.0;
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

final class ProgressReporter {
  private static final int BAR_WIDTH = 34;

  private final boolean enabled;
  private final String label;
  private final int total;
  private final long startNanos;
  private int completed;
  private boolean finished;

  ProgressReporter(boolean enabled, String label, int total) {
    this.enabled = enabled;
    this.label = label;
    this.total = Math.max(0, total);
    this.startNanos = System.nanoTime();
  }

  static ProgressReporter disabled() {
    return new ProgressReporter(false, "", 0);
  }

  void step() {
    update(completed + 1);
  }

  void update(int completed) {
    if (!enabled || finished) return;

    this.completed = Math.max(0, Math.min(completed, total));
    print();

    if (this.completed >= total) finish();
  }

  void finish() {
    if (!enabled || finished) return;
    if (completed < total) {
      completed = total;
      print();
    }
    System.out.println();
    finished = true;
  }

  private void print() {
    double frac = total <= 0 ? 1.0 : (double) completed / total;
    int filled = (int) Math.round(frac * BAR_WIDTH);

    StringBuilder bar = new StringBuilder();
    bar.append('[');
    for (int i = 0; i < BAR_WIDTH; i++) {
      bar.append(i < filled ? '=' : '-');
    }
    bar.append(']');

    double elapsedSeconds = elapsedSeconds(startNanos);
    double rate = elapsedSeconds <= 1e-9 ? 0.0 : completed / elapsedSeconds;
    double remainingSeconds = rate <= 1e-9 ? 0.0 : (total - completed) / rate;

    System.out.printf(
        Locale.US,
        "\r%s %s %3.0f%% %d/%d elapsed=%s eta=%s rate=%.2f/s",
        label,
        bar,
        frac * 100.0,
        completed,
        total,
        formatDuration(elapsedSeconds),
        formatDuration(remainingSeconds),
        rate);
    System.out.flush();
  }

  static double elapsedSeconds(long startNanos) {
    return (System.nanoTime() - startNanos) / 1_000_000_000.0;
  }

  static String formatDuration(double seconds) {
    if (seconds < 60.0) {
      return String.format(Locale.US, "%.1fs", seconds);
    }

    int totalSeconds = (int) Math.round(seconds);
    int minutes = totalSeconds / 60;
    int remainingSeconds = totalSeconds % 60;

    if (minutes < 60) {
      return String.format(Locale.US, "%dm%02ds", minutes, remainingSeconds);
    }

    int hours = minutes / 60;
    int remainingMinutes = minutes % 60;
    return String.format(Locale.US, "%dh%02dm%02ds", hours, remainingMinutes, remainingSeconds);
  }
}

record Options(
    int iterations,
    int randomScenarios,
    int validationScenarios,
    int checkpointScenarios,
    int threads,
    boolean progress,
    boolean reportScenarios,
    boolean skipPolish,
    int polishPasses,
    int safetyPolishPasses,
    int applyMaxValidationFailures,
    int applyMaxValidationCollisionScenarios,
    int applyMaxValidationCollisionSteps,
    long seed,
    Path outputDir,
    Path configPath,
    List<String> traceScenarios,
    boolean traceOnly,
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
    boolean reportScenarios =
        Boolean.parseBoolean(parsed.getOrDefault("report-scenarios", "true"))
            && !Boolean.parseBoolean(parsed.getOrDefault("skip-reports", "false"));

    return new Options(
        Integer.parseInt(parsed.getOrDefault("iterations", "320")),
        Integer.parseInt(parsed.getOrDefault("random-scenarios", "80")),
        Integer.parseInt(parsed.getOrDefault("validation-scenarios", "300")),
        Integer.parseInt(parsed.getOrDefault("checkpoint-scenarios", "80")),
        Integer.parseInt(parsed.getOrDefault("threads", "0")),
        Boolean.parseBoolean(parsed.getOrDefault("progress", "true")),
        reportScenarios,
        Boolean.parseBoolean(parsed.getOrDefault("skip-polish", "false")),
        Integer.parseInt(parsed.getOrDefault("polish-passes", "1")),
        Integer.parseInt(parsed.getOrDefault("safety-polish-passes", "2")),
        Integer.parseInt(parsed.getOrDefault("apply-max-validation-failures", "0")),
        Integer.parseInt(parsed.getOrDefault("apply-max-validation-collision-scenarios", "0")),
        Integer.parseInt(parsed.getOrDefault("apply-max-validation-collision-steps", "0")),
        seed,
        Path.of(parsed.getOrDefault("output-dir", "build/reactive-bypass-optimizer")),
        Path.of(parsed.getOrDefault("config", "src/main/deploy/ReactiveBypassConfig.yaml")),
        parseTraceScenarios(parsed.getOrDefault("trace-scenarios", "")),
        Boolean.parseBoolean(parsed.getOrDefault("trace-only", "false")),
        Boolean.parseBoolean(parsed.getOrDefault("apply", "false")));
  }

  private static List<String> parseTraceScenarios(String raw) {
    if (raw == null || raw.isBlank()) return List.of();
    return Arrays.stream(raw.split(","))
        .map(String::trim)
        .filter(s -> !s.isEmpty())
        .distinct()
        .toList();
  }
}
