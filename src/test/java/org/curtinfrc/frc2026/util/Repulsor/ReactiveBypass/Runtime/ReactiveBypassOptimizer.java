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

import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassEvaluation.addMinedHardCases;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassEvaluation.evaluateScenarioRows;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassEvaluation.printWorstScenarios;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassEvaluation.simulate;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassEvaluation.writeScenarioReport;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassScenarioFactory.buildCurriculumScenarios;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassScenarioFactory.buildHardRandomScenarios;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassScenarioFactory.buildRandomScenarios;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassScenarioFactory.buildScenarios;
import static org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassScenarioFactory.buildValidationScenarios;

import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Random;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletionService;
import java.util.concurrent.ExecutorCompletionService;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/** Offline optimizer for {@code ReactiveBypassConfig.yaml}. */
public final class ReactiveBypassOptimizer {
  // Quality/speed tuning:
  // - Training uses a curriculum so early generations learn basic bypassing quickly.
  // - Fixed scenarios are deliberately weighted higher than random scenarios.
  // - Bad random candidates are early-rejected once they are clearly worse than the current best.
  // - Hard-case mining reuses failed/collision scenarios in later generations.
  // - Collisions are treated as near-unacceptable for robot-safe tuning.
  private static final int LATE_HARD_RANDOM_SCENARIOS = 40;
  private static final int MINED_CASES_PER_GENERATION = 12;
  private static final int SCENARIO_REPORT_WORST_PRINT_COUNT = 5;

  private static final double CANDIDATE_CUTOFF_MULTIPLIER = 1.08;
  private static final double CANDIDATE_CUTOFF_MARGIN = 35_000.0;

  static final ScoreProfile EARLY_PROFILE =
      new ScoreProfile("early", 100_000.0, 160_000.0, 12_000.0, 80.0, 40.0, 6.0, 20.0, 140.0);
  static final ScoreProfile NORMAL_PROFILE =
      new ScoreProfile("normal", 110_000.0, 220_000.0, 18_000.0, 80.0, 40.0, 6.0, 20.0, 140.0);
  static final ScoreProfile LATE_PROFILE =
      new ScoreProfile("late", 130_000.0, 300_000.0, 28_000.0, 100.0, 45.0, 6.0, 20.0, 150.0);
  static final ScoreProfile SAFETY_PROFILE =
      new ScoreProfile("safety", 150_000.0, 400_000.0, 40_000.0, 120.0, 50.0, 5.0, 18.0, 170.0);

  static final List<Tunable> TUNABLES =
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

    Random random = new Random(options.seed());

    List<Scenario> fixedScenarios = buildScenarios();
    List<Scenario> randomTrainingScenarios =
        buildRandomScenarios(random, options.randomScenarios(), "train");
    List<Scenario> hardRandomScenarios =
        buildHardRandomScenarios(random, LATE_HARD_RANDOM_SCENARIOS, "hard");
    List<Scenario> minedHardCases = new ArrayList<>();

    int generations = Math.max(5, Math.min(30, options.iterations() / 16));
    int population = Math.max(8, options.iterations() / generations);
    int threads =
        options.threads() > 0
            ? options.threads()
            : Math.max(1, Runtime.getRuntime().availableProcessors() - 1);

    List<Scenario> initialTrainingScenarios =
        buildCurriculumScenarios(
            fixedScenarios, randomTrainingScenarios, hardRandomScenarios, minedHardCases, 1);

    ReactiveBypassOptimizerState optimizer =
        new ReactiveBypassOptimizerState(
            base, initialTrainingScenarios, random, profileForGeneration(1));

    System.out.println("ReactiveBypass optimizer");
    System.out.println("seed=" + options.seed());
    System.out.println("fixed scenarios=" + fixedScenarios.size());
    System.out.println("random training scenario pool=" + randomTrainingScenarios.size());
    System.out.println("hard random scenario pool=" + hardRandomScenarios.size());
    System.out.println("initial curriculum scenarios=" + initialTrainingScenarios.size());
    System.out.println(
        "validation scenarios=" + (fixedScenarios.size() + options.validationScenarios()));
    System.out.printf(
        Locale.US,
        "iterations=%d generations=%d population=%d threads=%d progress=%s reportScenarios=%s%n",
        options.iterations(),
        generations,
        population,
        threads,
        options.progress(),
        options.reportScenarios());

    List<Result> history = new ArrayList<>();
    Result baseline = optimizer.evaluate(optimizer.currentVector(), "baseline", 0);
    history.add(baseline);
    Result best = baseline;

    System.out.printf(
        Locale.US,
        "baseline score=%.3f success=%d/%d failed=%d collisionScenarios=%d collisionSteps=%d duration=%.3f blocked=%.3f pinned=%.3f%n",
        baseline.score(),
        baseline.successes(),
        initialTrainingScenarios.size(),
        baseline.failedScenarios(),
        baseline.collisionScenarios(),
        baseline.collisionSteps(),
        baseline.durationSeconds(),
        baseline.blockedSeconds(),
        baseline.pinnedSeconds());

    double[] mean = optimizer.currentVector();
    double[] sigma = optimizer.initialSigma();
    int evaluations = 0;

    ExecutorService evalPool = Executors.newFixedThreadPool(threads);

    try {
      for (int generation = 1;
          generation <= generations && evaluations < options.iterations();
          generation++) {
        ScoreProfile profile = profileForGeneration(generation);
        List<Scenario> activeScenarios =
            buildCurriculumScenarios(
                fixedScenarios,
                randomTrainingScenarios,
                hardRandomScenarios,
                minedHardCases,
                generation);
        optimizer.setScenariosAndProfile(activeScenarios, profile);

        // Re-score the incumbent on the current curriculum stage because scenario counts
        // and safety weights change across the curriculum.
        Result incumbent = optimizer.evaluateConfig(best.config(), "incumbent", generation);

        List<Candidate> candidates = new ArrayList<>();

        double generationCutoff =
            Math.max(
                incumbent.score() * CANDIDATE_CUTOFF_MULTIPLIER,
                incumbent.score() + CANDIDATE_CUTOFF_MARGIN);

        candidates.add(
            new Candidate(
                Arrays.copyOf(mean, mean.length), "mean", generation, Double.POSITIVE_INFINITY));

        while (candidates.size() < population && evaluations < options.iterations()) {
          double[] sample = optimizer.sample(mean, sigma);
          candidates.add(new Candidate(sample, "sample", generation, generationCutoff));
          evaluations++;
        }

        if (options.progress()) {
          System.out.printf(
              Locale.US,
              "gen=%02d curriculum=%s scenarios=%d mined=%d cutoff=%.1f%n",
              generation,
              profile.name(),
              activeScenarios.size(),
              minedHardCases.size(),
              generationCutoff);
        }

        List<Result> generationResults =
            evaluateCandidatesParallel(
                optimizer,
                candidates,
                evalPool,
                options.progress(),
                String.format(Locale.US, "gen %02d", generation));

        generationResults.sort(
            Comparator.comparingDouble(Result::score)
                .thenComparingInt(Result::failedScenarios)
                .thenComparingInt(Result::collisionScenarios)
                .thenComparingInt(Result::collisionSteps)
                .thenComparingInt(Result::generation)
                .thenComparing(Result::label));

        Result generationBest = generationResults.get(0);

        best = generationBest.score() < incumbent.score() ? generationBest : incumbent;

        history.addAll(generationResults);

        int eliteCount =
            Math.min(generationResults.size(), Math.max(1, generationResults.size() / 5));
        optimizer.updateDistribution(generationResults.subList(0, eliteCount), mean, sigma);

        List<ScenarioReportRow> reportRows =
            evaluateScenarioRows(generationBest.config(), activeScenarios, profile);
        reportRows.sort(Comparator.comparingDouble(ScenarioReportRow::severity).reversed());

        if (generation >= 9) {
          addMinedHardCases(
              minedHardCases, activeScenarios, reportRows, MINED_CASES_PER_GENERATION);
        }

        System.out.printf(
            Locale.US,
            "gen=%02d best=%.3f overall=%.3f success=%d/%d failed=%d collisionScenarios=%d collisionSteps=%d pinned=%.3f blocked=%.3f sigma=%.4f%n",
            generation,
            generationBest.score(),
            best.score(),
            best.successes(),
            activeScenarios.size(),
            best.failedScenarios(),
            best.collisionScenarios(),
            best.collisionSteps(),
            best.pinnedSeconds(),
            best.blockedSeconds(),
            average(sigma));

        printWorstScenarios(reportRows, SCENARIO_REPORT_WORST_PRINT_COUNT);
      }
    } finally {
      evalPool.shutdown();
      evalPool.awaitTermination(10, TimeUnit.SECONDS);
    }

    List<Scenario> finalTrainingScenarios =
        buildCurriculumScenarios(
            fixedScenarios,
            randomTrainingScenarios,
            hardRandomScenarios,
            minedHardCases,
            generations);
    optimizer.setScenariosAndProfile(finalTrainingScenarios, SAFETY_PROFILE);

    best = optimizer.evaluateConfig(best.config(), "pre-safety-polish", 198);
    history.add(best);
    best = optimizer.polish(best, mean, sigma, history);
    best = optimizer.safetyPolish(best, sigma, history);

    List<Scenario> validationScenarios =
        buildValidationScenarios(options.seed() ^ 0x5DEECE66DL, options.validationScenarios());

    long validationStart = System.nanoTime();
    Result validation =
        simulate(best.config(), validationScenarios, SAFETY_PROFILE).withLabel("validation", 999);
    if (options.progress()) {
      System.out.println(
          "validation completed in " + formatDuration(elapsedSeconds(validationStart)));
    }

    Path resultsPath = outputDir.resolve("results.csv");
    writeResults(resultsPath, history);

    Path bestPath = outputDir.resolve("best.yaml");
    writeYaml(bestPath, best.config());

    Path validationPath = outputDir.resolve("validation.csv");
    writeResults(validationPath, List.of(validation));

    if (options.reportScenarios()) {
      Path scenarioReportPath = outputDir.resolve("scenario-report.csv");
      writeScenarioReport(
          scenarioReportPath,
          evaluateScenarioRows(best.config(), validationScenarios, SAFETY_PROFILE));
      Path trainingScenarioReportPath = outputDir.resolve("training-scenario-report.csv");
      writeScenarioReport(
          trainingScenarioReportPath,
          evaluateScenarioRows(best.config(), finalTrainingScenarios, SAFETY_PROFILE));
      System.out.println("wrote " + scenarioReportPath.toAbsolutePath());
      System.out.println("wrote " + trainingScenarioReportPath.toAbsolutePath());
    }

    System.out.printf(
        Locale.US,
        "best training score=%.3f success=%d/%d failed=%d collisionScenarios=%d collisionSteps=%d duration=%.3f blocked=%.3f pinned=%.3f%n",
        best.score(),
        best.successes(),
        finalTrainingScenarios.size(),
        best.failedScenarios(),
        best.collisionScenarios(),
        best.collisionSteps(),
        best.durationSeconds(),
        best.blockedSeconds(),
        best.pinnedSeconds());

    System.out.printf(
        Locale.US,
        "validation score=%.3f success=%d/%d failed=%d collisionScenarios=%d collisionSteps=%d duration=%.3f blocked=%.3f pinned=%.3f%n",
        validation.score(),
        validation.successes(),
        validationScenarios.size(),
        validation.failedScenarios(),
        validation.collisionScenarios(),
        validation.collisionSteps(),
        validation.durationSeconds(),
        validation.blockedSeconds(),
        validation.pinnedSeconds());

    System.out.println("wrote " + bestPath.toAbsolutePath());
    System.out.println("wrote " + resultsPath.toAbsolutePath());
    System.out.println("wrote " + validationPath.toAbsolutePath());

    if (options.apply()) {
      writeYaml(options.configPath(), best.config());
      System.out.println("updated " + options.configPath().toAbsolutePath());
    } else {
      System.out.println("deploy YAML left unchanged; rerun with -PoptimizerApply=true to apply");
    }
  }

  private static ScoreProfile profileForGeneration(int generation) {
    if (generation <= 8) return EARLY_PROFILE;
    if (generation <= 20) return NORMAL_PROFILE;
    return LATE_PROFILE;
  }

  private static List<Result> evaluateCandidatesParallel(
      ReactiveBypassOptimizerState optimizer,
      List<Candidate> candidates,
      ExecutorService evalPool,
      boolean progress,
      String label)
      throws Exception {
    CompletionService<Result> completion = new ExecutorCompletionService<>(evalPool);
    List<Result> results = new ArrayList<>(candidates.size());
    long startNanos = System.nanoTime();

    for (Candidate candidate : candidates) {
      completion.submit(
          new Callable<Result>() {
            @Override
            public Result call() {
              return optimizer.evaluate(
                  candidate.vector(),
                  candidate.label(),
                  candidate.generation(),
                  candidate.cutoffScore());
            }
          });
    }

    for (int completed = 1; completed <= candidates.size(); completed++) {
      Result result = completion.take().get();
      results.add(result);

      if (progress) {
        printProgress(label, completed, candidates.size(), startNanos);
      }
    }

    if (progress) {
      System.out.println();
    }

    return results;
  }

  private static void printProgress(String label, int completed, int total, long startNanos) {
    int width = 34;
    double frac = total <= 0 ? 1.0 : (double) completed / total;
    int filled = (int) Math.round(frac * width);

    StringBuilder bar = new StringBuilder();
    bar.append('[');
    for (int i = 0; i < width; i++) {
      bar.append(i < filled ? '=' : '-');
    }
    bar.append(']');

    double elapsedSeconds = elapsedSeconds(startNanos);
    double rate = elapsedSeconds <= 1e-9 ? 0.0 : completed / elapsedSeconds;
    double remainingSeconds = rate <= 1e-9 ? 0.0 : (total - completed) / rate;

    String line =
        String.format(
            Locale.US,
            "\r%s %s %3.0f%% %d/%d elapsed=%s eta=%s rate=%.2f eval/s",
            label,
            bar,
            frac * 100.0,
            completed,
            total,
            formatDuration(elapsedSeconds),
            formatDuration(remainingSeconds),
            rate);

    System.out.print(line);
    System.out.flush();
  }

  private static double elapsedSeconds(long startNanos) {
    return (System.nanoTime() - startNanos) / 1_000_000_000.0;
  }

  private static String formatDuration(double seconds) {
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

  private static void writeResults(Path path, List<Result> results) throws IOException {
    StringBuilder out = new StringBuilder();
    out.append(
        "label,generation,score,successes,failed_scenarios,collision_scenarios,duration_s,blocked_s,pinned_s,collision_steps,path_m,remaining_m\n");
    for (Result result : results) {
      out.append(
          String.format(
              Locale.US,
              "%s,%d,%.6f,%d,%d,%d,%.6f,%.6f,%.6f,%d,%.6f,%.6f%n",
              result.label(),
              result.generation(),
              result.score(),
              result.successes(),
              result.failedScenarios(),
              result.collisionScenarios(),
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

  static ReactiveBypassConfig copyConfig(ReactiveBypassConfig source) {
    ReactiveBypassConfig copy = new ReactiveBypassConfig();
    copyConfigInto(source, copy);
    return copy;
  }

  static void copyConfigInto(ReactiveBypassConfig source, ReactiveBypassConfig target) {
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

  static double readDouble(ReactiveBypassConfig cfg, String name) {
    try {
      return ReactiveBypassConfig.class.getField(name).getDouble(cfg);
    } catch (ReflectiveOperationException e) {
      throw new IllegalArgumentException("Unknown double config field " + name, e);
    }
  }

  static int readInt(ReactiveBypassConfig cfg, String name) {
    try {
      return ReactiveBypassConfig.class.getField(name).getInt(cfg);
    } catch (ReflectiveOperationException e) {
      throw new IllegalArgumentException("Unknown int config field " + name, e);
    }
  }

  static void writeValue(ReactiveBypassConfig cfg, Tunable tunable, double value) {
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

  static void repair(ReactiveBypassConfig cfg) {
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

  static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static Tunable d(String name, double min, double max) {
    return new Tunable(name, min, max, false);
  }

  private static Tunable i(String name, double min, double max) {
    return new Tunable(name, min, max, true);
  }
}
