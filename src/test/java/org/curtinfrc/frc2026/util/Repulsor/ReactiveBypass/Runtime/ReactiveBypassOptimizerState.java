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

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

final class ReactiveBypassOptimizerState {
  private final ReactiveBypassConfig base;
  private List<Scenario> scenarios;
  private ScoreProfile scoreProfile;
  private final Random random;
  private final Map<String, Integer> indexByName = new HashMap<>();

  ReactiveBypassOptimizerState(
      ReactiveBypassConfig base,
      List<Scenario> scenarios,
      Random random,
      ScoreProfile scoreProfile) {
    this.base = ReactiveBypassOptimizer.copyConfig(base);
    this.scenarios = scenarios;
    this.random = random;
    this.scoreProfile = scoreProfile;
    for (int i = 0; i < ReactiveBypassOptimizer.TUNABLES.size(); i++) {
      indexByName.put(ReactiveBypassOptimizer.TUNABLES.get(i).name(), i);
    }
  }

  void setScenariosAndProfile(List<Scenario> scenarios, ScoreProfile scoreProfile) {
    this.scenarios = scenarios;
    this.scoreProfile = scoreProfile;
  }

  double[] currentVector() {
    double[] out = new double[ReactiveBypassOptimizer.TUNABLES.size()];
    for (int i = 0; i < ReactiveBypassOptimizer.TUNABLES.size(); i++) {
      Tunable tunable = ReactiveBypassOptimizer.TUNABLES.get(i);
      out[i] =
          tunable.integer()
              ? ReactiveBypassOptimizer.readInt(base, tunable.name())
              : ReactiveBypassOptimizer.readDouble(base, tunable.name());
      out[i] = ReactiveBypassOptimizer.clamp(out[i], tunable.min(), tunable.max());
    }
    return out;
  }

  double[] initialSigma() {
    double[] out = new double[ReactiveBypassOptimizer.TUNABLES.size()];
    for (int i = 0; i < ReactiveBypassOptimizer.TUNABLES.size(); i++) {
      Tunable tunable = ReactiveBypassOptimizer.TUNABLES.get(i);
      out[i] = Math.max((tunable.max() - tunable.min()) * 0.22, tunable.integer() ? 1.0 : 1e-3);
    }
    return out;
  }

  double[] sample(double[] mean, double[] sigma) {
    double[] out = Arrays.copyOf(mean, mean.length);
    for (int i = 0; i < out.length; i++) {
      Tunable tunable = ReactiveBypassOptimizer.TUNABLES.get(i);
      double value = mean[i] + random.nextGaussian() * sigma[i];
      if (random.nextDouble() < 0.06) {
        value = tunable.min() + random.nextDouble() * (tunable.max() - tunable.min());
      }
      out[i] = ReactiveBypassOptimizer.clamp(value, tunable.min(), tunable.max());
      if (tunable.integer()) out[i] = Math.round(out[i]);
    }
    repairVector(out);
    return out;
  }

  Result evaluate(double[] vector, String label, int generation) {
    return evaluate(vector, label, generation, Double.POSITIVE_INFINITY);
  }

  Result evaluate(double[] vector, String label, int generation, double cutoffScore) {
    ReactiveBypassConfig cfg = toConfig(vector);
    Result result = ReactiveBypassEvaluation.simulate(cfg, scenarios, scoreProfile, cutoffScore);
    return result.withLabel(label, generation);
  }

  Result evaluateConfig(ReactiveBypassConfig cfg, String label, int generation) {
    Result result = ReactiveBypassEvaluation.simulate(cfg, scenarios, scoreProfile);
    return result.withLabel(label, generation);
  }

  ReactiveBypassConfig toConfig(double[] vector) {
    ReactiveBypassConfig cfg = ReactiveBypassOptimizer.copyConfig(base);
    double[] repaired = Arrays.copyOf(vector, vector.length);
    repairVector(repaired);
    for (int i = 0; i < ReactiveBypassOptimizer.TUNABLES.size(); i++) {
      ReactiveBypassOptimizer.writeValue(cfg, ReactiveBypassOptimizer.TUNABLES.get(i), repaired[i]);
    }
    ReactiveBypassOptimizer.repair(cfg);
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
      Tunable tunable = ReactiveBypassOptimizer.TUNABLES.get(i);
      double floor = tunable.integer() ? 0.45 : (tunable.max() - tunable.min()) * 0.015;
      double sampled = Math.sqrt(nextSigma[i] / elite.size());
      sigma[i] =
          ReactiveBypassOptimizer.clamp(
              0.72 * sampled + 0.28 * sigma[i] * 0.82, floor, tunable.max() - tunable.min());
      mean[i] = ReactiveBypassOptimizer.clamp(nextMean[i], tunable.min(), tunable.max());
      if (tunable.integer()) mean[i] = Math.round(mean[i]);
    }
    repairVector(mean);
  }

  Result polish(Result best, double[] mean, double[] sigma, List<Result> history) {
    Result current = best;
    double[] vector = vectorFromConfig(best.config());
    for (int pass = 0; pass < 2; pass++) {
      for (int i = 0; i < vector.length; i++) {
        double step =
            Math.max(
                sigma[i] * 0.55, ReactiveBypassOptimizer.TUNABLES.get(i).integer() ? 1.0 : 1e-4);
        for (double sign : new double[] {-1.0, 1.0}) {
          double[] trial = Arrays.copyOf(vector, vector.length);
          trial[i] =
              ReactiveBypassOptimizer.clamp(
                  trial[i] + sign * step,
                  ReactiveBypassOptimizer.TUNABLES.get(i).min(),
                  ReactiveBypassOptimizer.TUNABLES.get(i).max());
          if (ReactiveBypassOptimizer.TUNABLES.get(i).integer()) trial[i] = Math.round(trial[i]);
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

  Result safetyPolish(Result best, double[] sigma, List<Result> history) {
    ScoreProfile previousProfile = scoreProfile;
    scoreProfile = ReactiveBypassOptimizer.SAFETY_PROFILE;

    Result current = best.withLabel("safety-polish-start", 200);
    double[] vector = vectorFromConfig(best.config());

    for (int pass = 0; pass < 3; pass++) {
      boolean improved = false;
      for (int i = 0; i < vector.length; i++) {
        double step =
            Math.max(
                sigma[i] * (0.45 / (pass + 1.0)),
                ReactiveBypassOptimizer.TUNABLES.get(i).integer() ? 1.0 : 1e-4);
        for (double sign : new double[] {-1.0, 1.0}) {
          double[] trial = Arrays.copyOf(vector, vector.length);
          trial[i] =
              ReactiveBypassOptimizer.clamp(
                  trial[i] + sign * step,
                  ReactiveBypassOptimizer.TUNABLES.get(i).min(),
                  ReactiveBypassOptimizer.TUNABLES.get(i).max());
          if (ReactiveBypassOptimizer.TUNABLES.get(i).integer()) trial[i] = Math.round(trial[i]);
          repairVector(trial);

          Result result = evaluate(trial, "safety-polish", 200 + pass);
          history.add(result);

          if (ReactiveBypassEvaluation.safetyBetter(result, current)) {
            current = result;
            vector = vectorFromConfig(result.config());
            improved = true;
          }
        }
      }

      if (!improved) {
        break;
      }
    }

    scoreProfile = previousProfile;
    return current.withLabel("safety-polish-best", 299);
  }

  private double[] vectorFromConfig(ReactiveBypassConfig cfg) {
    double[] out = new double[ReactiveBypassOptimizer.TUNABLES.size()];
    for (int i = 0; i < ReactiveBypassOptimizer.TUNABLES.size(); i++) {
      Tunable tunable = ReactiveBypassOptimizer.TUNABLES.get(i);
      out[i] =
          tunable.integer()
              ? ReactiveBypassOptimizer.readInt(cfg, tunable.name())
              : ReactiveBypassOptimizer.readDouble(cfg, tunable.name());
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
        ReactiveBypassOptimizer.clamp(
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
        Math.max(get(values, "pinnedMaxTimeSeconds"), get(values, "pinnedMinTimeSeconds") + 0.25));
  }

  private double get(double[] values, String name) {
    return values[indexByName.get(name)];
  }

  private void set(double[] values, String name, double value) {
    int idx = indexByName.get(name);
    Tunable tunable = ReactiveBypassOptimizer.TUNABLES.get(idx);
    values[idx] = ReactiveBypassOptimizer.clamp(value, tunable.min(), tunable.max());
    if (tunable.integer()) values[idx] = Math.round(values[idx]);
  }
}
