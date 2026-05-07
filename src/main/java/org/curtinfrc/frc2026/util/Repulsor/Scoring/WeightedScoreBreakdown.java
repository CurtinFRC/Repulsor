package org.curtinfrc.frc2026.util.Repulsor.Scoring;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/** Immutable named contribution list for explaining and comparing score calculations. */
public record WeightedScoreBreakdown(List<WeightedScoreTerm> terms) {
  public WeightedScoreBreakdown {
    terms = terms == null ? List.of() : List.copyOf(terms);
  }

  public static WeightedScoreBreakdown of(WeightedScoreTerm... terms) {
    return new WeightedScoreBreakdown(terms == null ? List.of() : Arrays.asList(terms));
  }

  public static WeightedScoreBreakdown empty() {
    return new WeightedScoreBreakdown(List.of());
  }

  public double total() {
    double total = 0.0;
    for (WeightedScoreTerm term : terms) {
      if (term != null) total += term.contribution();
    }
    return total;
  }

  public Optional<WeightedScoreTerm> term(String name) {
    if (name == null) return Optional.empty();
    return terms.stream().filter(term -> term != null && name.equals(term.name())).findFirst();
  }

  public WeightedScoreBreakdown plus(WeightedScoreBreakdown other) {
    if (other == null || other.terms().isEmpty()) return this;
    ArrayList<WeightedScoreTerm> combined = new ArrayList<>(terms);
    combined.addAll(other.terms());
    return new WeightedScoreBreakdown(combined);
  }
}
