package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.EnumSet;
import org.junit.jupiter.api.Test;

class RuleReasonerTieBreakTest {
  private enum TestFlag {
    FLAG_A,
    FLAG_B,
    FLAG_C
  }

  private static Condition<Object> alwaysTrue() {
    return (ctx, signals) -> true;
  }

  @Test
  void equalPriorityRulesResolveToFirstAddedRule() {
    RuleReasoner<TestFlag, Object> reasoner = new RuleReasoner<>(TestFlag.class);
    reasoner
        .addRule("first", 5, alwaysTrue(), EnumSet.of(TestFlag.FLAG_A))
        .addRule("second", 5, alwaysTrue(), EnumSet.of(TestFlag.FLAG_B));

    assertEquals(EnumSet.of(TestFlag.FLAG_A), reasoner.update(new Object()));
  }

  @Test
  void higherPriorityLaterRuleStillOverrides() {
    RuleReasoner<TestFlag, Object> reasoner = new RuleReasoner<>(TestFlag.class);
    reasoner
        .addRule("first", 5, alwaysTrue(), EnumSet.of(TestFlag.FLAG_A))
        .addRule("second", 10, alwaysTrue(), EnumSet.of(TestFlag.FLAG_B))
        .addRule("third", 10, alwaysTrue(), EnumSet.of(TestFlag.FLAG_C));

    assertEquals(EnumSet.of(TestFlag.FLAG_B), reasoner.update(new Object()));
  }

  @Test
  void insertionOrderDecidesTiesRegardlessOfPriorityDirection() {
    RuleReasoner<TestFlag, Object> descending = new RuleReasoner<>(TestFlag.class);
    descending
        .addRule("early-low", 1, alwaysTrue(), EnumSet.of(TestFlag.FLAG_A))
        .addRule("late-high", 9, alwaysTrue(), EnumSet.of(TestFlag.FLAG_B));
    assertEquals(EnumSet.of(TestFlag.FLAG_B), descending.update(new Object()));

    RuleReasoner<TestFlag, Object> tied = new RuleReasoner<>(TestFlag.class);
    tied.addRule("tie-early", 3, alwaysTrue(), EnumSet.of(TestFlag.FLAG_C));
    tied.addRule("tie-late", 3, (ctx, signals) -> false, EnumSet.of(TestFlag.FLAG_B));
    assertEquals(EnumSet.of(TestFlag.FLAG_C), tied.update(new Object()));
  }

  @Test
  void noMatchingRuleFallsBackAndRepeatedUpdatesStayDeterministic() {
    RuleReasoner<TestFlag, Object> reasoner = new RuleReasoner<>(TestFlag.class);
    reasoner.setFallback(EnumSet.of(TestFlag.FLAG_C));
    reasoner.addRule("never", 5, (ctx, signals) -> false, EnumSet.of(TestFlag.FLAG_A));

    assertEquals(EnumSet.of(TestFlag.FLAG_C), reasoner.update(new Object()));

    reasoner.addRule("first-tied", 2, alwaysTrue(), EnumSet.of(TestFlag.FLAG_A));
    reasoner.addRule("second-tied", 2, alwaysTrue(), EnumSet.of(TestFlag.FLAG_B));
    for (int i = 0; i < 5; i++) {
      assertEquals(EnumSet.of(TestFlag.FLAG_A), reasoner.update(new Object()));
    }
  }
}
