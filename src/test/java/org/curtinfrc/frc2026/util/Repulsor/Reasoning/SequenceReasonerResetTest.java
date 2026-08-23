package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.EnumSet;
import org.junit.jupiter.api.Test;

class SequenceReasonerResetTest {
  private enum TestPhaseFlag {
    FLAG_A,
    FLAG_B
  }

  @Test
  void resetResumesAtConfiguredStartIndex() {
    SequenceReasoner<TestPhaseFlag, Object> reasoner =
        new SequenceReasoner.Builder<TestPhaseFlag, Object>(TestPhaseFlag.class, () -> 0.0)
            .addPhaseFor("phase-0", EnumSet.of(TestPhaseFlag.FLAG_A), 1.0)
            .addPhaseFor("phase-1", EnumSet.of(TestPhaseFlag.FLAG_A), 1.0)
            .addPhaseFor("phase-2", EnumSet.of(TestPhaseFlag.FLAG_B), 1.0)
            .startAt(2)
            .build();

    assertEquals(2, reasoner.phaseIndex());
    assertEquals("phase-2", reasoner.phaseName());

    reasoner.forcePhase(0);
    assertEquals(0, reasoner.phaseIndex());

    reasoner.reset();
    assertEquals(2, reasoner.phaseIndex());
    assertEquals("phase-2", reasoner.phaseName());
  }

  @Test
  void resetReturnsToStartIndexAfterAutomaticAdvance() {
    SequenceReasoner<TestPhaseFlag, Object> reasoner =
        new SequenceReasoner.Builder<TestPhaseFlag, Object>(TestPhaseFlag.class, () -> 10.0)
            .addPhaseFor("phase-0", EnumSet.of(TestPhaseFlag.FLAG_A), 0.5)
            .addPhaseFor("phase-1", EnumSet.of(TestPhaseFlag.FLAG_B), 0.5)
            .addPhaseFor("phase-2", EnumSet.of(TestPhaseFlag.FLAG_B), 100.0)
            .startAt(2)
            .build();

    reasoner.update(new Object());
    assertEquals(2, reasoner.phaseIndex());

    reasoner.reset();
    assertEquals(2, reasoner.phaseIndex());
  }
}
