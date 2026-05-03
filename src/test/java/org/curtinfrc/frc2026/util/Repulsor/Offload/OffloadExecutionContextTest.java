package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class OffloadExecutionContextTest {
  @Test
  void explicitContextMarksWorkerAndRestoresAfterRun() {
    assertFalse(OffloadExecutionContext.isWorker());

    boolean inside = OffloadExecutionContext.runWorker(OffloadExecutionContext::isWorker);

    assertTrue(inside);
    assertFalse(OffloadExecutionContext.isWorker());
  }

  @Test
  void sampleProbeUsesExplicitContextWithoutThreadNameDependency() {
    assertFalse(SampleMathOffloadEntrypoints.runsOnOffloadWorkerThread(1));

    boolean inside =
        OffloadExecutionContext.runWorker(
            () -> SampleMathOffloadEntrypoints.runsOnOffloadWorkerThread(1));

    assertTrue(inside);
  }
}
