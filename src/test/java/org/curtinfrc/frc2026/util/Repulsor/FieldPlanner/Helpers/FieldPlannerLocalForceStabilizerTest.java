package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Force;
import org.junit.jupiter.api.Test;

class FieldPlannerLocalForceStabilizerTest {
  @Test
  void repeatedLowProgressSamplesTriggerBlendingAndOscillationSignal() {
    FieldPlannerLocalForceStabilizer stabilizer = new FieldPlannerLocalForceStabilizer();
    Translation2d sample = new Translation2d(1.0, 1.0);
    Translation2d target = new Translation2d(4.0, 1.0);

    var first = stabilizer.stabilize(sample, target, new Force(1.0, 0.0), true);
    var second = stabilizer.stabilize(sample, target, new Force(1.0, 0.0), true);
    var third = stabilizer.stabilize(sample, target, new Force(1.0, 0.0), true);
    var fourth = stabilizer.stabilize(sample, target, new Force(1.0, 0.0), true);

    assertFalse(first.snapshot().blended());
    assertFalse(second.snapshot().blended());
    assertTrue(third.snapshot().blended());
    assertTrue(fourth.snapshot().oscillationSuspected());
    assertEquals(3, fourth.snapshot().lowProgressSamples());
  }

  @Test
  void alternatingForceDirectionsAreBlendedDeterministically() {
    FieldPlannerLocalForceStabilizer stabilizer = new FieldPlannerLocalForceStabilizer();
    Translation2d sample = new Translation2d(1.0, 1.0);
    Translation2d target = new Translation2d(4.0, 1.0);

    stabilizer.stabilize(sample, target, new Force(1.0, 0.0), true);
    var flip = stabilizer.stabilize(sample, target, new Force(-1.0, 0.0), true);

    assertTrue(flip.snapshot().blended());
    assertEquals(1, flip.snapshot().directionFlipSamples());
    assertTrue(flip.force().getX() < 0.0);
    assertTrue(flip.force().getX() > -1.0, "blend should damp the full reversal");
  }

  @Test
  void disabledStabilizationStillReportsSignalsWithoutChangingForce() {
    FieldPlannerLocalForceStabilizer stabilizer = new FieldPlannerLocalForceStabilizer();
    Translation2d sample = new Translation2d(1.0, 1.0);
    Translation2d target = new Translation2d(4.0, 1.0);

    stabilizer.stabilize(sample, target, new Force(1.0, 0.0), false);
    var flip = stabilizer.stabilize(sample, target, new Force(-1.0, 0.0), false);

    assertFalse(flip.snapshot().blended());
    assertEquals(-1.0, flip.force().getX(), 1e-9);
    assertEquals(1, flip.snapshot().directionFlipSamples());
  }
}
