package org.curtinfrc.frc2026.util.Repulsor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class IntervalNormalizationTest {
  @Test
  void reversedClosedOpenBehavesAsForwardClosedOpen() {
    Interval<Double> swapped = Interval.closedOpen(5.0, 1.0);
    Interval<Double> forward = Interval.closedOpen(1.0, 5.0);

    assertEquals(forward.min(), swapped.min());
    assertEquals(forward.max(), swapped.max());

    assertTrue(swapped.within(1.0));
    assertTrue(swapped.within(2.5));
    assertTrue(swapped.within(4.999999));
    assertFalse(swapped.within(5.0));
    assertFalse(swapped.within(0.0));

    double[] probes = {-1.0, 0.0, 1.0, 2.5, 4.999999, 5.0, 6.0};
    for (double x : probes) {
      assertEquals(forward.contains(x), swapped.contains(x), "probe=" + x);
      assertEquals(forward.within(x), swapped.within(x), "probe=" + x);
    }
  }

  @Test
  void reversedClosedClosedContainsBothEndpoints() {
    Interval<Integer> swapped = Interval.closed(5, 1);

    assertTrue(swapped.within(1));
    assertTrue(swapped.within(3));
    assertTrue(swapped.within(5));
    assertFalse(swapped.within(0));
    assertFalse(swapped.within(6));
    assertFalse(swapped.isEmpty());
    assertEquals(Integer.valueOf(1), swapped.min());
    assertEquals(Integer.valueOf(5), swapped.max());
  }

  @Test
  void reversedOpenExcludesBothEndpoints() {
    Interval<Double> swapped = Interval.open(5.0, 1.0);

    assertFalse(swapped.within(1.0));
    assertFalse(swapped.within(5.0));
    assertTrue(swapped.within(2.0));
  }

  @Test
  void reversedOpenClosedKeepsTypesOnPositionalBounds() {
    Interval<Double> swapped = Interval.openClosed(5.0, 1.0);

    assertFalse(swapped.within(1.0));
    assertTrue(swapped.within(5.0));
    assertTrue(swapped.within(2.0));
  }

  @Test
  void overlapsAcrossSwappedBounds() {
    Interval<Double> swapped = Interval.closedOpen(5.0, 1.0);

    assertTrue(swapped.overlaps(Interval.closed(4.0, 6.0)));
    assertTrue(swapped.overlaps(Interval.openClosed(1.0, 3.0)));
    assertTrue(swapped.overlaps(Interval.closed(0.0, 1.0)));
    assertFalse(swapped.overlaps(Interval.closed(5.0, 10.0)));
    assertFalse(swapped.overlaps(Interval.closed(6.0, 8.0)));
    assertFalse(swapped.overlaps(Interval.closedOpen(-3.0, -1.0)));
  }

  @Test
  void reversedClosedClosedOverlapTouchingEndpoints() {
    Interval<Integer> swapped = Interval.closed(5, 1);

    assertTrue(swapped.overlaps(Interval.closed(5, 10)));
    assertTrue(swapped.overlaps(Interval.closed(0, 1)));
    assertFalse(swapped.overlaps(Interval.open(5, 10)));
    assertFalse(swapped.overlaps(Interval.open(0, 1)));
  }

  @Test
  void degenerateIntervalsUnchangedByNormalization() {
    assertTrue(Interval.closedOpen(2.0, 2.0).isEmpty());
    assertTrue(Interval.open(2.0, 2.0).isEmpty());
    assertTrue(Interval.openClosed(2.0, 2.0).isEmpty());
    assertFalse(Interval.closed(2.0, 2.0).isEmpty());
    assertFalse(Interval.closedOpen(2.0, 2.0).overlaps(Interval.closedOpen(5.0, 1.0)));
  }
}
