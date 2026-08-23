package org.curtinfrc.frc2026.util.Repulsor.State;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class MutableStateCopyTest {
  private static final class CountingState extends MutableState {
    int updates = 0;

    @Override
    public void update(double dt) {
      updates++;
    }
  }

  @Test
  void copyReturnsDistinctInstanceWithCopiedFields() {
    CountingState state = new CountingState();
    state.update(0.02);
    state.update(0.02);

    State copy = state.copy();

    assertNotSame(state, copy);
    assertEquals(CountingState.class, copy.getClass());
    assertNotSame(state, state.copy());
    assertEquals(2, ((CountingState) copy).updates);
    assertEquals(2, state.updates);
  }

  @Test
  void mutatingOriginalDoesNotAffectExistingCopy() {
    CountingState state = new CountingState();
    State copy = state.copy();
    state.update(0.02);

    assertEquals(0, ((CountingState) copy).updates);
    assertEquals(1, state.updates);
  }
}
