package org.curtinfrc.frc2026.util.Repulsor;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class IntakeFootprintResetTest {
  @AfterEach
  void tearDown() {
    IntakeFootprint.resetFootprint();
  }

  @Test
  void setRemainsSetOnceWithoutReset() {
    IntakeFootprint.setFootprint(IntakeFootprint.robotRect(1.0, 0.5));
    assertThrows(
        IllegalStateException.class,
        () -> IntakeFootprint.setFootprint(IntakeFootprint.robotSquare(1.0)));
    assertTrue(IntakeFootprint.getFootprint().containsPointRobotFrame(new Translation2d(0.4, 0.0)));
  }

  @Test
  void resetAllowsInstallingDifferentFootprint() {
    IntakeFootprint.setFootprint(IntakeFootprint.robotRect(1.0, 0.5));
    assertTrue(IntakeFootprint.getFootprint().containsPointRobotFrame(new Translation2d(0.4, 0.0)));

    IntakeFootprint.resetFootprint();
    assertThrows(IllegalStateException.class, IntakeFootprint::getFootprint);

    IntakeFootprint.setFootprint(IntakeFootprint.frontRect(1.0, 0.5, 0.4));
    assertTrue(
        IntakeFootprint.getFootprint().containsPointRobotFrame(new Translation2d(0.85, 0.0)));
    assertFalse(
        IntakeFootprint.getFootprint().containsPointRobotFrame(new Translation2d(-0.4, 0.0)));
  }
}
