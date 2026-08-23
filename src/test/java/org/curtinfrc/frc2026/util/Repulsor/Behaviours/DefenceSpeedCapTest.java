package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class DefenceSpeedCapTest {
  private static final double EPS = 1e-9;

  @Test
  void diagonalCommandMagnitudeEqualsCapAfterClamp() {
    ChassisSpeeds speeds = new ChassisSpeeds(2.8, 2.8, 0.0);
    ChassisSpeeds clamped = DefenceBehaviour.clampSpeedCap(speeds, 2.8);

    double mag = Math.hypot(clamped.vxMetersPerSecond, clamped.vyMetersPerSecond);
    assertEquals(2.8, mag, EPS);
    assertEquals(clamped.vxMetersPerSecond, clamped.vyMetersPerSecond, EPS);
  }

  @Test
  void clampedDiagonalIsStrictlyBelowPerAxisLimit() {
    double cap = 2.8;
    ChassisSpeeds clamped = DefenceBehaviour.clampSpeedCap(new ChassisSpeeds(cap, cap, 0.0), cap);

    assertTrue(
        Math.abs(clamped.vxMetersPerSecond) <= cap + EPS,
        "per-axis clamp previously allowed vx=cap on a diagonal");
    assertEquals(cap, Math.hypot(clamped.vxMetersPerSecond, clamped.vyMetersPerSecond), EPS);
  }

  @Test
  void belowCapCommandUnchanged() {
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, -1.5, 0.7);
    ChassisSpeeds clamped = DefenceBehaviour.clampSpeedCap(speeds, 2.8);

    assertEquals(1.0, clamped.vxMetersPerSecond, EPS);
    assertEquals(-1.5, clamped.vyMetersPerSecond, EPS);
    assertEquals(0.7, clamped.omegaRadiansPerSecond, EPS);
  }

  @Test
  void directionPreservedWhenScaling() {
    double cap = 1.0;
    ChassisSpeeds clamped = DefenceBehaviour.clampSpeedCap(new ChassisSpeeds(-3.0, 4.0, 0.0), cap);

    assertEquals(cap, Math.hypot(clamped.vxMetersPerSecond, clamped.vyMetersPerSecond), EPS);
    assertEquals(-0.6, clamped.vxMetersPerSecond, EPS);
    assertEquals(0.8, clamped.vyMetersPerSecond, EPS);
  }

  @Test
  void zeroCommandStaysZero() {
    ChassisSpeeds clamped = DefenceBehaviour.clampSpeedCap(new ChassisSpeeds(), 2.8);

    assertEquals(0.0, clamped.vxMetersPerSecond, EPS);
    assertEquals(0.0, clamped.vyMetersPerSecond, EPS);
  }
}
