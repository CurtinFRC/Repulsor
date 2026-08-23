package org.curtinfrc.frc2026.util.Repulsor.State;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/**
 * Provides sim match driver functionality for the Repulsor match-state storage and simulation
 * driver layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class SimMatchDriver {
  private static final double AUTO_LENGTH_SEC = 20.0;
  private static final double TELEOP_LENGTH_SEC = 140.0;
  private static double matchTimeSec = 0.0;
  private static boolean runAuto = false;

  /**
   * Runs sim init in the Repulsor runtime.
   *
   * @param runAuto value used by this operation.
   */
  public static void simInit(boolean runAuto) {
    if (!RobotBase.isSimulation()) {
      return;
    }
    SimMatchDriver.runAuto = runAuto;
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(true);

    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.setGameSpecificMessage("R");

    DriverStationSim.setAutonomous(runAuto);
    // DriverStationSim.setEnabled(true);

    matchTimeSec = 0.0;
    DriverStationSim.setMatchTime(remainingMatchTime());

    DriverStationSim.notifyNewData();
  }

  /**
   * Runs sim periodic in the Repulsor runtime.
   *
   * @param dt value used by this operation.
   */
  public static void simPeriodic(double dt) {
    if (!RobotBase.isSimulation()) {
      return;
    }
    if (DriverStationSim.getEnabled() == false) {
      return;
    }

    matchTimeSec += dt;
    DriverStationSim.setMatchTime(remainingMatchTime());

    if (matchTimeSec >= AUTO_LENGTH_SEC && runAuto) {
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setEnabled(true);
    }

    DriverStationSim.notifyNewData();
  }

  /**
   * Returns the remaining match time value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static double remainingMatchTime() {
    double total = AUTO_LENGTH_SEC + TELEOP_LENGTH_SEC - matchTimeSec;
    return Math.min(TELEOP_LENGTH_SEC, Math.max(0.0, total));
  }
}
