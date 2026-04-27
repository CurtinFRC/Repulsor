package org.curtinfrc.frc2026.util.Repulsor.State;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/**
 * Provides sim match driver functionality for the Repulsor match-state storage and simulation
 * driver layer. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class SimMatchDriver {
  private static double matchTimeSec = 0.0;
  private static boolean runAuto = false;

  /**
   * Runs sim init in the Repulsor runtime.
   *
   * @param runAuto value used by this operation.
   */
  public static void simInit(boolean runAuto) {
    SimMatchDriver.runAuto = runAuto;
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(true);

    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.setGameSpecificMessage("R");

    DriverStationSim.setAutonomous(runAuto);
    // DriverStationSim.setEnabled(true);

    matchTimeSec = 0.0;
    DriverStationSim.setMatchTime(matchTimeSec);

    DriverStationSim.notifyNewData();
  }

  /**
   * Runs sim periodic in the Repulsor runtime.
   *
   * @param dt value used by this operation.
   */
  public static void simPeriodic(double dt) {
    if (DriverStationSim.getEnabled() == false) {
      return;
    }

    matchTimeSec += dt;
    DriverStationSim.setMatchTime(matchTimeSec);

    if (matchTimeSec >= 20.0 && runAuto) {
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setEnabled(true);
    }

    DriverStationSim.notifyNewData();
  }
}
