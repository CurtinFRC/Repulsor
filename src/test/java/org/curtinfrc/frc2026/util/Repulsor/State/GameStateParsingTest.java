package org.curtinfrc.frc2026.util.Repulsor.State;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.lang.reflect.Field;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class GameStateParsingTest {
  private GameState state;

  @BeforeEach
  void setUp() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setGameSpecificMessage("B3X");
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    state = new GameState();
  }

  @AfterEach
  void tearDown() {
    DriverStationSim.setGameSpecificMessage("");
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private Optional<DriverStation.Alliance> inactiveFirst() throws Exception {
    Field f = GameState.class.getDeclaredField("inactiveFirst");
    f.setAccessible(true);
    @SuppressWarnings("unchecked")
    Optional<DriverStation.Alliance> v = (Optional<DriverStation.Alliance>) f.get(state);
    return v;
  }

  @Test
  void multiCharMessageStartingWithBClassifiesBlue() throws Exception {
    state.update(0.02);
    assertTrue(inactiveFirst().isPresent());
    assertEquals(DriverStation.Alliance.Blue, inactiveFirst().get());
  }

  @Test
  void messageStartingWithRClassifiesRed() throws Exception {
    DriverStationSim.setGameSpecificMessage("R1");
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    state.update(0.02);
    assertEquals(DriverStation.Alliance.Red, inactiveFirst().get());
  }

  @Test
  void emptyMessageLeavesClassificationEmptyAndAlertRaised() throws Exception {
    DriverStationSim.setGameSpecificMessage("");
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    state.update(0.02);
    assertTrue(inactiveFirst().isEmpty());
  }

  @Test
  void classificationIsStickyOnceRead() throws Exception {
    state.update(0.02);
    DriverStationSim.setGameSpecificMessage("");
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    state.update(0.02);
    assertEquals(DriverStation.Alliance.Blue, inactiveFirst().get());
  }
}
