package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateLocalAccess;
import org.junit.jupiter.api.Test;

class PredictiveRecoveryOffloadParityTest {
  @Test
  void predictiveRecoveryOffloadMatchesLocalSelectionAndContractMetadata() {
    Pose2d robot = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(5.0));
    List<ShuttleRecoveryDynamicObjectDTO> objects =
        List.of(resource("fuel-a", 2.0, 2.0, 0.1, 0.0), resource("fuel-b", 3.0, 3.5, 0.0, 0.0));

    ShuttleRecoveryPointDTO local =
        PredictiveFieldStateLocalAccess.selectShuttleRecoveryPointLocal(
            robot, 3.0, 1, false, objects);
    ShuttleRecoveryPointDTO offloaded =
        PredictiveFieldStateOffloadEntrypoints.selectShuttleRecoveryPoint(
            robot, 3.0, 1, false, objects);

    assertEquals(local.isFound(), offloaded.isFound());
    assertEquals(local.getX(), offloaded.getX(), 1e-9);
    assertEquals(local.getY(), offloaded.getY(), 1e-9);
    assertEquals(local.getYawDeg(), offloaded.getYawDeg(), 1e-9);
    assertEquals(local.getScore(), offloaded.getScore(), 1e-9);
    assertEquals(local.getReason(), offloaded.getReason());
    assertEquals(RepulsorOffloadContract.CONTRACT_VERSION, offloaded.getContractVersion());
    assertTrue(RepulsorOffloadContract.isCompatible(offloaded.getContractVersion()));
    assertEquals(1, offloaded.getTaskVersion());
    assertFalse(offloaded.getReason().isBlank());
  }

  private static ShuttleRecoveryDynamicObjectDTO resource(
      String id, double x, double y, double vx, double vy) {
    ShuttleRecoveryDynamicObjectDTO dto = new ShuttleRecoveryDynamicObjectDTO();
    dto.setId(id);
    dto.setType("fuel");
    dto.setX(x);
    dto.setY(y);
    dto.setVx(vx);
    dto.setVy(vy);
    dto.setAgeS(0.05);
    return dto;
  }
}
