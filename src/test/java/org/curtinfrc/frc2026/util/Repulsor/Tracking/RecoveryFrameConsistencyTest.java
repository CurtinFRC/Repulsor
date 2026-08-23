package org.curtinfrc.frc2026.util.Repulsor.Tracking;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryDynamicObjectDTO;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointUtil;
import org.junit.jupiter.api.Test;

class RecoveryFrameConsistencyTest {
  private static final double EPS = 1e-9;

  @Test
  void blueAllianceRecoveryGoalStaysInOwnHalf() {
    Pose2d robotPoseBlue =
        new Pose2d(1.5, Constants.FIELD_WIDTH * 0.5, Rotation2d.fromDegrees(25.0));
    List<ShuttleRecoveryDynamicObjectDTO> dynamics =
        List.of(fuel("fuel-1", 2.0, Constants.FIELD_WIDTH * 0.5, 0.4, 0.0));

    Pose2d goal =
        FieldTrackerLocalAccess.nextAllianceShuttleRecoveryGoalBlueLocal(
            robotPoseBlue, 3.0, 2, false, dynamics);

    assertTrue(
        goal.getX() < Constants.FIELD_LENGTH * 0.5,
        "blue alliance goal should stay in the blue half, got x=" + goal.getX());
    assertTrue(goal.getX() > -EPS);
  }

  @Test
  void redAllianceRecoveryGoalResolvesIntoOwnHalf() {
    Pose2d redRobotPoseBlueOrigin =
        new Pose2d(Constants.FIELD_LENGTH - 1.5, Constants.FIELD_WIDTH * 0.5, Rotation2d.kZero);
    List<ShuttleRecoveryDynamicObjectDTO> redFrameDynamics =
        List.of(fuel("fuel-1", 2.0, Constants.FIELD_WIDTH * 0.5, -0.4, 0.0));

    Pose2d goal =
        FieldTrackerLocalAccess.nextAllianceShuttleRecoveryGoalBlueLocal(
            redRobotPoseBlueOrigin, 3.0, 2, true, redFrameDynamics);

    double blueOriginX = SetpointUtil.flipToBlue(goal).getX();
    assertTrue(
        blueOriginX > Constants.FIELD_LENGTH * 0.5,
        "red alliance goal should map into the red half, got x=" + blueOriginX);
  }

  @Test
  void samePhysicalSceneYieldsSameGoalUnderEitherFrameEncoding() {
    double y = Constants.FIELD_WIDTH * 0.5;

    Pose2d bluePose = new Pose2d(2.0, y, Rotation2d.kZero);
    List<ShuttleRecoveryDynamicObjectDTO> blueDynamics = List.of(fuel("fuel-1", 2.5, y, 0.3, 0.0));
    Pose2d blueGoal =
        FieldTrackerLocalAccess.nextAllianceShuttleRecoveryGoalBlueLocal(
            bluePose, 3.0, 2, false, blueDynamics);

    Pose2d redEncodedPose = new Pose2d(Constants.FIELD_LENGTH - 2.0, y, Rotation2d.kPi);
    List<ShuttleRecoveryDynamicObjectDTO> redEncodedDynamics =
        List.of(fuel("fuel-1", Constants.FIELD_LENGTH - 2.5, y, -0.3, 0.0));
    Pose2d redGoal =
        FieldTrackerLocalAccess.nextAllianceShuttleRecoveryGoalBlueLocal(
            redEncodedPose, 3.0, 2, true, redEncodedDynamics);

    assertEquals(blueGoal.getX(), redGoal.getX(), 1.0);
    assertEquals(blueGoal.getY(), redGoal.getY(), 1.0);
    assertTrue(redGoal.getX() < Constants.FIELD_LENGTH * 0.5);
  }

  private static ShuttleRecoveryDynamicObjectDTO fuel(
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
