package org.curtinfrc.frc2026.util.Repulsor.Tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryDynamicObjectDTO;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryPointDTO;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateRuntime;

/**
 * Provides field tracker local access functionality for the Repulsor field-object tracking and
 * collection objective layer. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class FieldTrackerLocalAccess {
  private FieldTrackerLocalAccess() {}

  /**
   * Returns the next alliance shuttle recovery goal blue local value maintained by this Repulsor
   * component.
   *
   * @param robotPoseBlue value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param goalUnits value used by this operation.
   * @param flipRedToBlue value used by this operation.
   * @param dynamicObjects value used by this operation.
   * @return value produced by this operation.
   */
  public static Pose2d nextAllianceShuttleRecoveryGoalBlueLocal(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects) {
    if (robotPoseBlue == null) {
      return Pose2d.kZero;
    }

    PredictiveFieldStateRuntime runtime = new PredictiveFieldStateRuntime();
    ShuttleRecoveryPointDTO point =
        runtime.selectShuttleRecoveryPointLocal(
            robotPoseBlue, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects);
    if (point != null && point.isFound()) {
      return new Pose2d(point.getX(), point.getY(), Rotation2d.fromDegrees(point.getYawDeg()));
    }

    return fallbackGoal(robotPoseBlue);
  }

  /**
   * Returns the fallback goal value maintained by this Repulsor component.
   *
   * @param robotPoseBlue value used by this operation.
   * @return value produced by this operation.
   */
  static Pose2d fallbackGoal(Pose2d robotPoseBlue) {
    double x = Math.max(0.7, Math.min(Constants.FIELD_LENGTH * 0.28, robotPoseBlue.getX()));
    double y = Math.max(0.7, Math.min(Constants.FIELD_WIDTH - 0.7, robotPoseBlue.getY()));
    return new Pose2d(x, y, robotPoseBlue.getRotation());
  }
}
