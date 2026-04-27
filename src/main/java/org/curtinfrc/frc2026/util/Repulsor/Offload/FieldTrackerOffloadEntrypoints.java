package org.curtinfrc.frc2026.util.Repulsor.Offload;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerLocalAccess;

@SuppressWarnings("unused")
/**
 * Provides field tracker offload entrypoints functionality for the Repulsor offload serialization
 * and native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class FieldTrackerOffloadEntrypoints {
  private FieldTrackerOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.FIELD_TRACKER_NEXT_SHUTTLE_RECOVERY_GOAL_BLUE,
      version = 1,
      timeoutMs = 250,
      fallback = true)
  /**
   * Returns the next shuttle recovery goal blue value maintained by this Repulsor component.
   *
   * @param robotPoseBlue value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param goalUnits value used by this operation.
   * @param flipRedToBlue value used by this operation.
   * @param dynamicObjects value used by this operation.
   * @return value produced by this operation.
   */
  public static Pose2d nextShuttleRecoveryGoalBlue(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects) {
    return FieldTrackerLocalAccess.nextAllianceShuttleRecoveryGoalBlueLocal(
        robotPoseBlue, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects);
  }
}
