package org.curtinfrc.frc2026.util.Repulsor.Offload;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateLocalAccess;

@SuppressWarnings("unused")
/**
 * Provides predictive field state offload entrypoints functionality for the Repulsor offload
 * serialization and native/JNI entrypoint boundary. Use this type from robot code, field profiles,
 * or tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public final class PredictiveFieldStateOffloadEntrypoints {
  private PredictiveFieldStateOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.PREDICTIVE_SELECT_SHUTTLE_RECOVERY_POINT,
      version = 1,
      timeoutMs = 250,
      fallback = true)
  /**
   * Computes the select shuttle recovery point value for the current Repulsor planning state. Call
   * this from periodic planning or tests when a fresh decision is required; inputs should already
   * be expressed in the coordinate frame expected by the parameter names.
   *
   * @param robotPoseBlue value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param goalUnits value used by this operation.
   * @param flipRedToBlue value used by this operation.
   * @param dynamicObjects value used by this operation.
   * @return shuttle recovery point dto result for select shuttle recovery point.
   */
  public static ShuttleRecoveryPointDTO selectShuttleRecoveryPoint(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects) {
    return PredictiveFieldStateLocalAccess.selectShuttleRecoveryPointLocal(
        robotPoseBlue, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects);
  }
}
