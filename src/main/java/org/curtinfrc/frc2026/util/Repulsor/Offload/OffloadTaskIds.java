package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides offload task ids functionality for the Repulsor offload serialization and native/JNI
 * entrypoint boundary. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public final class OffloadTaskIds {
  private OffloadTaskIds() {}

  /**
   * Configuration value for drag shot find best shot auto. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final String DRAG_SHOT_FIND_BEST_SHOT_AUTO =
      "repulsor.dragshot.findBestShotAuto.v1";

  /**
   * Configuration value for drag shot calc static shot angle speed. Angles use WPILib rotation
   * conventions; names ending in degrees are degrees, otherwise radians are assumed by the API.
   */
  public static final String DRAG_SHOT_CALC_STATIC_SHOT_ANGLE_SPEED =
      "repulsor.dragshot.calculateStaticShotAngleAndSpeed.v1";

  /**
   * Configuration value for predictive select shuttle recovery point. The valid range and tuning
   * source are defined by the owning subsystem or field profile.
   */
  public static final String PREDICTIVE_SELECT_SHUTTLE_RECOVERY_POINT =
      "repulsor.predictive.selectShuttleRecoveryPoint.v1";

  /**
   * Generic name for predictive resource recovery selection. It currently aliases the legacy
   * shuttle task ID so existing generated wrappers and robot deployments remain wire-compatible.
   */
  public static final String PREDICTIVE_SELECT_RESOURCE_RECOVERY_POINT =
      PREDICTIVE_SELECT_SHUTTLE_RECOVERY_POINT;

  /**
   * Configuration value for field tracker next shuttle recovery goal blue. The valid range and
   * tuning source are defined by the owning subsystem or field profile.
   */
  public static final String FIELD_TRACKER_NEXT_SHUTTLE_RECOVERY_GOAL_BLUE =
      "repulsor.fieldtracker.nextShuttleRecoveryGoalBlue.v1";

  /**
   * Generic name for field-tracker resource recovery. It aliases the legacy shuttle task ID until
   * the generated offload protocol has a profile DTO for generic resource recovery.
   */
  public static final String FIELD_TRACKER_NEXT_RESOURCE_RECOVERY_GOAL_BLUE =
      FIELD_TRACKER_NEXT_SHUTTLE_RECOVERY_GOAL_BLUE;

  /**
   * Configuration value for field planner calculate. The valid range and tuning source are defined
   * by the owning subsystem or field profile.
   */
  public static final String FIELD_PLANNER_CALCULATE = "repulsor.fieldplanner.calculate.v1";

  /**
   * Configuration value for field planner is clear path. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final String FIELD_PLANNER_IS_CLEAR_PATH = "repulsor.fieldplanner.isClearPath.v1";

  /**
   * Configuration value for field planner robot intersects. Time values use seconds and should be
   * tuned against measured robot loop and mechanism latency.
   */
  public static final String FIELD_PLANNER_ROBOT_INTERSECTS =
      "repulsor.fieldplanner.robotIntersects.v1";

  /**
   * Configuration value for sample double value. The valid range and tuning source are defined by
   * the owning subsystem or field profile.
   */
  public static final String SAMPLE_DOUBLE_VALUE = "repulsor.sample.math.double.v1";

  /**
   * Configuration value for sample worker thread probe. The valid range and tuning source are
   * defined by the owning subsystem or field profile.
   */
  public static final String SAMPLE_WORKER_THREAD_PROBE = "repulsor.sample.workerThreadProbe.v1";
}
