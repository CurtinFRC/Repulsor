package org.curtinfrc.frc2026.util.Repulsor.Setpoints;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Provides static pose setpoint functionality for the Repulsor game setpoint abstraction layer for
 * field-relative goals and mechanisms. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class StaticPoseSetpoint extends GameSetpoint {
  private final Pose2d bluePose;

  /**
   * Returns the static pose setpoint value maintained by this Repulsor component.
   *
   * @param name value used by this operation.
   * @param type value used by this operation.
   * @param bluePose value used by this operation.
   */
  public StaticPoseSetpoint(String name, SetpointType type, Pose2d bluePose) {
    super(name, type, false);
    this.bluePose = bluePose == null ? Pose2d.kZero : bluePose;
  }

  /**
   * Returns the blue pose value maintained by this Repulsor component.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  @Override
  public Pose2d bluePose(SetpointContext ctx) {
    return bluePose;
  }

  /**
   * Returns the red pose value maintained by this Repulsor component.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  @Override
  public Pose2d redPose(SetpointContext ctx) {
    return bluePose;
  }
}
