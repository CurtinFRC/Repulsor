/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;

/**
 * Provides defence behaviour functionality for the Repulsor command-behaviour layer that converts
 * strategy and state into WPILib commands. Use this type from robot code, field profiles, or tests
 * when integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
 * method documents robot-relative motion.
 */
public final class DefenceBehaviour extends Behaviour {
  private final int prio;
  private final Supplier<RepulsorSetpoint> defenseGoal;
  private final Supplier<Double> speedCap;

  /**
   * Returns the defence behaviour value maintained by this Repulsor component.
   *
   * @param priority distance or field-coordinate value in meters.
   * @param defenseGoal value used by this operation.
   * @param speedCap value used by this operation.
   */
  public DefenceBehaviour(
      int priority, Supplier<RepulsorSetpoint> defenseGoal, Supplier<Double> speedCap) {
    this.prio = priority;
    this.defenseGoal = defenseGoal;
    this.speedCap = speedCap;
  }

  /**
   * Returns the name value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public String name() {
    return "Defense";
  }

  /**
   * Returns the priority value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public int priority() {
    return prio;
  }

  /**
   * Returns the should run value maintained by this Repulsor component.
   *
   * @param flags value used by this operation.
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  @Override
  public boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
    return flags.contains(BehaviourFlag.DEFENCE_MODE);
  }

  private static SetpointContext makeCtx(BehaviourContext ctx, Pose2d robotPose) {
    return new SetpointContext(
        Optional.ofNullable(robotPose),
        Math.max(0.0, ctx.robot_x) * 2.0,
        Math.max(0.0, ctx.robot_y) * 2.0,
        0.0,
        ctx.vision.getObstacles());
  }

  /**
   * Builds the WPILib command sequence for the current behaviour context.
   *
   * @param ctx runtime context carrying robot state, setpoints, and subsystem access.
   * @return value produced by this operation.
   */
  @Override
  public Command build(BehaviourContext ctx) {
    return Commands.run(
            () -> {
              Pose2d robotPose = ctx.robotPose.get();
              RepulsorSetpoint sp = defenseGoal != null ? defenseGoal.get() : null;
              if (sp == null) {
                ctx.drive.runVelocity(new ChassisSpeeds());
                return;
              }

              Pose2d goalPose = sp.get(makeCtx(ctx, robotPose));
              ctx.repulsor.setCurrentGoal(sp);
              ctx.planner.setRequestedGoal(goalPose);

              RepulsorSample sample =
                  ctx.planner.calculate(
                      robotPose,
                      ctx.vision.getObstacles(),
                      ctx.robot_x,
                      ctx.robot_y,
                      CategorySpec.kEndgame,
                      false,
                      0.0);

              ChassisSpeeds speeds =
                  sample.asChassisSpeeds(
                      ctx.repulsor.getDrive().getOmegaPID(), robotPose.getRotation());

              double cap = speedCap != null ? Math.max(0.25, speedCap.get()) : 2.8;
              speeds.vxMetersPerSecond = Math.max(-cap, Math.min(cap, speeds.vxMetersPerSecond));
              speeds.vyMetersPerSecond = Math.max(-cap, Math.min(cap, speeds.vyMetersPerSecond));

              ctx.drive.runVelocity(speeds);
            },
            ctx.drive)
        .finallyDo(i -> ctx.drive.runVelocity(new ChassisSpeeds()));
  }
}
