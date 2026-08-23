package org.curtinfrc.frc2026.util.Repulsor.Behaviours.Runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.VisionPlanner;
import org.junit.jupiter.api.Test;

class ProjectileCycleMakeCtxTest {
  private static final double EPS = 1e-9;

  private static BehaviourContext context(double robotX, double robotY, Pose2d pose) {
    return new BehaviourContext(
        null, null, new VisionPlanner(), null, robotX, robotY, () -> pose);
  }

  @Test
  void suppliedReleaseHeightFlowsIntoContext() {
    Pose2d pose = new Pose2d(3.1, 4.2, Rotation2d.fromDegrees(30.0));
    SetpointContext ctx = ProjectileCycleRuntime.makeCtx(context(0.85, 0.85, pose), pose, () -> 1.2);

    assertEquals(1.2, ctx.shooterReleaseHeightMeters(), EPS);
    assertTrue(ctx.robotPose().isPresent());
    assertEquals(pose, ctx.robotPose().get());
    assertEquals(List.of(), ctx.dynamicObstacles());
  }

  @Test
  void robotDimensionsPassThroughUnchanged() {
    Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.kZero);
    SetpointContext ctx = ProjectileCycleRuntime.makeCtx(context(0.85, 0.60, pose), pose, () -> 0.0);

    assertEquals(0.85, ctx.robotLengthMeters(), EPS);
    assertEquals(0.60, ctx.robotWidthMeters(), EPS);
    assertEquals(0.0, ctx.shooterReleaseHeightMeters(), EPS);
  }

  @Test
  void targetHeightReleaseFlowsIntoContext() {
    Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.kZero);
    SetpointContext ctx = ProjectileCycleRuntime.makeCtx(context(0.85, 0.60, pose), pose);

    assertEquals(0.85, ctx.robotLengthMeters(), EPS);
    assertEquals(0.60, ctx.robotWidthMeters(), EPS);
  }

  @Test
  void negativeInputsClampedToZero() {
    Pose2d pose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    SetpointContext ctx = ProjectileCycleRuntime.makeCtx(context(-0.5, -0.5, pose), pose, () -> -2.0);

    assertEquals(0.0, ctx.robotLengthMeters(), EPS);
    assertEquals(0.0, ctx.robotWidthMeters(), EPS);
    assertEquals(0.0, ctx.shooterReleaseHeightMeters(), EPS);
  }

  @Test
  void nullRobotPoseYieldsEmptyOptional() {
    SetpointContext ctx =
        ProjectileCycleRuntime.makeCtx(context(0.85, 0.85, Pose2d.kZero), null, () -> 0.5);

    assertTrue(ctx.robotPose().isEmpty());
    assertEquals(0.5, ctx.shooterReleaseHeightMeters(), EPS);
  }

  @Test
  void visionObstaclesCarriedIntoContext() {
    Pose2d pose = new Pose2d(1.0, 1.0, Rotation2d.kZero);
    BehaviourContext ctx =
        new BehaviourContext(null, null, new VisionPlanner(), null, 0.85, 0.85, () -> pose);

    SetpointContext spCtx = ProjectileCycleRuntime.makeCtx(ctx, pose, () -> 0.25);

    assertEquals(pose, spCtx.robotPose().orElseThrow());
    assertEquals(0.25, spCtx.shooterReleaseHeightMeters(), EPS);
  }
}
