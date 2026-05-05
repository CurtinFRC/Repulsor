package org.curtinfrc.frc2026.util.Repulsor.Scenario;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorDiagnosticsSnapshot;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorPlanningRequest;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorPlanningResult;

/** Deterministic helper for measuring FieldPlanner behavior over multiple control cycles. */
public final class RepulsorPlannerScenarioRunner {
  private RepulsorPlannerScenarioRunner() {}

  public static RepulsorPlannerScenarioResult run(RepulsorPlannerScenario scenario) {
    RepulsorPlannerScenario safeScenario = scenario == null ? defaultScenario() : scenario;
    safeScenario
        .planner()
        .syncGoalManagerState(safeScenario.requestedGoal(), safeScenario.requestedGoal());

    Pose2d pose = safeScenario.startPose();
    double initialDistance = distanceToRequestedGoal(safeScenario, pose);
    double minDistance = initialDistance;
    RepulsorPlanningResult lastResult = RepulsorPlanningResult.empty();
    int pathBlockedCycles = 0;
    int globalFallbackCycles = 0;
    int reactiveBypassCycles = 0;
    int robotIntersectingCycles = 0;
    int stuckAbortCycles = 0;
    double maxCommandSpeed = 0.0;
    int stepsTaken = 0;

    for (int step = 0; step < safeScenario.maxSteps(); step++) {
      double distanceBeforeStep = distanceToRequestedGoal(safeScenario, pose);
      if (distanceBeforeStep <= safeScenario.goalToleranceMeters()) {
        break;
      }

      lastResult = safeScenario.planner().calculateDetailed(requestFor(safeScenario, pose));
      RepulsorDiagnosticsSnapshot diagnostics = lastResult.diagnostics();
      if (diagnostics.pathBlocked()) pathBlockedCycles++;
      if (diagnostics.globalFallbackActive()) globalFallbackCycles++;
      if (diagnostics.reactiveBypassActive()) reactiveBypassCycles++;
      if (diagnostics.robotIntersecting()) robotIntersectingCycles++;
      if (diagnostics.stuckAbort()) stuckAbortCycles++;

      double vx = lastResult.sample().vxMetersPerSecond();
      double vy = lastResult.sample().vyMetersPerSecond();
      double commandSpeed = Math.hypot(vx, vy);
      maxCommandSpeed = Math.max(maxCommandSpeed, commandSpeed);
      pose = integrate(pose, vx, vy, lastResult.sample().omegaRadians(), safeScenario.dtSeconds());
      minDistance = Math.min(minDistance, distanceToRequestedGoal(safeScenario, pose));
      stepsTaken = step + 1;
    }

    double finalDistance = distanceToRequestedGoal(safeScenario, pose);
    return new RepulsorPlannerScenarioResult(
        safeScenario,
        pose,
        lastResult,
        stepsTaken,
        finalDistance <= safeScenario.goalToleranceMeters(),
        initialDistance,
        finalDistance,
        minDistance,
        pathBlockedCycles,
        globalFallbackCycles,
        reactiveBypassCycles,
        robotIntersectingCycles,
        stuckAbortCycles,
        maxCommandSpeed);
  }

  private static RepulsorPlanningRequest requestFor(RepulsorPlannerScenario scenario, Pose2d pose) {
    return new RepulsorPlanningRequest(
        pose,
        scenario.dynamicObstacles(),
        scenario.robotHalfLengthMeters(),
        scenario.robotHalfWidthMeters(),
        scenario.category(),
        scenario.suppressFallback(),
        scenario.shooterReleaseHeightMeters(),
        scenario.fallbackAlliance(),
        scenario.name());
  }

  private static Pose2d integrate(
      Pose2d pose, double vx, double vy, double omega, double dtSeconds) {
    Translation2d nextTranslation =
        new Translation2d(pose.getX() + vx * dtSeconds, pose.getY() + vy * dtSeconds);
    Rotation2d nextRotation = pose.getRotation().plus(Rotation2d.fromRadians(omega * dtSeconds));
    return new Pose2d(nextTranslation, nextRotation);
  }

  private static double distanceToRequestedGoal(RepulsorPlannerScenario scenario, Pose2d pose) {
    return pose.getTranslation().getDistance(scenario.requestedGoal().getTranslation());
  }

  private static RepulsorPlannerScenario defaultScenario() {
    return RepulsorPlannerScenario.simple(
        "default", null, Pose2d.kZero, new Pose2d(1.0, 0.0, Rotation2d.kZero));
  }
}
