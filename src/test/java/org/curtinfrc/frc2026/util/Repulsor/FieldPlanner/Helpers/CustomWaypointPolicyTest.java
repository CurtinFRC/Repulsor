package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PointObstacle;
import org.junit.jupiter.api.Test;

class CustomWaypointPolicyTest {
  private static final double EPS = 1e-9;

  @Test
  void customPolicyStagesWhenRobotToGoalSegmentPassesNearPoint() {
    FieldPlannerWaypointPolicy policy =
        context -> {
          if (!context.currentlyStaging() && context.robotPosition().getX() < 6.0) {
            return List.of(FieldPlannerWaypointProposal.at("crossing", new Translation2d(6.0, 5.0)));
          }
          return List.of();
        };
    FieldPlannerGoalManager manager =
        new FieldPlannerGoalManager(
            List.of(),
            Constants.FIELD_LENGTH,
            Constants.FIELD_WIDTH,
            FieldPlannerWaypointConfig.defaults().withCustomPolicies(policy));

    Pose2d requested = new Pose2d(new Translation2d(12.0, 2.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(2.0, 2.0), List.of()));
    assertEquals(6.0, manager.getGoalTranslation().getX(), EPS);
    assertEquals(5.0, manager.getGoalTranslation().getY(), EPS);

    FieldPlannerWaypointStatus status = manager.getWaypointStatus();
    assertEquals(
        FieldPlannerWaypointDecision.Mode.USE_DEFAULT,
        status.lastStrategyDecision().mode());
    assertEquals("custom_policy_stage", status.transitionReason());
    assertTrue(status.activeStage());
    assertEquals(6.0, status.stagedAttractor().getX(), EPS);
    assertEquals(5.0, status.stagedAttractor().getY(), EPS);
    assertEquals(requested.getX(), status.requestedGoal().getX(), EPS);

    assertTrue(manager.updateStagedGoal(new Translation2d(6.0, 5.0), List.of()));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void customProposalWithExitPointAdvancesThroughSecondWaypoint() {
    FieldPlannerWaypointPolicy policy =
        context -> {
          if (!context.currentlyStaging() && context.robotPosition().getX() < 5.0) {
            return List.of(
                FieldPlannerWaypointProposal.at("entry", new Translation2d(5.0, 4.0))
                    .withExitPoint(new Translation2d(9.0, 4.0)));
          }
          return List.of();
        };
    FieldPlannerGoalManager manager =
        new FieldPlannerGoalManager(
            List.of(),
            Constants.FIELD_LENGTH,
            Constants.FIELD_WIDTH,
            FieldPlannerWaypointConfig.defaults().withCustomPolicies(policy));

    Pose2d requested = new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(2.0, 4.0), List.of()));
    assertEquals(5.0, manager.getGoalTranslation().getX(), EPS);

    assertFalse(manager.updateStagedGoal(new Translation2d(5.05, 4.0), List.of()));
    assertEquals(9.0, manager.getGoalTranslation().getX(), EPS);
    assertTrue(manager.getWaypointStatus().exitPhase());

    assertTrue(manager.updateStagedGoal(new Translation2d(9.0, 4.0), List.of()));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void disabledBuiltInsYieldOnlyTheCustomProposal() {
    GatedAttractorObstacle gate = gate(new Translation2d(8.0, 4.0), new Translation2d(8.2, 4.0));
    FieldPlannerWaypointPolicy policy =
        context -> {
          if (!context.currentlyStaging() && context.robotPosition().getX() < 9.0) {
            return List.of(
                FieldPlannerWaypointProposal.at("custom", new Translation2d(10.5, 6.5)));
          }
          return List.of();
        };
    FieldPlannerWaypointConfig config =
        FieldPlannerWaypointConfig.defaults()
            .withBandTransitionStagingEnabled(false)
            .withOccludingGateStagingEnabled(false)
            .withCenterReturnStagingEnabled(false)
            .withCustomPolicies(policy);
    FieldPlannerGoalManager manager =
        new FieldPlannerGoalManager(
            List.of(gate), Constants.FIELD_LENGTH, Constants.FIELD_WIDTH, config);
    List<? extends Obstacle> obstacles = List.of(gate);

    Pose2d requested = new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(6.0, 4.0), obstacles));
    assertEquals(10.5, manager.getGoalTranslation().getX(), EPS);
    assertEquals(6.5, manager.getGoalTranslation().getY(), EPS);
    FieldPlannerWaypointStatus status = manager.getWaypointStatus();
    assertEquals("custom_policy_stage", status.transitionReason());
    assertFalse(status.centerReturn());
    assertFalse(status.usingBypass());

    assertTrue(manager.updateStagedGoal(new Translation2d(10.5, 6.5), obstacles));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void higherPriorityCustomPolicyWinsOverBetterScoringLowerPriority() {
    FieldPlannerWaypointPolicy high =
        context ->
            List.of(
                FieldPlannerWaypointProposal.at("high", new Translation2d(5.0, 6.0))
                    .withPriority(10));
    FieldPlannerWaypointPolicy low =
        context ->
            List.of(
                FieldPlannerWaypointProposal.at("low", new Translation2d(5.0, 2.0))
                    .withPriority(1));
    FieldPlannerGoalManager manager = managerWithPolicies(high, low);

    manager.setRequestedGoal(new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero));

    assertFalse(manager.updateStagedGoal(new Translation2d(2.0, 4.0), List.of()));
    assertEquals(6.0, manager.getGoalTranslation().getY(), EPS);
    assertEquals("custom_policy_stage", manager.getWaypointStatus().transitionReason());
  }

  @Test
  void equalPriorityProposalsKeepRegistrationOrderOnScoreTie() {
    FieldPlannerWaypointPolicy first =
        context -> List.of(FieldPlannerWaypointProposal.at("first", new Translation2d(5.0, 6.0)));
    FieldPlannerWaypointPolicy second =
        context -> List.of(FieldPlannerWaypointProposal.at("second", new Translation2d(5.0, 2.0)));
    FieldPlannerGoalManager manager = managerWithPolicies(first, second);

    manager.setRequestedGoal(new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero));

    assertFalse(manager.updateStagedGoal(new Translation2d(2.0, 4.0), List.of()));
    assertEquals(6.0, manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void offFieldProposalIsRejectedAndManagerGoesDirect() {
    FieldPlannerWaypointPolicy policy =
        context ->
            List.of(
                FieldPlannerWaypointProposal.at(
                    "off-field",
                    new Translation2d(Constants.FIELD_LENGTH + 2.0, 4.0)));
    FieldPlannerGoalManager manager =
        new FieldPlannerGoalManager(
            List.of(),
            Constants.FIELD_LENGTH,
            Constants.FIELD_WIDTH,
            FieldPlannerWaypointConfig.defaults().withCustomPolicies(policy));

    Pose2d requested = new Pose2d(new Translation2d(12.0, 2.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertTrue(manager.updateStagedGoal(new Translation2d(2.0, 2.0), List.of()));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
    assertEquals("no_gates_direct", manager.getWaypointStatus().transitionReason());
  }

  @Test
  void proposalInsideObstacleRadiusIsRejected() {
    PointObstacle hazard = new PointObstacle(new Translation2d(6.0, 4.0), 1.0, false);
    hazard.radius = 1.0;
    FieldPlannerWaypointPolicy policy =
        context ->
            List.of(FieldPlannerWaypointProposal.at("inside-hazard", new Translation2d(6.2, 4.0)));
    FieldPlannerGoalManager manager =
        new FieldPlannerGoalManager(
            List.of(),
            Constants.FIELD_LENGTH,
            Constants.FIELD_WIDTH,
            FieldPlannerWaypointConfig.defaults().withCustomPolicies(policy));

    Pose2d requested = new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertTrue(manager.updateStagedGoal(new Translation2d(2.0, 4.0), List.of(hazard)));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals("no_gates_direct", manager.getWaypointStatus().transitionReason());
  }

  @Test
  void proposalOutsideObstacleRadiusIsAccepted() {
    PointObstacle hazard = new PointObstacle(new Translation2d(6.0, 4.0), 1.0, false);
    hazard.radius = 1.0;
    FieldPlannerWaypointPolicy policy =
        context ->
            List.of(
                FieldPlannerWaypointProposal.at(
                    "clear-of-hazard", new Translation2d(7.5, 4.0)));
    FieldPlannerGoalManager manager =
        new FieldPlannerGoalManager(
            List.of(),
            Constants.FIELD_LENGTH,
            Constants.FIELD_WIDTH,
            FieldPlannerWaypointConfig.defaults().withCustomPolicies(policy));

    Pose2d requested = new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(2.0, 4.0), List.of(hazard)));
    assertEquals(7.5, manager.getGoalTranslation().getX(), EPS);
    assertEquals(4.0, manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void roleConstrainedProposalOnlyAppliesToMatchingObjective() {
    FieldPlannerWaypointPolicy policy =
        context ->
            List.of(
                new FieldPlannerWaypointProposal(
                    "score-only",
                    new Translation2d(6.0, 6.0),
                    null,
                    null,
                    FieldPlannerWaypointObjectiveRole.SCORE,
                    null,
                    null,
                    0.0,
                    0,
                    true));
    FieldPlannerGoalManager manager =
        new FieldPlannerGoalManager(
            List.of(),
            Constants.FIELD_LENGTH,
            Constants.FIELD_WIDTH,
            FieldPlannerWaypointConfig.defaults().withCustomPolicies(policy));

    Pose2d requested = new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertTrue(
        manager.updateStagedGoal(
            new Translation2d(2.0, 4.0), List.of(), FieldPlannerWaypointObjectiveRole.COLLECT));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);

    assertFalse(
        manager.updateStagedGoal(
            new Translation2d(2.0, 4.0), List.of(), FieldPlannerWaypointObjectiveRole.SCORE));
    assertEquals(6.0, manager.getGoalTranslation().getX(), EPS);
    assertEquals(6.0, manager.getGoalTranslation().getY(), EPS);
  }

  private static FieldPlannerGoalManager managerWithPolicies(
      FieldPlannerWaypointPolicy first, FieldPlannerWaypointPolicy second) {
    FieldPlannerGoalManager manager =
        new FieldPlannerGoalManager(
            List.of(),
            Constants.FIELD_LENGTH,
            Constants.FIELD_WIDTH,
            FieldPlannerWaypointConfig.defaults().withCustomPolicies(first, second));
    return manager;
  }

  private static GatedAttractorObstacle gate(Translation2d center, Translation2d bypassPoint) {
    Translation2d[] gatePoly =
        new Translation2d[] {
          new Translation2d(center.getX() - 0.30, center.getY() - 0.30),
          new Translation2d(center.getX() + 0.30, center.getY() - 0.30),
          new Translation2d(center.getX() + 0.30, center.getY() + 0.30),
          new Translation2d(center.getX() - 0.30, center.getY() + 0.30)
        };

    return new GatedAttractorObstacle(center, 1.0, 5.0, gatePoly, bypassPoint, 1.0, 5.0, true);
  }
}
