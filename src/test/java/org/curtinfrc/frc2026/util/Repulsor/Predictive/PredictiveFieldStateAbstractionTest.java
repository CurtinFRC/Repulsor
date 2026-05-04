package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryDynamicObjectDTO;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryPointDTO;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceCollectionProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceRecoveryProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Strategy.ResourceRegionSummary;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElementModel;
import org.junit.jupiter.api.Test;

class PredictiveFieldStateAbstractionTest {
  private static final double EPS = 1e-9;

  @Test
  void collectionProfileSupportsNonFuelResourcesAndRegionSummaries() {
    FieldGeometry geometry = new FieldGeometry(12.0, 6.0);
    PredictiveFieldStateRuntime predictor = new PredictiveFieldStateRuntime();
    predictor.configureCollectionProfile(
        new ResourceCollectionProfile(
            "pipe",
            new ResourceSpec(0.12, 2.0, 0.08),
            0.50,
            0.10,
            geometry,
            List.of(point -> point.getX() > 10.0)));
    predictor.setDynamicObjects(
        List.of(
            new DynamicObject(
                "pipe-a", "pipe", new Translation2d(2.0, 2.0), new Translation2d(), 0.10),
            new DynamicObject(
                "pipe-b", "pipe", new Translation2d(5.0, 2.0), new Translation2d(), 0.10),
            new DynamicObject(
                "pipe-stale", "pipe", new Translation2d(3.0, 2.0), new Translation2d(), 0.80),
            new DynamicObject(
                "pipe-excluded", "pipe", new Translation2d(11.0, 2.0), new Translation2d(), 0.10),
            new DynamicObject(
                "fuel", "fuel", new Translation2d(2.2, 2.0), new Translation2d(), 0.10)));

    ResourceRegionSummary left =
        predictor.summarizeResourceRegion(
            "left", point -> point.getX() < 4.0, new Translation2d(1.0, 2.0));
    ResourceRegionSummary right =
        predictor.summarizeResourceRegion(
            "right", point -> point.getX() > 4.0, new Translation2d(1.0, 2.0));

    assertTrue(left.hasResources());
    assertEquals(2.0, left.nearestResource().getX(), EPS);
    assertTrue(left.resourceUnits() > 1.5);
    assertTrue(right.hasResources());
    assertEquals(5.0, right.nearestResource().getX(), EPS);
    assertFalse(predictor.footprintHasCollectResource(new Translation2d(11.0, 2.0), 0.10));
  }

  @Test
  void recoveryProfileUsesCustomGeometryAndResourceType() {
    FieldGeometry geometry = new FieldGeometry(10.0, 5.0);
    ResourceRecoveryProfile profile =
        new ResourceRecoveryProfile(
            "pipe", new ResourceSpec(0.10, 1.0, 0.08), geometry, 0.50, 0.25, 0.50, 64);

    ShuttleRecoveryPointDTO point =
        PredictiveFieldStateLocalAccess.selectResourceRecoveryPointLocal(
            new Pose2d(1.0, 2.5, Rotation2d.kZero),
            3.0,
            1,
            true,
            List.of(resourceObject("pipe-red", "pipe", 9.0, 2.5, 0.2, 0.0)),
            profile);

    assertTrue(point.isFound());
    assertTrue(point.getX() <= geometry.lengthMeters() * profile.allianceZoneXMaxFraction());
    assertEquals(1.0, point.getX(), 0.55);
  }

  @Test
  void rankSynthesizesFallbackSetpointForGenericFieldObjective() {
    PredictiveFieldStateRuntime predictor = new PredictiveFieldStateRuntime();
    GameElement generic =
        new GameElement(
            Alliance.kBlue,
            3,
            new GameElementModel(new Pose3d(3.0, 2.0, 0.0, null)),
            gameObject -> true,
            null,
            CategorySpec.kScore);

    predictor.setWorld(List.of(generic), Alliance.kBlue);

    List<Candidate> ranked =
        predictor.rank(new Translation2d(1.0, 2.0), 3.0, CategorySpec.kScore, 4);

    assertEquals(1, ranked.size());
    assertEquals(3.0, ranked.get(0).targetXY.getX(), EPS);
    assertEquals(2.0, ranked.get(0).targetXY.getY(), EPS);
    assertEquals("score", ranked.get(0).setpoint.levelId());
    assertEquals(3.0, ranked.get(0).setpoint.getBlue(null).getX(), EPS);
    assertEquals(2.0, ranked.get(0).setpoint.getBlue(null).getY(), EPS);
  }

  @Test
  void rankSynthesizesFallbackSetpointForRedFieldObjectiveWithoutFlipping() {
    PredictiveFieldStateRuntime predictor = new PredictiveFieldStateRuntime();
    GameElement generic =
        new GameElement(
            Alliance.kRed,
            2,
            new GameElementModel(new Pose3d(13.0, 6.0, 0.0, null)),
            gameObject -> true,
            null,
            CategorySpec.kScore);

    predictor.setWorld(List.of(generic), Alliance.kRed);

    List<Candidate> ranked =
        predictor.rank(new Translation2d(14.0, 6.0), 3.0, CategorySpec.kScore, 4);

    assertEquals(1, ranked.size());
    assertEquals(13.0, ranked.get(0).targetXY.getX(), EPS);
    assertEquals(6.0, ranked.get(0).targetXY.getY(), EPS);
    Pose2d redPose =
        ranked
            .get(0)
            .setpoint
            .getForAlliance(edu.wpi.first.wpilibj.DriverStation.Alliance.Red, null);
    assertEquals(13.0, redPose.getX(), EPS);
    assertEquals(6.0, redPose.getY(), EPS);
  }

  @Test
  void rankStillUsesRebuilt2026ProfileRelatedSetpoints() {
    FieldTrackerCore tracker = new FieldTrackerCore(new Rebuilt2026());

    List<Candidate> ranked =
        tracker.getPredictedCandidates(
            Alliance.kBlue, new Translation2d(1.0, 2.0), 3.0, CategorySpec.kScore, 0);

    assertFalse(ranked.isEmpty());
    assertTrue(
        ranked.stream().anyMatch(candidate -> "net".equals(candidate.setpoint.levelId())),
        "Rebuilt2026 score candidates should preserve profile-provided net mechanism setpoints");
  }

  private static ShuttleRecoveryDynamicObjectDTO resourceObject(
      String id, String type, double x, double y, double vx, double vy) {
    ShuttleRecoveryDynamicObjectDTO dto = new ShuttleRecoveryDynamicObjectDTO();
    dto.setId(id);
    dto.setType(type);
    dto.setX(x);
    dto.setY(y);
    dto.setVx(vx);
    dto.setVy(vy);
    dto.setAgeS(0.05);
    return dto;
  }
}
