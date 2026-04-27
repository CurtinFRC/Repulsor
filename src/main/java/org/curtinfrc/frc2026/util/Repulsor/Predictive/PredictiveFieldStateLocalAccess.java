package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryDynamicObjectDTO;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryPointDTO;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceRecoveryProfile;

/**
 * Provides predictive field state local access functionality for the Repulsor predictive
 * field-state and collection-planning layer. Use this type from robot code, field profiles, or
 * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public final class PredictiveFieldStateLocalAccess {
  private PredictiveFieldStateLocalAccess() {}

  /**
   * Computes the select shuttle recovery point local value for the current Repulsor planning state.
   * Call this from periodic planning or tests when a fresh decision is required; inputs should
   * already be expressed in the coordinate frame expected by the parameter names.
   *
   * @param robotPoseBlue value used by this operation.
   * @param ourSpeedCap value used by this operation.
   * @param goalUnits value used by this operation.
   * @param flipRedToBlue value used by this operation.
   * @param dynamicObjects value used by this operation.
   * @return shuttle recovery point dto result for select shuttle recovery point local.
   */
  public static ShuttleRecoveryPointDTO selectShuttleRecoveryPointLocal(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects) {
    return selectResourceRecoveryPointLocal(
        robotPoseBlue,
        ourSpeedCap,
        goalUnits,
        flipRedToBlue,
        dynamicObjects,
        ResourceRecoveryProfile.fuel2026(Constants.FIELD_GEOMETRY));
  }

  /**
   * Selects a recovery point for transferred resources using a generic resource profile.
   *
   * @param robotPoseBlue robot pose expressed in blue-origin field coordinates
   * @param ourSpeedCap robot speed cap in meters per second used for ranking travel time
   * @param goalUnits desired resource units to recover
   * @param flipRedToBlue whether dynamic-object X positions should be mirrored into blue
   *     coordinates
   * @param dynamicObjects observed transferred resources and other dynamic field objects
   * @param profile resource recovery profile for the active game
   * @return selected recovery point, or {@link ShuttleRecoveryPointDTO#notFound()} when no resource
   *     is recoverable
   */
  public static ShuttleRecoveryPointDTO selectResourceRecoveryPointLocal(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects,
      ResourceRecoveryProfile profile) {
    if (robotPoseBlue == null) {
      return ShuttleRecoveryPointDTO.notFound();
    }
    ResourceRecoveryProfile recoveryProfile =
        profile != null ? profile : ResourceRecoveryProfile.fuel2026(Constants.FIELD_GEOMETRY);

    List<DynamicObject> normalized =
        normalizeDynamicObjects(dynamicObjects, flipRedToBlue, recoveryProfile.fieldGeometry());
    if (normalized.isEmpty()) {
      return ShuttleRecoveryPointDTO.notFound();
    }

    PredictiveFieldStateRuntime predictor = new PredictiveFieldStateRuntime();
    predictor.setFieldGeometry(recoveryProfile.fieldGeometry());
    predictor.registerResourceSpec(recoveryProfile.resourceType(), recoveryProfile.resourceSpec());
    predictor.setCollectResourceTypes(java.util.Set.of(recoveryProfile.resourceType()));
    predictor.setDynamicObjects(normalized);

    Translation2d[] candidates = buildAllianceZoneGrid(recoveryProfile);
    PointCandidate point =
        predictor.rankCollectPoints(
            robotPoseBlue.getTranslation(),
            Math.max(0.25, ourSpeedCap),
            candidates,
            Math.max(1, goalUnits),
            recoveryProfile.collectLimit());

    if (point != null && point.point != null && inAllianceZoneBlue(point.point, recoveryProfile)) {
      double yawDeg =
          point.rotation != null
              ? point.rotation.getDegrees()
              : robotPoseBlue.getRotation().getDegrees();
      return ShuttleRecoveryPointDTO.of(
          point.point.getX(), point.point.getY(), yawDeg, point.score);
    }

    Translation2d nearest =
        nearestResourceInZone(normalized, robotPoseBlue.getTranslation(), recoveryProfile);
    if (nearest == null) {
      return ShuttleRecoveryPointDTO.notFound();
    }
    Rotation2d yaw = nearest.minus(robotPoseBlue.getTranslation()).getAngle();
    return ShuttleRecoveryPointDTO.of(nearest.getX(), nearest.getY(), yaw.getDegrees(), -1.0);
  }

  /**
   * Returns the normalize dynamic objects value maintained by this Repulsor component.
   *
   * @param input value used by this operation.
   * @param flipRedToBlue value used by this operation.
   * @return list of dynamic object values produced by this operation.
   */
  static List<DynamicObject> normalizeDynamicObjects(
      List<ShuttleRecoveryDynamicObjectDTO> input, boolean flipRedToBlue) {
    return normalizeDynamicObjects(input, flipRedToBlue, Constants.FIELD_GEOMETRY);
  }

  static List<DynamicObject> normalizeDynamicObjects(
      List<ShuttleRecoveryDynamicObjectDTO> input, boolean flipRedToBlue, FieldGeometry geometry) {
    if (input == null || input.isEmpty()) {
      return List.of();
    }
    FieldGeometry fieldGeometry = geometry != null ? geometry : Constants.FIELD_GEOMETRY;

    ArrayList<DynamicObject> out = new ArrayList<>(input.size());
    for (ShuttleRecoveryDynamicObjectDTO dto : input) {
      if (dto == null) {
        continue;
      }
      double x = dto.getX();
      double y = dto.getY();
      double vx = dto.getVx();
      double vy = dto.getVy();

      if (flipRedToBlue) {
        x = fieldGeometry.lengthMeters() - x;
        vx = -vx;
      }

      Translation2d pos = new Translation2d(x, y);
      if (!inField(pos, fieldGeometry)) {
        continue;
      }

      String type = dto.getType() == null ? "unknown" : dto.getType();
      out.add(
          new DynamicObject(
              dto.getId(), type, pos, new Translation2d(vx, vy), Math.max(0.0, dto.getAgeS())));
    }
    return out;
  }

  /**
   * Returns the build alliance zone grid value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  static Translation2d[] buildAllianceZoneGrid() {
    return buildAllianceZoneGrid(ResourceRecoveryProfile.fuel2026(Constants.FIELD_GEOMETRY));
  }

  static Translation2d[] buildAllianceZoneGrid(ResourceRecoveryProfile profile) {
    ArrayList<Translation2d> points = new ArrayList<>();
    ResourceRecoveryProfile recoveryProfile =
        profile != null ? profile : ResourceRecoveryProfile.fuel2026(Constants.FIELD_GEOMETRY);
    double xMin = recoveryProfile.zoneEdgeMarginMeters();
    double xMax =
        recoveryProfile.fieldGeometry().lengthMeters() * recoveryProfile.allianceZoneXMaxFraction();
    double yMin = recoveryProfile.zoneEdgeMarginMeters();
    double yMax =
        recoveryProfile.fieldGeometry().widthMeters() - recoveryProfile.zoneEdgeMarginMeters();

    for (double x = xMin; x <= xMax; x += recoveryProfile.gridStepMeters()) {
      for (double y = yMin; y <= yMax; y += recoveryProfile.gridStepMeters()) {
        points.add(new Translation2d(x, y));
      }
    }
    return points.toArray(new Translation2d[0]);
  }

  /**
   * Computes the nearest fuel in zone value for the current Repulsor planning state.
   *
   * @param objects value used by this operation.
   * @param from value used by this operation.
   * @return value produced by this operation.
   */
  static Translation2d nearestFuelInZone(List<DynamicObject> objects, Translation2d from) {
    return nearestResourceInZone(
        objects, from, ResourceRecoveryProfile.fuel2026(Constants.FIELD_GEOMETRY));
  }

  static Translation2d nearestResourceInZone(
      List<DynamicObject> objects, Translation2d from, ResourceRecoveryProfile profile) {
    Translation2d best = null;
    double bestDist = Double.POSITIVE_INFINITY;
    ResourceRecoveryProfile recoveryProfile =
        profile != null ? profile : ResourceRecoveryProfile.fuel2026(Constants.FIELD_GEOMETRY);

    for (DynamicObject object : objects) {
      if (object == null || object.pos == null) {
        continue;
      }
      if (!inAllianceZoneBlue(object.pos, recoveryProfile)) {
        continue;
      }
      String type = object.type == null ? "" : object.type;
      if (!type.equalsIgnoreCase(recoveryProfile.resourceType())) {
        continue;
      }
      double dist = from.getDistance(object.pos);
      if (dist < bestDist) {
        bestDist = dist;
        best = object.pos;
      }
    }
    return best;
  }

  /**
   * Returns the in alliance zone blue value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @return value produced by this operation.
   */
  static boolean inAllianceZoneBlue(Translation2d point) {
    return inAllianceZoneBlue(point, ResourceRecoveryProfile.fuel2026(Constants.FIELD_GEOMETRY));
  }

  static boolean inAllianceZoneBlue(Translation2d point, ResourceRecoveryProfile profile) {
    ResourceRecoveryProfile recoveryProfile =
        profile != null ? profile : ResourceRecoveryProfile.fuel2026(Constants.FIELD_GEOMETRY);
    return point != null
        && point.getX() >= recoveryProfile.zoneEdgeMarginMeters()
        && point.getX()
            <= recoveryProfile.fieldGeometry().lengthMeters()
                * recoveryProfile.allianceZoneXMaxFraction()
        && point.getY() >= recoveryProfile.zoneEdgeMarginMeters()
        && point.getY()
            <= recoveryProfile.fieldGeometry().widthMeters()
                - recoveryProfile.zoneEdgeMarginMeters();
  }

  /**
   * Returns the in field value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @return value produced by this operation.
   */
  static boolean inField(Translation2d point) {
    return inField(point, Constants.FIELD_GEOMETRY);
  }

  static boolean inField(Translation2d point, FieldGeometry geometry) {
    FieldGeometry fieldGeometry = geometry != null ? geometry : Constants.FIELD_GEOMETRY;
    return fieldGeometry.contains(point);
  }
}
