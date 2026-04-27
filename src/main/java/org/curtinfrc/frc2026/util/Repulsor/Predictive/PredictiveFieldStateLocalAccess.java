package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryDynamicObjectDTO;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryPointDTO;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;

/**
 * Provides predictive field state local access functionality for the Repulsor predictive
 * field-state and collection-planning layer. Use this type from robot code, field profiles, or
 * tests when integrating the corresponding Repulsor subsystem. Coordinates are field-relative
 * unless a method documents robot-relative motion.
 */
public final class PredictiveFieldStateLocalAccess {
  private static final String RECOVERY_RESOURCE_TYPE = "fuel";
  private static final ResourceSpec RECOVERY_RESOURCE_SPEC = new ResourceSpec(0.075, 1.0, 0.95);

  private static final double ZONE_X_MAX_FRAC = 0.42;
  private static final double ZONE_EDGE_MARGIN_M = 0.35;
  private static final double GRID_STEP_M = 0.45;
  private static final int COLLECT_LIMIT = 96;

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
    if (robotPoseBlue == null) {
      return ShuttleRecoveryPointDTO.notFound();
    }

    List<DynamicObject> normalized = normalizeDynamicObjects(dynamicObjects, flipRedToBlue);
    if (normalized.isEmpty()) {
      return ShuttleRecoveryPointDTO.notFound();
    }

    PredictiveFieldStateRuntime predictor = new PredictiveFieldStateRuntime();
    predictor.registerResourceSpec(RECOVERY_RESOURCE_TYPE, RECOVERY_RESOURCE_SPEC);
    predictor.addCollectResourceType(RECOVERY_RESOURCE_TYPE);
    predictor.setDynamicObjects(normalized);

    Translation2d[] candidates = buildAllianceZoneGrid();
    PointCandidate point =
        predictor.rankCollectPoints(
            robotPoseBlue.getTranslation(),
            Math.max(0.25, ourSpeedCap),
            candidates,
            Math.max(1, goalUnits),
            COLLECT_LIMIT);

    if (point != null && point.point != null && inAllianceZoneBlue(point.point)) {
      double yawDeg =
          point.rotation != null
              ? point.rotation.getDegrees()
              : robotPoseBlue.getRotation().getDegrees();
      return ShuttleRecoveryPointDTO.of(
          point.point.getX(), point.point.getY(), yawDeg, point.score);
    }

    Translation2d nearest = nearestFuelInZone(normalized, robotPoseBlue.getTranslation());
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
    if (input == null || input.isEmpty()) {
      return List.of();
    }

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
        x = Constants.FIELD_LENGTH - x;
        vx = -vx;
      }

      Translation2d pos = new Translation2d(x, y);
      if (!inField(pos)) {
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
    ArrayList<Translation2d> points = new ArrayList<>();
    double xMin = ZONE_EDGE_MARGIN_M;
    double xMax = Constants.FIELD_LENGTH * ZONE_X_MAX_FRAC;
    double yMin = ZONE_EDGE_MARGIN_M;
    double yMax = Constants.FIELD_WIDTH - ZONE_EDGE_MARGIN_M;

    for (double x = xMin; x <= xMax; x += GRID_STEP_M) {
      for (double y = yMin; y <= yMax; y += GRID_STEP_M) {
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
    Translation2d best = null;
    double bestDist = Double.POSITIVE_INFINITY;

    for (DynamicObject object : objects) {
      if (object == null || object.pos == null) {
        continue;
      }
      if (!inAllianceZoneBlue(object.pos)) {
        continue;
      }
      String type = object.type == null ? "" : object.type;
      if (!type.equalsIgnoreCase(RECOVERY_RESOURCE_TYPE)) {
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
    return point != null
        && point.getX() >= ZONE_EDGE_MARGIN_M
        && point.getX() <= Constants.FIELD_LENGTH * ZONE_X_MAX_FRAC
        && point.getY() >= ZONE_EDGE_MARGIN_M
        && point.getY() <= Constants.FIELD_WIDTH - ZONE_EDGE_MARGIN_M;
  }

  /**
   * Returns the in field value maintained by this Repulsor component.
   *
   * @param point value used by this operation.
   * @return value produced by this operation.
   */
  static boolean inField(Translation2d point) {
    return point.getX() >= 0.0
        && point.getX() <= Constants.FIELD_LENGTH
        && point.getY() >= 0.0
        && point.getY() <= Constants.FIELD_WIDTH;
  }
}
