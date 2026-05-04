package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;

/** Immutable input passed to custom staged-waypoint policies. */
public record FieldPlannerWaypointContext(
    Translation2d robotPosition,
    Pose2d requestedGoal,
    List<GatedAttractorObstacle> waypointGates,
    List<? extends Obstacle> obstacles,
    double fieldLengthMeters,
    double fieldWidthMeters,
    FieldPlannerWaypointConfig config,
    boolean previousStageComplete,
    Translation2d previousStagePoint,
    boolean currentlyStaging,
    Translation2d activeWaypoint,
    boolean exitPhase) {}
