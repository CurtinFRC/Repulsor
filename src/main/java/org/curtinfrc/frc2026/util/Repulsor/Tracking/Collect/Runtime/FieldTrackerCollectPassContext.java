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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;

/**
 * Immutable data record for field tracker collect pass context values passed through the Repulsor
 * runtime helper layer shared by behaviours and planners. Use this type from robot code, field
 * profiles, or tests when integrating the corresponding Repulsor subsystem. Coordinates are
 * field-relative unless a method documents robot-relative motion.
 *
 * @param robotPoseBlue component of the field tracker collect pass context model
 * @param robotPos record component for the field tracker collect pass context model
 * @param cap record component for the field tracker collect pass context model
 * @param usePts record component for the field tracker collect pass context model
 * @param robotInCenterBand record component for the field tracker collect pass context model
 * @param dynAll record component for the field tracker collect pass context model
 * @param dynUse record component for the field tracker collect pass context model
 * @param lockHalf record component for the field tracker collect pass context model
 * @param sensedHalf record component for the field tracker collect pass context model
 * @param nowNs record component for the field tracker collect pass context model
 * @param dt record component for the field tracker collect pass context model
 * @param midX record component for the field tracker collect pass context model
 * @param leftBandX0 record component for the field tracker collect pass context model
 * @param leftBandX1 record component for the field tracker collect pass context model
 * @param rightBandX0 record component for the field tracker collect pass context model
 * @param rightBandX1 record component for the field tracker collect pass context model
 * @param clampToFieldRobotSafe record component for the field tracker collect pass context model
 * @param inForbidden record component for the field tracker collect pass context model
 * @param violatesWall record component for the field tracker collect pass context model
 * @param nudgeOutOfForbidden record component for the field tracker collect pass context model
 * @param safePushedFromRobot record component for the field tracker collect pass context model
 * @param holdPose record component for the field tracker collect pass context model
 * @param nearbyFuelCount record component for the field tracker collect pass context model
 * @param nearbyCentroid record component for the field tracker collect pass context snapshot
 */
public record FieldTrackerCollectPassContext(
    Pose2d robotPoseBlue,
    Translation2d robotPos,
    double cap,
    Translation2d[] usePts,
    boolean robotInCenterBand,
    List<DynamicObject> dynAll,
    List<DynamicObject> dynUse,
    int lockHalf,
    int sensedHalf,
    long nowNs,
    double dt,
    double midX,
    double leftBandX0,
    double leftBandX1,
    double rightBandX0,
    double rightBandX1,
    Function<Translation2d, Translation2d> clampToFieldRobotSafe,
    Predicate<Translation2d> inForbidden,
    Predicate<Translation2d> violatesWall,
    Function<Translation2d, Translation2d> nudgeOutOfForbidden,
    Function<Translation2d, Translation2d> safePushedFromRobot,
    Function<Translation2d, Pose2d> holdPose,
    int nearbyFuelCount,
    Translation2d nearbyCentroid) {}
