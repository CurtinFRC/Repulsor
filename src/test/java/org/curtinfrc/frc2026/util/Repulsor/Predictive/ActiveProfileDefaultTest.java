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

package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Set;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldGeometry;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceCollectionProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceRecoveryProfile;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Runtime.PredictiveCollectConfigRuntime;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ActiveProfileDefaultTest {
  @BeforeEach
  void resetDefaults() {
    PredictiveFieldStateLocalAccess.clearDefaultRecoveryProfile();
    PredictiveFieldStateLocalAccess.clearDefaultCollectionProfile();
  }

  @AfterEach
  void restoreDefaults() {
    PredictiveFieldStateLocalAccess.clearDefaultRecoveryProfile();
    PredictiveFieldStateLocalAccess.clearDefaultCollectionProfile();
  }

  @Test
  void unsetRecoveryDefaultMatchesFuel2026Geometry() {
    ResourceRecoveryProfile recovery = PredictiveFieldStateLocalAccess.defaultRecoveryProfile();
    assertEquals("fuel", recovery.resourceType());
    assertEquals(Constants.FIELD_GEOMETRY, recovery.fieldGeometry());
    assertEquals(Constants.FIELD_LENGTH, recovery.fieldGeometry().lengthMeters());
    assertEquals(Constants.FIELD_WIDTH, recovery.fieldGeometry().widthMeters());
    assertEquals(0.42, recovery.allianceZoneXMaxFraction());
    assertEquals(0.35, recovery.zoneEdgeMarginMeters());
    assertEquals(0.45, recovery.gridStepMeters());
    assertEquals(96, recovery.collectLimit());
    assertEquals(0.075, recovery.resourceSpec().radiusM);
    assertEquals(1.0, recovery.resourceSpec().unitValue);
    assertEquals(0.95, recovery.resourceSpec().sigmaM);
  }

  @Test
  void unsetCollectionDefaultMatchesFuel2026Geometry() {
    ResourceCollectionProfile collection =
        PredictiveFieldStateLocalAccess.defaultCollectionProfile();
    assertEquals("fuel", collection.defaultResourceType());
    assertEquals(Constants.FIELD_GEOMETRY, collection.fieldGeometry());
    assertEquals(0.95, collection.observationHardMaxAgeSeconds());
    assertEquals(0.75, collection.observationAgeDecay());
    assertTrue(collection.excludedRegions().isEmpty());
    assertEquals(0.10, collection.defaultResourceSpec().radiusM);
    PredictiveFieldStateOps ops = new PredictiveFieldStateOps();
    assertEquals("fuel", ops.collectionProfile.defaultResourceType());
    assertEquals(Constants.FIELD_GEOMETRY, ops.collectionProfile.fieldGeometry());
    assertEquals(
        collection.observationHardMaxAgeSeconds(),
        ops.collectionProfile.observationHardMaxAgeSeconds());
    assertEquals(Set.of(collection.defaultResourceType()), ops.collectResourceTypes);
  }

  @Test
  void injectedRecoveryProfileIsHonoredAndCleared() {
    Translation2d deepBluePoint = new Translation2d(12.0, 5.0);
    assertFalse(PredictiveFieldStateLocalAccess.inAllianceZoneBlue(deepBluePoint));

    ResourceRecoveryProfile custom =
        new ResourceRecoveryProfile(
            "algae",
            new ResourceSpec(0.05, 0.9, 0.9),
            new FieldGeometry(22.0, 11.0),
            0.55,
            0.4,
            0.5,
            8);
    PredictiveFieldStateLocalAccess.setDefaultRecoveryProfile(custom);

    assertSame(custom, PredictiveFieldStateLocalAccess.defaultRecoveryProfile());
    assertSame(
        custom, PredictiveFieldStateLocalAccess.activeOrDefault((ResourceRecoveryProfile) null));
    ResourceRecoveryProfile explicit = ResourceRecoveryProfile.fuel2026(Constants.FIELD_GEOMETRY);
    assertSame(explicit, PredictiveFieldStateLocalAccess.activeOrDefault(explicit));
    assertTrue(PredictiveFieldStateLocalAccess.inAllianceZoneBlue(deepBluePoint));

    PredictiveFieldStateLocalAccess.clearDefaultRecoveryProfile();
    assertFalse(PredictiveFieldStateLocalAccess.inAllianceZoneBlue(deepBluePoint));
  }

  @Test
  void injectedCollectionProfileIsHonoredAndCleared() {
    ResourceCollectionProfile custom =
        new ResourceCollectionProfile(
            "pipe",
            new ResourceSpec(0.08, 0.9, 0.9),
            1.2,
            0.5,
            new FieldGeometry(18.0, 9.0),
            List.of());
    PredictiveFieldStateLocalAccess.setDefaultCollectionProfile(custom);

    assertSame(custom, PredictiveFieldStateLocalAccess.defaultCollectionProfile());
    assertSame(
        custom, PredictiveFieldStateLocalAccess.activeOrDefault((ResourceCollectionProfile) null));
    ResourceCollectionProfile explicit =
        ResourceCollectionProfile.fuel2026(Constants.FIELD_GEOMETRY);
    assertSame(explicit, PredictiveFieldStateLocalAccess.activeOrDefault(explicit));

    PredictiveFieldStateOps ops = new PredictiveFieldStateOps();
    assertSame(custom, ops.collectionProfile);

    PredictiveFieldStateLocalAccess.clearDefaultCollectionProfile();
    assertEquals(
        Constants.FIELD_GEOMETRY,
        PredictiveFieldStateLocalAccess.defaultCollectionProfile().fieldGeometry());
  }

  @Test
  void seededCollectResourceTypesFollowInjectedCollectionProfile() {
    ResourceCollectionProfile custom =
        new ResourceCollectionProfile(
            "pipe",
            new ResourceSpec(0.08, 0.9, 0.9),
            1.2,
            0.5,
            new FieldGeometry(18.0, 9.0),
            List.of());
    PredictiveFieldStateLocalAccess.setDefaultCollectionProfile(custom);

    PredictiveFieldStateOps ops = new PredictiveFieldStateOps();
    assertEquals(Set.of(custom.defaultResourceType()), ops.collectResourceTypes);
    assertSame(custom, ops.collectionProfile);

    PredictiveCollectConfigRuntime.configureCollectionProfile(ops, null);
    assertEquals(Set.of("pipe"), ops.collectResourceTypes);
    assertSame(custom, ops.collectionProfile);

    PredictiveFieldStateLocalAccess.clearDefaultCollectionProfile();
    PredictiveCollectConfigRuntime.configureCollectionProfile(ops, null);
    assertEquals(Set.of("fuel"), ops.collectResourceTypes);
    assertEquals(
        "fuel", PredictiveFieldStateLocalAccess.defaultCollectionProfile().defaultResourceType());
  }
}
