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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Runtime;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;

/**
 * Provides predictive collect config runtime functionality for the Repulsor runtime helper layer
 * shared by behaviours and planners. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class PredictiveCollectConfigRuntime {
  private PredictiveCollectConfigRuntime() {}

  /**
   * Updates register resource spec state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param ops value used by this operation.
   * @param type value used by this operation.
   * @param spec value used by this operation.
   */
  public static void registerResourceSpec(
      PredictiveFieldStateOps ops, String type, ResourceSpec spec) {
    if (type == null || type.isEmpty() || spec == null) return;
    ops.resourceSpecs.put(type.toLowerCase(), spec);
    ops.specsVersion++;
    ops.invalidateDynCache();
  }

  /**
   * Updates register other type weight state or telemetry as part of the Repulsor runtime loop.
   * This may mutate local state, NetworkTables output, planner caches, or command-side runtime
   * state depending on the owning type.
   *
   * @param ops value used by this operation.
   * @param type value used by this operation.
   * @param weight value used by this operation.
   */
  public static void registerOtherTypeWeight(
      PredictiveFieldStateOps ops, String type, double weight) {
    if (type == null || type.isEmpty()) return;
    ops.otherTypeWeights.put(type.toLowerCase(), Math.max(0.0, weight));
    ops.specsVersion++;
    ops.invalidateDynCache();
  }

  /**
   * Updates set collect resource types state or telemetry as part of the Repulsor runtime loop.
   * This may mutate local state, NetworkTables output, planner caches, or command-side runtime
   * state depending on the owning type.
   *
   * @param ops value used by this operation.
   * @param types value used by this operation.
   */
  public static void setCollectResourceTypes(PredictiveFieldStateOps ops, Set<String> types) {
    ops.collectResourceTypes.clear();
    if (types != null) {
      for (String type : types) {
        if (type == null || type.isEmpty()) continue;
        ops.collectResourceTypes.add(type.toLowerCase());
      }
    }
    if (ops.collectResourceTypes.isEmpty()) {
      ops.collectResourceTypes.add(PredictiveFieldStateOps.DEFAULT_COLLECT_RESOURCE_TYPE);
    }
    ops.specsVersion++;
    ops.invalidateDynCache();
  }

  /**
   * Returns the get collect resource types value maintained by this Repulsor component.
   *
   * @param ops value used by this operation.
   * @return value produced by this operation.
   */
  public static Set<String> getCollectResourceTypes(PredictiveFieldStateOps ops) {
    return Collections.unmodifiableSet(new HashSet<>(ops.collectResourceTypes));
  }

  /**
   * Runs add collect resource type in the Repulsor runtime.
   *
   * @param ops value used by this operation.
   * @param type value used by this operation.
   */
  public static void addCollectResourceType(PredictiveFieldStateOps ops, String type) {
    if (type == null || type.isEmpty()) return;
    if (ops.collectResourceTypes.add(type.toLowerCase())) {
      ops.specsVersion++;
      ops.invalidateDynCache();
    }
  }

  /**
   * Runs remove collect resource type in the Repulsor runtime.
   *
   * @param ops value used by this operation.
   * @param type value used by this operation.
   */
  public static void removeCollectResourceType(PredictiveFieldStateOps ops, String type) {
    if (type == null || type.isEmpty()) return;
    if (ops.collectResourceTypes.remove(type.toLowerCase())) {
      if (ops.collectResourceTypes.isEmpty()) {
        ops.collectResourceTypes.add(PredictiveFieldStateOps.DEFAULT_COLLECT_RESOURCE_TYPE);
      }
      ops.specsVersion++;
      ops.invalidateDynCache();
    }
  }

  /**
   * Returns the is collect resource type value maintained by this Repulsor component.
   *
   * @param ops value used by this operation.
   * @param type value used by this operation.
   * @return value produced by this operation.
   */
  public static boolean isCollectResourceType(PredictiveFieldStateOps ops, String type) {
    if (type == null || type.isEmpty()) return false;
    return ops.collectResourceTypes.contains(type.toLowerCase());
  }

  /**
   * Updates set collect resource position filter state or telemetry as part of the Repulsor runtime
   * loop. This may mutate local state, NetworkTables output, planner caches, or command-side
   * runtime state depending on the owning type.
   *
   * @param ops value used by this operation.
   * @param filter value used by this operation.
   */
  public static void setCollectResourcePositionFilter(
      PredictiveFieldStateOps ops, Predicate<Translation2d> filter) {
    ops.collectResourcePositionFilter =
        filter != null ? filter : PredictiveFieldStateOps::defaultCollectResourcePositionFilter;
    ops.specsVersion++;
    ops.invalidateDynCache();
  }

  /**
   * Updates set dynamic objects state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param ops value used by this operation.
   * @param objs value used by this operation.
   */
  public static void setDynamicObjects(PredictiveFieldStateOps ops, List<DynamicObject> objs) {
    ops.dynamicObjects = (objs != null) ? List.copyOf(objs) : List.of();
    ops.invalidateDynCache();
  }

  /**
   * Updates set world state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param ops value used by this operation.
   * @param elements value used by this operation.
   * @param ours value used by this operation.
   */
  public static void setWorld(
      PredictiveFieldStateOps ops, List<GameElement> elements, Alliance ours) {
    ops.worldElements = elements != null ? elements : List.of();
    ops.ourAlliance = ours;
  }
}
