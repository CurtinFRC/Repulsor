package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import java.util.ArrayList;
import java.util.List;

/**
 * Tunable policy for staged FieldPlanner waypoints. Different games can use this to decide when the
 * goal manager inserts intermediate waypoints and how far those waypoints lead through gates.
 * Seasons can register custom {@link FieldPlannerWaypointPolicy} instances and override placement
 * distances via {@link FieldPlannerWaypointPlacement} without editing planner internals.
 */
public record FieldPlannerWaypointConfig(
    boolean bandTransitionStagingEnabled,
    boolean occludingGateStagingEnabled,
    boolean centerReturnStagingEnabled,
    double centerBandMeters,
    double restageDistanceMeters,
    double gatePaddingMeters,
    double leadThroughScale,
    double leadThroughMinMeters,
    double leadThroughMaxMeters,
    double deepCenterBandMeters,
    double centerReturnStageTriggerMeters,
    double centerReturnIntersectionTriggerMeters,
    double centerReturnExitMinMeters,
    double centerReturnExitMaxMeters,
    double centerReturnGateMinOffsetMeters,
    double fieldEdgeMarginMeters,
    List<FieldPlannerWaypointPolicy> customPolicies,
    FieldPlannerWaypointPlacement placement) {
  public static FieldPlannerWaypointConfig defaults() {
    return new FieldPlannerWaypointConfig(
        true,
        true,
        true,
        3.648981,
        1.5,
        0.25,
        0.28,
        0.45,
        1.05,
        1.40,
        3.0,
        4.2,
        0.70,
        2.40,
        2.0,
        0.35,
        List.of(),
        FieldPlannerWaypointPlacement.defaults());
  }

  /**
   * Compatibility constructor for callers created before custom policies and placement tuning
   * existed. Equivalent to the canonical constructor with no custom policies and default placement.
   */
  public FieldPlannerWaypointConfig(
      boolean bandTransitionStagingEnabled,
      boolean occludingGateStagingEnabled,
      boolean centerReturnStagingEnabled,
      double centerBandMeters,
      double restageDistanceMeters,
      double gatePaddingMeters,
      double leadThroughScale,
      double leadThroughMinMeters,
      double leadThroughMaxMeters,
      double deepCenterBandMeters,
      double centerReturnStageTriggerMeters,
      double centerReturnIntersectionTriggerMeters,
      double centerReturnExitMinMeters,
      double centerReturnExitMaxMeters,
      double centerReturnGateMinOffsetMeters,
      double fieldEdgeMarginMeters) {
    this(
        bandTransitionStagingEnabled,
        occludingGateStagingEnabled,
        centerReturnStagingEnabled,
        centerBandMeters,
        restageDistanceMeters,
        gatePaddingMeters,
        leadThroughScale,
        leadThroughMinMeters,
        leadThroughMaxMeters,
        deepCenterBandMeters,
        centerReturnStageTriggerMeters,
        centerReturnIntersectionTriggerMeters,
        centerReturnExitMinMeters,
        centerReturnExitMaxMeters,
        centerReturnGateMinOffsetMeters,
        fieldEdgeMarginMeters,
        List.of(),
        FieldPlannerWaypointPlacement.defaults());
  }

  public FieldPlannerWaypointConfig {
    centerBandMeters = nonNegative(centerBandMeters, 3.648981);
    restageDistanceMeters = nonNegative(restageDistanceMeters, 1.5);
    gatePaddingMeters = nonNegative(gatePaddingMeters, 0.25);
    leadThroughScale = nonNegative(leadThroughScale, 0.28);
    leadThroughMinMeters = nonNegative(leadThroughMinMeters, 0.45);
    leadThroughMaxMeters = Math.max(leadThroughMinMeters, nonNegative(leadThroughMaxMeters, 1.05));
    deepCenterBandMeters = nonNegative(deepCenterBandMeters, 1.40);
    centerReturnStageTriggerMeters = nonNegative(centerReturnStageTriggerMeters, 3.0);
    centerReturnIntersectionTriggerMeters = nonNegative(centerReturnIntersectionTriggerMeters, 4.2);
    centerReturnExitMinMeters = nonNegative(centerReturnExitMinMeters, 0.70);
    centerReturnExitMaxMeters =
        Math.max(centerReturnExitMinMeters, nonNegative(centerReturnExitMaxMeters, 2.40));
    centerReturnGateMinOffsetMeters = nonNegative(centerReturnGateMinOffsetMeters, 2.0);
    fieldEdgeMarginMeters = nonNegative(fieldEdgeMarginMeters, 0.35);
    customPolicies = customPolicies == null ? List.of() : List.copyOf(customPolicies);
    placement = placement == null ? FieldPlannerWaypointPlacement.defaults() : placement;
  }

  public FieldPlannerWaypointConfig withBandTransitionStagingEnabled(boolean enabled) {
    return new FieldPlannerWaypointConfig(
        enabled,
        occludingGateStagingEnabled,
        centerReturnStagingEnabled,
        centerBandMeters,
        restageDistanceMeters,
        gatePaddingMeters,
        leadThroughScale,
        leadThroughMinMeters,
        leadThroughMaxMeters,
        deepCenterBandMeters,
        centerReturnStageTriggerMeters,
        centerReturnIntersectionTriggerMeters,
        centerReturnExitMinMeters,
        centerReturnExitMaxMeters,
        centerReturnGateMinOffsetMeters,
        fieldEdgeMarginMeters,
        customPolicies,
        placement);
  }

  public FieldPlannerWaypointConfig withOccludingGateStagingEnabled(boolean enabled) {
    return new FieldPlannerWaypointConfig(
        bandTransitionStagingEnabled,
        enabled,
        centerReturnStagingEnabled,
        centerBandMeters,
        restageDistanceMeters,
        gatePaddingMeters,
        leadThroughScale,
        leadThroughMinMeters,
        leadThroughMaxMeters,
        deepCenterBandMeters,
        centerReturnStageTriggerMeters,
        centerReturnIntersectionTriggerMeters,
        centerReturnExitMinMeters,
        centerReturnExitMaxMeters,
        centerReturnGateMinOffsetMeters,
        fieldEdgeMarginMeters,
        customPolicies,
        placement);
  }

  public FieldPlannerWaypointConfig withCenterReturnStagingEnabled(boolean enabled) {
    return new FieldPlannerWaypointConfig(
        bandTransitionStagingEnabled,
        occludingGateStagingEnabled,
        enabled,
        centerBandMeters,
        restageDistanceMeters,
        gatePaddingMeters,
        leadThroughScale,
        leadThroughMinMeters,
        leadThroughMaxMeters,
        deepCenterBandMeters,
        centerReturnStageTriggerMeters,
        centerReturnIntersectionTriggerMeters,
        centerReturnExitMinMeters,
        centerReturnExitMaxMeters,
        centerReturnGateMinOffsetMeters,
        fieldEdgeMarginMeters,
        customPolicies,
        placement);
  }

  public FieldPlannerWaypointConfig withLeadThroughMeters(double minMeters, double maxMeters) {
    return new FieldPlannerWaypointConfig(
        bandTransitionStagingEnabled,
        occludingGateStagingEnabled,
        centerReturnStagingEnabled,
        centerBandMeters,
        restageDistanceMeters,
        gatePaddingMeters,
        leadThroughScale,
        minMeters,
        maxMeters,
        deepCenterBandMeters,
        centerReturnStageTriggerMeters,
        centerReturnIntersectionTriggerMeters,
        centerReturnExitMinMeters,
        centerReturnExitMaxMeters,
        centerReturnGateMinOffsetMeters,
        fieldEdgeMarginMeters,
        customPolicies,
        placement);
  }

  public FieldPlannerWaypointConfig withCenterBandMeters(double meters) {
    return new FieldPlannerWaypointConfig(
        bandTransitionStagingEnabled,
        occludingGateStagingEnabled,
        centerReturnStagingEnabled,
        meters,
        restageDistanceMeters,
        gatePaddingMeters,
        leadThroughScale,
        leadThroughMinMeters,
        leadThroughMaxMeters,
        deepCenterBandMeters,
        centerReturnStageTriggerMeters,
        centerReturnIntersectionTriggerMeters,
        centerReturnExitMinMeters,
        centerReturnExitMaxMeters,
        centerReturnGateMinOffsetMeters,
        fieldEdgeMarginMeters,
        customPolicies,
        placement);
  }

  public FieldPlannerWaypointConfig withCustomPolicies(FieldPlannerWaypointPolicy... policies) {
    ArrayList<FieldPlannerWaypointPolicy> merged = new ArrayList<>(customPolicies);
    if (policies != null) {
      for (FieldPlannerWaypointPolicy policy : policies) {
        if (policy != null) merged.add(policy);
      }
    }
    return new FieldPlannerWaypointConfig(
        bandTransitionStagingEnabled,
        occludingGateStagingEnabled,
        centerReturnStagingEnabled,
        centerBandMeters,
        restageDistanceMeters,
        gatePaddingMeters,
        leadThroughScale,
        leadThroughMinMeters,
        leadThroughMaxMeters,
        deepCenterBandMeters,
        centerReturnStageTriggerMeters,
        centerReturnIntersectionTriggerMeters,
        centerReturnExitMinMeters,
        centerReturnExitMaxMeters,
        centerReturnGateMinOffsetMeters,
        fieldEdgeMarginMeters,
        merged,
        placement);
  }

  public FieldPlannerWaypointConfig withPlacement(FieldPlannerWaypointPlacement placement) {
    return new FieldPlannerWaypointConfig(
        bandTransitionStagingEnabled,
        occludingGateStagingEnabled,
        centerReturnStagingEnabled,
        centerBandMeters,
        restageDistanceMeters,
        gatePaddingMeters,
        leadThroughScale,
        leadThroughMinMeters,
        leadThroughMaxMeters,
        deepCenterBandMeters,
        centerReturnStageTriggerMeters,
        centerReturnIntersectionTriggerMeters,
        centerReturnExitMinMeters,
        centerReturnExitMaxMeters,
        centerReturnGateMinOffsetMeters,
        fieldEdgeMarginMeters,
        customPolicies,
        placement == null ? FieldPlannerWaypointPlacement.defaults() : placement);
  }

  private static double nonNegative(double value, double fallback) {
    return Double.isFinite(value) && value >= 0.0 ? value : fallback;
  }
}
