package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;

/** Owns the SpatialDyn snapshot cache for {@link PredictiveFieldStateOps}. */
final class PredictiveSpatialDynCache {
  private volatile List<DynamicObject> lastDynRef = null;
  private volatile SpatialDyn lastDyn = null;
  private volatile int lastDynSpecsVersion = -1;

  SpatialDyn cached(PredictiveFieldStateOps ops) {
    List<DynamicObject> ref = ops.dynamicObjects;
    int sv = ops.specsVersion;
    SpatialDyn d = lastDyn;
    if (d != null && ref == lastDynRef && sv == lastDynSpecsVersion) return d;
    SpatialDyn nd =
        new SpatialDyn(
            ref,
            ops.resourceSpecs,
            ops.otherTypeWeights,
            ops.collectResourceTypes,
            ops.collectResourcePositionFilter,
            ops.collectionProfile.observationHardMaxAgeSeconds(),
            ops.collectionProfile.observationAgeDecay());
    lastDynRef = ref;
    lastDyn = nd;
    lastDynSpecsVersion = sv;
    return nd;
  }

  void invalidate() {
    lastDynRef = null;
    lastDyn = null;
    lastDynSpecsVersion = -1;
  }
}
