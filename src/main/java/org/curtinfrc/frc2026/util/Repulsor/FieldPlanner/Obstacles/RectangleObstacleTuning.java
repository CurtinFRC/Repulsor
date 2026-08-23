package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles;

/** Tunable constants for {@link RectangleObstacle} flow-assist behavior. */
public record RectangleObstacleTuning(
    double xAxisAngleBiasRad,
    double teardropEdgeOffsetMin,
    double teardropEdgeOffsetExtra,
    double teardropEdgeOffsetMax,
    double teardropGoalBias,
    double edgeTearBlend,
    double desiredEdgeClearMeters,
    double desiredCornerClearMeters) {

  public static RectangleObstacleTuning defaults() {
    return new RectangleObstacleTuning(
        Math.toRadians(18.0), 0.18, 0.45, 0.50, 0.65, 0.70, 0.24, 0.30);
  }

  public RectangleObstacleTuning {
    xAxisAngleBiasRad = finiteOr(xAxisAngleBiasRad, Math.toRadians(18.0));
    teardropEdgeOffsetMin = nonNegative(teardropEdgeOffsetMin);
    teardropEdgeOffsetExtra = nonNegative(teardropEdgeOffsetExtra);
    teardropEdgeOffsetMax = nonNegative(teardropEdgeOffsetMax);
    teardropGoalBias = finiteOr(teardropGoalBias, 0.65);
    edgeTearBlend = finiteOr(edgeTearBlend, 0.70);
    desiredEdgeClearMeters = nonNegative(desiredEdgeClearMeters);
    desiredCornerClearMeters = nonNegative(desiredCornerClearMeters);
  }

  private static double finiteOr(double value, double fallback) {
    return Double.isFinite(value) ? value : fallback;
  }

  private static double nonNegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
