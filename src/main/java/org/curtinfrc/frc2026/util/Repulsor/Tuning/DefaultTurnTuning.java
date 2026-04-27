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

package org.curtinfrc.frc2026.util.Repulsor.Tuning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Provides default turn tuning functionality for the Repulsor drive and turn tuning model layer.
 * Use this type from robot code, field profiles, or tests when integrating the corresponding
 * Repulsor subsystem. Coordinates are field-relative unless a method documents robot-relative
 * motion.
 */
public class DefaultTurnTuning extends TurnTuning {
  private double maxOmega = 6.0;
  private double turnMargin = 0.12;
  private int samples = 9;
  private double scoreMarginMult = 1.6;
  private double scoreBubble = 0.55;
  private double blendStart = 1.30;
  private double blendEnd = 0.25;
  private int blendSamples = 7;
  private double scoreSlowdown = 0.9;
  private double shortHopMeters = 0.80;
  private double minAlphaAtShort = 0.55;
  private double arcLookaheadTime = 0.28;
  private double maxAlphaFromAngleDeg = 110.0;
  private double collisionInflation = 0.09;
  private double reserveK0 = 0.30;
  private double reserveKAngle = 0.70;
  private double reserveKShort = 0.40;
  private double probeAngleDeg = 20.0;
  private double probeTime = 0.14;
  private double ttcBuffer = 0.05;
  private double snapDist = 0.15;
  private double snapAngleDeg = 22.0;

  private double lastAlpha = 0.0;
  private boolean turnLatched = false;
  private double lastTurnSign = 0.0;
  private double lastDistAlong = Double.POSITIVE_INFINITY;

  private double alphaEma = 0.55;
  private double triggerOnBias = 0.88;
  private double triggerOffGap = 0.28;

  private double alphaRateLimit = 3.0;
  private double cooldownS = 0.18;
  private double timeSinceLatchChange = 1e9;

  private double dynamicSnapK = 0.35;
  private double speedMarginK = 0.35;
  private double minAlphaWhenInfeasible = 0.32;
  private double angleDeadbandDeg = 2.0;

  private double yawRateGuardScale = 0.90;
  private double bigAngleDeg = 48.0;
  private double bigAngleAlphaMin = 0.38;
  private double ttcAggroBoost = 0.35;
  private double latchedSlowFloor = 0.72;

  /** Returns the default turn tuning value maintained by this Repulsor component. */
  public DefaultTurnTuning() {
    super("Turn/Default");
  }

  /**
   * Returns the with max omega value maintained by this Repulsor component.
   *
   * @param radps value used by this operation.
   * @return default turn tuning result for with max omega.
   */
  public DefaultTurnTuning withMaxOmega(double radps) {
    this.maxOmega = radps;
    return this;
  }

  /**
   * Returns the with turn margin value maintained by this Repulsor component.
   *
   * @param m value used by this operation.
   * @return default turn tuning result for with turn margin.
   */
  public DefaultTurnTuning withTurnMargin(double m) {
    this.turnMargin = m;
    return this;
  }

  /**
   * Returns the with samples value maintained by this Repulsor component.
   *
   * @param n value used by this operation.
   * @return default turn tuning result for with samples.
   */
  public DefaultTurnTuning withSamples(int n) {
    this.samples = n;
    return this;
  }

  /**
   * Returns the with score margin mult value maintained by this Repulsor component.
   *
   * @param m value used by this operation.
   * @return default turn tuning result for with score margin mult.
   */
  public DefaultTurnTuning withScoreMarginMult(double m) {
    this.scoreMarginMult = m;
    return this;
  }

  /**
   * Returns the with score bubble value maintained by this Repulsor component.
   *
   * @param m value used by this operation.
   * @return default turn tuning result for with score bubble.
   */
  public DefaultTurnTuning withScoreBubble(double m) {
    this.scoreBubble = m;
    return this;
  }

  /**
   * Returns the with blend window value maintained by this Repulsor component.
   *
   * @param startM value used by this operation.
   * @param endM value used by this operation.
   * @param n value used by this operation.
   * @return default turn tuning result for with blend window.
   */
  public DefaultTurnTuning withBlendWindow(double startM, double endM, int n) {
    this.blendStart = startM;
    this.blendEnd = endM;
    this.blendSamples = n;
    return this;
  }

  /**
   * Returns the with score slowdown value maintained by this Repulsor component.
   *
   * @param s value used by this operation.
   * @return default turn tuning result for with score slowdown.
   */
  public DefaultTurnTuning withScoreSlowdown(double s) {
    this.scoreSlowdown = s;
    return this;
  }

  /**
   * Returns the with short hop meters value maintained by this Repulsor component.
   *
   * @param m value used by this operation.
   * @return default turn tuning result for with short hop meters.
   */
  public DefaultTurnTuning withShortHopMeters(double m) {
    this.shortHopMeters = m;
    return this;
  }

  /**
   * Returns the with min alpha at short value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @return default turn tuning result for with min alpha at short.
   */
  public DefaultTurnTuning withMinAlphaAtShort(double a) {
    this.minAlphaAtShort = a;
    return this;
  }

  /**
   * Returns the with arc lookahead time value maintained by this Repulsor component.
   *
   * @param t value used by this operation.
   * @return default turn tuning result for with arc lookahead time.
   */
  public DefaultTurnTuning withArcLookaheadTime(double t) {
    this.arcLookaheadTime = t;
    return this;
  }

  /**
   * Returns the with max alpha from angle deg value maintained by this Repulsor component.
   *
   * @param d value used by this operation.
   * @return default turn tuning result for with max alpha from angle deg.
   */
  public DefaultTurnTuning withMaxAlphaFromAngleDeg(double d) {
    this.maxAlphaFromAngleDeg = d;
    return this;
  }

  /**
   * Returns the with collision inflation value maintained by this Repulsor component.
   *
   * @param m value used by this operation.
   * @return default turn tuning result for with collision inflation.
   */
  public DefaultTurnTuning withCollisionInflation(double m) {
    this.collisionInflation = m;
    return this;
  }

  /**
   * Returns the with reserve coeffs value maintained by this Repulsor component.
   *
   * @param k0 value used by this operation.
   * @param kAngle value used by this operation.
   * @param kShort value used by this operation.
   * @return default turn tuning result for with reserve coeffs.
   */
  public DefaultTurnTuning withReserveCoeffs(double k0, double kAngle, double kShort) {
    this.reserveK0 = k0;
    this.reserveKAngle = kAngle;
    this.reserveKShort = kShort;
    return this;
  }

  /**
   * Returns the with probe value maintained by this Repulsor component.
   *
   * @param angleDeg value used by this operation.
   * @param timeS value used by this operation.
   * @return default turn tuning result for with probe.
   */
  public DefaultTurnTuning withProbe(double angleDeg, double timeS) {
    this.probeAngleDeg = angleDeg;
    this.probeTime = timeS;
    return this;
  }

  /**
   * Returns the with snap value maintained by this Repulsor component.
   *
   * @param distM value used by this operation.
   * @param angleDeg value used by this operation.
   * @return default turn tuning result for with snap.
   */
  public DefaultTurnTuning withSnap(double distM, double angleDeg) {
    this.snapDist = distM;
    this.snapAngleDeg = angleDeg;
    return this;
  }

  /**
   * Returns the with ttc buffer value maintained by this Repulsor component.
   *
   * @param t value used by this operation.
   * @return default turn tuning result for with ttc buffer.
   */
  public DefaultTurnTuning withTtcBuffer(double t) {
    this.ttcBuffer = t;
    return this;
  }

  /**
   * Returns the with alpha ema value maintained by this Repulsor component.
   *
   * @param k value used by this operation.
   * @return default turn tuning result for with alpha ema.
   */
  public DefaultTurnTuning withAlphaEma(double k) {
    this.alphaEma = clamp01(k);
    return this;
  }

  /**
   * Returns the with trigger bias value maintained by this Repulsor component.
   *
   * @param onBias value used by this operation.
   * @param offGap value used by this operation.
   * @return default turn tuning result for with trigger bias.
   */
  public DefaultTurnTuning withTriggerBias(double onBias, double offGap) {
    this.triggerOnBias = Math.max(0.5, onBias);
    this.triggerOffGap = Math.max(0.05, offGap);
    return this;
  }

  /**
   * Returns the with alpha rate limit value maintained by this Repulsor component.
   *
   * @param perSec time value in seconds.
   * @return default turn tuning result for with alpha rate limit.
   */
  public DefaultTurnTuning withAlphaRateLimit(double perSec) {
    this.alphaRateLimit = Math.max(0.1, perSec);
    return this;
  }

  /**
   * Returns the with latch cooldown value maintained by this Repulsor component.
   *
   * @param seconds time value in seconds.
   * @return default turn tuning result for with latch cooldown.
   */
  public DefaultTurnTuning withLatchCooldown(double seconds) {
    this.cooldownS = Math.max(0.0, seconds);
    return this;
  }

  /**
   * Returns the with dynamic snap k value maintained by this Repulsor component.
   *
   * @param k value used by this operation.
   * @return default turn tuning result for with dynamic snap k.
   */
  public DefaultTurnTuning withDynamicSnapK(double k) {
    this.dynamicSnapK = Math.max(0.0, k);
    return this;
  }

  /**
   * Returns the with speed margin k value maintained by this Repulsor component.
   *
   * @param k value used by this operation.
   * @return default turn tuning result for with speed margin k.
   */
  public DefaultTurnTuning withSpeedMarginK(double k) {
    this.speedMarginK = Math.max(0.0, k);
    return this;
  }

  /**
   * Returns the with yaw rate guard scale value maintained by this Repulsor component.
   *
   * @param s value used by this operation.
   * @return default turn tuning result for with yaw rate guard scale.
   */
  public DefaultTurnTuning withYawRateGuardScale(double s) {
    this.yawRateGuardScale = clamp01(s);
    return this;
  }

  /** Runs apply defaults in the Repulsor runtime. */
  @Override
  public void applyDefaults() {
    setDtSeconds(0.02);
  }

  /**
   * Updates reset state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  @Override
  public void reset() {
    lastAlpha = 0.0;
    turnLatched = false;
    lastTurnSign = 0.0;
    lastDistAlong = Double.POSITIVE_INFINITY;
    timeSinceLatchChange = 1e9;
  }

  /**
   * Returns the max omega rad per sec value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public double maxOmegaRadPerSec() {
    return maxOmega;
  }

  /**
   * Returns the turn margin meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public double turnMarginMeters() {
    return turnMargin;
  }

  /**
   * Returns the turn samples value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public int turnSamples() {
    return samples;
  }

  /**
   * Computes the score turn margin mult value for the current Repulsor planning state.
   *
   * @return value produced by this operation.
   */
  @Override
  public double scoreTurnMarginMult() {
    return scoreMarginMult;
  }

  /**
   * Computes the score safety bubble meters value for the current Repulsor planning state.
   *
   * @return value produced by this operation.
   */
  @Override
  public double scoreSafetyBubbleMeters() {
    return scoreBubble;
  }

  /**
   * Returns the dock blend start meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public double dockBlendStartMeters() {
    return blendStart;
  }

  /**
   * Returns the dock blend end meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public double dockBlendEndMeters() {
    return blendEnd;
  }

  /**
   * Returns the dock blend samples value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  @Override
  public int dockBlendSamples() {
    return blendSamples;
  }

  /**
   * Computes the score turn slowdown factor value for the current Repulsor planning state.
   *
   * @return value produced by this operation.
   */
  @Override
  public double scoreTurnSlowdownFactor() {
    return scoreSlowdown;
  }

  /**
   * Returns the plan value maintained by this Repulsor component.
   *
   * @param pose WPILib Pose2d in field-relative coordinates.
   * @param goal value used by this operation.
   * @param pathHeading value used by this operation.
   * @param stepVec value used by this operation.
   * @param isScoring value used by this operation.
   * @param robotX distance or field-coordinate value in meters.
   * @param robotY distance or field-coordinate value in meters.
   * @param checker value used by this operation.
   * @return turn result result for plan.
   */
  @Override
  public TurnResult plan(
      Pose2d pose,
      Pose2d goal,
      Rotation2d pathHeading,
      Translation2d stepVec,
      boolean isScoring,
      double robotX,
      double robotY,
      CollisionChecker checker) {

    timeSinceLatchChange += dtSeconds();

    Rotation2d curYaw = pose.getRotation();
    Rotation2d goalYaw = goal.getRotation();

    double angErr =
        Math.atan2(
            Math.sin(goalYaw.minus(curYaw).getRadians()),
            Math.cos(goalYaw.minus(curYaw).getRadians()));
    double angMag = Math.abs(angErr);

    if (Math.toDegrees(angMag) < angleDeadbandDeg) angErr = 0.0;

    double tRot = angMag / Math.max(1e-6, maxOmegaRadPerSec());
    double v = stepVec.getNorm() / Math.max(1e-6, dtSeconds());
    double speedBias = clamp01(v / 3.0);

    double halfDiag = 0.5 * Math.hypot(robotX, robotY);
    double speedMarginScale = 1.0 + 0.7 * speedBias;
    double baseMargin = turnMarginMeters() + 0.5 * halfDiag + speedMarginK * speedMarginScale * v;
    double margin = baseMargin;
    if (!isScoring) {
      double collectExtra = 0.45 + 0.40 * speedBias;
      margin += collectExtra;
    }
    if (isScoring) margin *= scoreTurnMarginMult();

    Translation2d toGoalVec = goal.getTranslation().minus(pose.getTranslation());
    Rotation2d toGoalAng = (toGoalVec.getNorm() > 1e-6) ? toGoalVec.getAngle() : pathHeading;

    double dRemain = pose.getTranslation().getDistance(goal.getTranslation());
    double dAlong = dRemain;
    if (isScoring) {
      Translation2d rel = toGoalVec.rotateBy(goalYaw.unaryMinus());
      dAlong = Math.max(0.0, rel.getX());
    }

    double snapDistDyn = snapDist + dynamicSnapK * v;
    double snapAngDyn = Math.max(0.0, snapAngleDeg - 3.0 * Math.min(1.0, v));
    if (dAlong < snapDistDyn && Math.toDegrees(angMag) > snapAngDyn) {
      return new TurnResult(goalYaw, scoreSlowdown);
    }

    double safetyBubble = isScoring ? Math.max(scoreSafetyBubbleMeters(), halfDiag) : 0.0;

    double vForTurn = v;
    if (isScoring && dAlong <= (safetyBubble + margin)) {
      vForTurn = Math.min(v, Math.max(0.0, (dAlong - safetyBubble) / Math.max(1e-6, tRot)));
    }

    double dNeeded = vForTurn * tRot + margin;

    double angleBias = clamp01(angMag / Math.toRadians(maxAlphaFromAngleDeg));
    double shortBias =
        clamp01(
            (shortHopMeters - Math.min(shortHopMeters, dAlong)) / Math.max(1e-6, shortHopMeters));

    double riskBias = 0.0;
    {
      double angFrac = clamp01(angMag / Math.toRadians(maxAlphaFromAngleDeg));
      double adv =
          Math.max(
                  0.04, vForTurn * Math.max(probeTime * (0.6 + 0.4 * (1.0 - angFrac)), dtSeconds()))
              + 0.05;
      Rotation2d yawL = Rotation2d.fromRadians(curYaw.getRadians() + Math.toRadians(probeAngleDeg));
      Rotation2d yawR = Rotation2d.fromRadians(curYaw.getRadians() - Math.toRadians(probeAngleDeg));
      Translation2d posF = pose.getTranslation().plus(new Translation2d(adv, toGoalAng));
      Translation2d posN = pose.getTranslation().plus(new Translation2d(adv * 0.6, toGoalAng));
      double infXr = robotX + 2.0 * collisionInflation;
      double infYr = robotY + 2.0 * collisionInflation;
      if (checker.intersects(robotRect(posF, yawL, infXr, infYr)))
        riskBias = Math.max(riskBias, 1.0);
      if (checker.intersects(robotRect(posF, yawR, infXr, infYr)))
        riskBias = Math.max(riskBias, 1.0);
      if (checker.intersects(robotRect(posN, yawL, infXr, infYr)))
        riskBias = Math.max(riskBias, 0.6);
      if (checker.intersects(robotRect(posN, yawR, infXr, infYr)))
        riskBias = Math.max(riskBias, 0.6);
      if (riskBias == 0.0 && dRemain < shortHopMeters && angleBias > 0.25) riskBias = 0.35;
    }

    double ttc = Double.POSITIVE_INFINITY;
    double ttcBuf = ttcBuffer + 0.06 * speedBias;
    {
      double infX = robotX + 2.0 * collisionInflation;
      double infY = robotY + 2.0 * collisionInflation;
      double probeX = pose.getTranslation().getX();
      double probeY = pose.getTranslation().getY();
      double dirx = Math.cos(toGoalAng.getRadians());
      double diry = Math.sin(toGoalAng.getRadians());
      double maxProbe = Math.max(dAlong, 2.0);
      double travelled = 0.0;
      int iters = 0;
      int maxIters = 80;
      while (iters++ < maxIters && travelled <= maxProbe) {
        double stepM =
            (travelled < 0.6)
                ? Math.max(0.02, Math.min(0.12, vForTurn * 0.02))
                : Math.max(0.06, Math.min(0.25, vForTurn * 0.04));
        probeX += dirx * stepM;
        probeY += diry * stepM;
        travelled += stepM;
        if (checker.intersects(robotRect(new Translation2d(probeX, probeY), curYaw, infX, infY))) {
          ttc = travelled / Math.max(1e-6, vForTurn);
          break;
        }
      }
    }

    double distBias;
    if (isScoring) {
      double bStart = safetyBubble + margin + dockBlendStartMeters();
      double bEnd = Math.max(safetyBubble + margin + dockBlendEndMeters(), safetyBubble + 0.05);
      distBias = smooth01((bStart - dAlong) / Math.max(1e-6, (bStart - bEnd)));
    } else {
      double freeRange = margin + 1.2 + 1.6 * angleBias;
      freeRange += 0.6 * (1.0 + speedBias);
      distBias = smooth01((freeRange - dRemain) / Math.max(1e-6, freeRange));
    }

    double closeAngBias;
    if (isScoring) {
      double near = clamp01((safetyBubble + margin + 0.8 - dAlong) / (safetyBubble + margin + 0.8));
      closeAngBias = 0.5 * near * clamp01((angMag - Math.toRadians(10.0)) / Math.toRadians(50.0));
    } else {
      double near = clamp01((margin + 0.8 - dRemain) / (margin + 0.8));
      closeAngBias = 0.35 * near * clamp01((angMag - Math.toRadians(12.0)) / Math.toRadians(60.0));
    }

    double aggro = 0.0;
    if (Double.isFinite(ttc)) {
      double tRel = clamp01(((tRot + ttcBuf) - ttc) / (tRot + ttcBuf + 1e-6));
      aggro = ttcAggroBoost * Math.max(0.0, tRel);
    }

    double alphaTarget = clamp01(Math.max(distBias, minAlphaAtShort * shortBias));
    alphaTarget = clamp01(alphaTarget + 0.60 * angleBias + 0.45 * riskBias + closeAngBias + aggro);
    if (speedBias > 0.0) {
      alphaTarget =
          clamp01(alphaTarget + 0.30 * speedBias * angleBias + 0.22 * speedBias * riskBias);
    }

    boolean infeasible = (isScoring ? dAlong : dRemain) < dNeeded;
    if (infeasible) alphaTarget = Math.max(alphaTarget, minAlphaWhenInfeasible + 0.25 * angleBias);

    double reserve =
        reserveK0 + reserveKAngle * angleBias + reserveKShort * shortBias + 0.40 * riskBias;
    reserve += 0.20 * speedBias;
    if (!isScoring) {
      reserve += 0.35 * (0.3 + 0.7 * speedBias);
    }
    double triggerDist = dNeeded * (1.0 + reserve);

    boolean triggerDistGate = (isScoring ? dAlong : dRemain) <= triggerDist;
    boolean triggerTtcGate = (ttc < (tRot + ttcBuf));
    boolean triggerRaw = triggerDistGate || triggerTtcGate || infeasible;

    boolean earlyGate = (isScoring ? dAlong : dRemain) <= (triggerDist * triggerOnBias);
    if (earlyGate || triggerRaw) {
      if (!turnLatched) timeSinceLatchChange = 0.0;
      turnLatched = true;
    } else {
      boolean clearDist = (isScoring ? dAlong : dRemain) > (triggerDist * (1.0 + triggerOffGap));
      if (turnLatched
          && clearDist
          && Math.abs(angErr) < Math.toRadians(6.0)
          && timeSinceLatchChange >= cooldownS) {
        turnLatched = false;
        timeSinceLatchChange = 0.0;
      }
    }

    boolean trigger = turnLatched;

    double alphaFloor = 0.25 + 0.65 * angleBias + 0.35 * riskBias + 0.30 * speedBias;
    if (!isScoring) alphaFloor += 0.10 * speedBias;
    double alphaPre = trigger ? Math.max(alphaTarget, clamp01(alphaFloor)) : alphaTarget;

    double curSign = Math.signum(angErr == 0.0 ? lastTurnSign : angErr);
    if (lastTurnSign != 0.0 && curSign != 0.0 && (curSign != lastTurnSign)) {
      if (Math.abs(angErr) < Math.toRadians(10.0)) alphaPre *= 0.65;
    }

    if (Math.toDegrees(angMag) > bigAngleDeg) alphaPre = Math.max(alphaPre, bigAngleAlphaMin);

    double aSmoothed = alphaEma * alphaPre + (1.0 - alphaEma) * lastAlpha;
    double alpha = clamp01(aSmoothed);
    if (!trigger) {
      double s =
          sigmoid((isScoring ? (dAlong - (safetyBubble + margin)) : (dRemain - margin)) * (-3.0));
      double base = 0.65 + 0.35 * s;
      double extra = 0.20 * speedBias + (!isScoring ? 0.15 * speedBias : 0.0);
      alpha = clamp01(alpha * (base + extra));
    }

    double maxDelta = alphaRateLimit * dtSeconds();
    double delta = alpha - lastAlpha;
    if (delta > maxDelta) alpha = lastAlpha + maxDelta;
    if (delta < -maxDelta) alpha = lastAlpha - maxDelta;
    alpha = clamp01(alpha);

    double maxYawThisStep = yawRateGuardScale * maxOmegaRadPerSec() * dtSeconds();
    if (Math.abs(angErr * alpha) > maxYawThisStep)
      alpha = maxYawThisStep / Math.max(1e-9, Math.abs(angErr));

    double yawBlend = curYaw.getRadians() + angErr * alpha;
    Rotation2d blendedYaw = Rotation2d.fromRadians(yawBlend);

    int baseN = isScoring ? dockBlendSamples() : turnSamples();
    int n =
        Math.max(
            10,
            baseN
                + (int) Math.ceil(8 * angleBias)
                + (shortBias > 0.25 ? 4 : 0)
                + (riskBias > 0.0 ? 6 : 0));
    n += Math.min(6, (int) Math.floor(v * 2.0));

    double horizonBase = Math.max(arcLookaheadTime, dtSeconds() * n);
    double horizon = horizonBase + 0.30 * speedBias;
    double stepTime = horizon / n;
    double stepAdvance = Math.max(0.01, vForTurn * stepTime);

    double infX = robotX + 2.0 * collisionInflation;
    double infY = robotY + 2.0 * collisionInflation;

    double simX = pose.getTranslation().getX();
    double simY = pose.getTranslation().getY();

    for (int i = 1; i <= n; i++) {
      double a = clamp01(alpha * ((double) i / n));
      double yaw_i = curYaw.getRadians() + angErr * a;
      Rotation2d face = Rotation2d.fromRadians(yaw_i);
      double dx, dy;
      if (isScoring) {
        double mix = clamp01((double) i / n);
        double gx = Math.cos(goalYaw.getRadians());
        double gy = Math.sin(goalYaw.getRadians());
        double fx = Math.cos(face.getRadians());
        double fy = Math.sin(face.getRadians());
        double mx = fx * (1.0 - mix) + gx * mix;
        double my = fy * (1.0 - mix) + gy * mix;
        double norm = Math.hypot(mx, my);
        mx /= (norm + 1e-9);
        my /= (norm + 1e-9);
        dx = stepAdvance * mx;
        dy = stepAdvance * my;
      } else {
        double w0 = 0.20 + 0.80 * ((double) i / n);
        double wv = clamp01(0.20 + 0.20 * v);
        double w = clamp01(w0 + wv * 0.5);
        double tx = Math.cos(toGoalAng.getRadians());
        double ty = Math.sin(toGoalAng.getRadians());
        double fx = Math.cos(face.getRadians());
        double fy = Math.sin(face.getRadians());
        double mx = fx * (1.0 - w) + tx * w;
        double my = fy * (1.0 - w) + ty * w;
        double norm = Math.hypot(mx, my);
        mx /= (norm + 1e-9);
        my /= (norm + 1e-9);
        dx = stepAdvance * mx;
        dy = stepAdvance * my;
      }
      simX += dx;
      simY += dy;
      Translation2d[] rect_i = robotRect(new Translation2d(simX, simY), face, infX, infY);
      if (checker.intersects(rect_i)) {
        double prevA = clamp01(alpha * ((double) (i - 1) / n));
        double safeYaw = curYaw.getRadians() + angErr * prevA;
        Rotation2d safe = Rotation2d.fromRadians(safeYaw);
        double reAimSlow = 1.0;
        if (isScoring) {
          double bigTurn = clamp01((angMag - Math.toRadians(18.0)) / Math.toRadians(60.0));
          double shortish =
              clamp01((shortHopMeters - Math.min(shortHopMeters, dAlong)) / shortHopMeters);
          reAimSlow = clamp01(1.0 - 0.18 * bigTurn * shortish);
        }
        double slow;
        if (isScoring) {
          double close = clamp01(1.0 - Math.min(1.0, dAlong / (safetyBubble + margin + 0.5)));
          slow =
              clamp01(
                  Math.max(
                      latchedSlowFloor,
                      (scoreTurnSlowdownFactor() + 0.25 * close + 0.20 * riskBias) * reAimSlow));
        } else {
          slow = clamp01((1.0 - 0.30 * prevA - 0.20 * riskBias) * reAimSlow);
        }
        lastAlpha = alpha;
        lastTurnSign = (Math.signum(angErr) == 0.0 ? lastTurnSign : Math.signum(angErr));
        lastDistAlong = dAlong;
        return new TurnResult(safe, slow);
      }
    }

    if (isScoring) {
      double close = clamp01(1.0 - Math.min(1.0, dAlong / (safetyBubble + margin + 0.5)));
      double reAimSlow = 1.0;
      double bigTurn = clamp01((angMag - Math.toRadians(18.0)) / Math.toRadians(60.0));
      double shortish =
          clamp01((shortHopMeters - Math.min(shortHopMeters, dAlong)) / shortHopMeters);
      reAimSlow = clamp01(1.0 - 0.18 * bigTurn * shortish);
      double slow =
          clamp01(
              Math.max(
                  latchedSlowFloor,
                  (1.0 - (1.0 - scoreTurnSlowdownFactor()) * alpha + 0.25 * close + 0.10 * riskBias)
                      * reAimSlow));
      lastAlpha = alpha;
      lastTurnSign = (Math.signum(angErr) == 0.0 ? lastTurnSign : Math.signum(angErr));
      lastDistAlong = dAlong;
      return new TurnResult(blendedYaw, slow);
    }

    if (alpha > 1e-3) {
      double slow = clamp01(1.0 - 0.24 * alpha - 0.12 * riskBias);
      lastAlpha = alpha;
      lastTurnSign = (Math.signum(angErr) == 0.0 ? lastTurnSign : Math.signum(angErr));
      lastDistAlong = dAlong;
      return new TurnResult(blendedYaw, slow);
    }

    lastAlpha = alpha;
    lastTurnSign = (Math.signum(angErr) == 0.0 ? lastTurnSign : Math.signum(angErr));
    lastDistAlong = dAlong;
    return new TurnResult(goalYaw, 1.0);
  }
}
