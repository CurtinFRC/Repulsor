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

package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Provides heatmap functionality for the Repulsor core Repulsor coordination layer. Use this type
 * from robot code, field profiles, or tests when integrating the corresponding Repulsor subsystem.
 * Coordinates are field-relative unless a method documents robot-relative motion.
 */
public final class Heatmap {
  /**
   * Contract for heatmap provider implementations used by the Repulsor core Repulsor coordination
   * layer. Use this type from robot code, field profiles, or tests when integrating the
   * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
   * robot-relative motion.
   */
  public static interface HeatmapProvider {
    /**
     * Returns the get heatmap value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    Heatmap getHeatmap();
  }

  /**
   * Provides block functionality for the Repulsor core Repulsor coordination layer. Use this type
   * from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static final class Block {
    /**
     * Configuration value for uid. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final String uid;

    /**
     * Configuration value for position. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final Translation2d position;

    /**
     * Configuration value for size. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final Transform2d size;

    /**
     * Configuration value for heat. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final double heat;

    /**
     * Returns the block value maintained by this Repulsor component.
     *
     * @param uid value used by this operation.
     * @param position value used by this operation.
     * @param size value used by this operation.
     * @param heat value used by this operation.
     */
    public Block(String uid, Translation2d position, Transform2d size, double heat) {
      this.uid = requireUid(uid);
      this.position = Objects.requireNonNull(position, "position");
      this.size = Objects.requireNonNull(size, "size");
      this.heat = heat;
    }

    /**
     * Returns the x0 value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public double x0() {
      return position.getX();
    }

    /**
     * Returns the y0 value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public double y0() {
      return position.getY();
    }

    /**
     * Returns w for the current Repulsor state.
     *
     * @return value produced by this operation.
     */
    public double w() {
      return Math.max(0.0, size.getTranslation().getX());
    }

    /**
     * Returns h for the current Repulsor state.
     *
     * @return value produced by this operation.
     */
    public double h() {
      return Math.max(0.0, size.getTranslation().getY());
    }

    /**
     * Returns the x1 value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public double x1() {
      return x0() + w();
    }

    /**
     * Returns the y1 value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public double y1() {
      return y0() + h();
    }

    /**
     * Returns the center value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public Translation2d center() {
      return new Translation2d(x0() + 0.5 * w(), y0() + 0.5 * h());
    }

    /**
     * Returns the contains value maintained by this Repulsor component.
     *
     * @param p value used by this operation.
     * @return value produced by this operation.
     */
    public boolean contains(Translation2d p) {
      double x = p.getX();
      double y = p.getY();
      return x >= x0() && x <= x1() && y >= y0() && y <= y1();
    }

    /**
     * Returns the area value maintained by this Repulsor component.
     *
     * @return value produced by this operation.
     */
    public double area() {
      return w() * h();
    }
  }

  /**
   * Provides transition functionality for the Repulsor core Repulsor coordination layer. Use this
   * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static final class Transition {
    /**
     * Configuration value for from. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final String from;

    /**
     * Configuration value for to. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final String to;

    /**
     * Configuration value for scale. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final double scale;

    /**
     * Returns the transition value maintained by this Repulsor component.
     *
     * @param from value used by this operation.
     * @param to value used by this operation.
     * @param scale value used by this operation.
     */
    public Transition(String from, String to, double scale) {
      this.from = requireUid(from);
      this.to = requireUid(to);
      this.scale = scale;
    }
  }

  /**
   * Defines the blend mode values used by the Repulsor core Repulsor coordination layer. Use this
   * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public enum BlendMode {
    MIN_T,
    MAX_EDGE_INFLUENCE,
    WEIGHTED_SUM
  }

  /**
   * Provides builder functionality for the Repulsor core Repulsor coordination layer. Use this type
   * from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static final class Builder {
    private final Map<String, Block> blocks = new LinkedHashMap<>();
    private final List<Transition> transitions = new ArrayList<>();
    private double eps = 1e-9;
    private BlendMode blendMode = BlendMode.MAX_EDGE_INFLUENCE;

    /**
     * Returns the eps value maintained by this Repulsor component.
     *
     * @param eps value used by this operation.
     * @return value produced by this operation.
     */
    public Builder eps(double eps) {
      this.eps = Math.max(0.0, eps);
      return this;
    }

    /**
     * Returns the blend mode value maintained by this Repulsor component.
     *
     * @param mode value used by this operation.
     * @return value produced by this operation.
     */
    public Builder blendMode(BlendMode mode) {
      this.blendMode = Objects.requireNonNull(mode, "mode");
      return this;
    }

    /**
     * Returns the block value maintained by this Repulsor component.
     *
     * @param uid value used by this operation.
     * @param pos value used by this operation.
     * @param w value used by this operation.
     * @param h value used by this operation.
     * @param heat value used by this operation.
     * @return value produced by this operation.
     */
    public Builder block(String uid, Translation2d pos, double w, double h, double heat) {
      return block(
          new Block(
              uid,
              pos,
              new Transform2d(
                  new Translation2d(w, h), edu.wpi.first.math.geometry.Rotation2d.kZero),
              heat));
    }

    /**
     * Returns the block value maintained by this Repulsor component.
     *
     * @param b value used by this operation.
     * @return value produced by this operation.
     */
    public Builder block(Block b) {
      Objects.requireNonNull(b, "b");
      if (blocks.containsKey(b.uid))
        throw new IllegalArgumentException("Duplicate block uid: " + b.uid);
      blocks.put(b.uid, b);
      return this;
    }

    /**
     * Returns the transition value maintained by this Repulsor component.
     *
     * @param from value used by this operation.
     * @param to value used by this operation.
     * @param scale value used by this operation.
     * @return value produced by this operation.
     */
    public Builder transition(String from, String to, double scale) {
      transitions.add(new Transition(from, to, scale));
      return this;
    }

    /**
     * Returns the bidirectional value maintained by this Repulsor component.
     *
     * @param a value used by this operation.
     * @param b value used by this operation.
     * @param scaleAB value used by this operation.
     * @param scaleBA value used by this operation.
     * @return value produced by this operation.
     */
    public Builder bidirectional(String a, String b, double scaleAB, double scaleBA) {
      transitions.add(new Transition(a, b, scaleAB));
      transitions.add(new Transition(b, a, scaleBA));
      return this;
    }

    /**
     * Builds the WPILib command sequence for the current behaviour context.
     *
     * @return value produced by this operation.
     */
    public Heatmap build() {
      return new Heatmap(new ArrayList<>(blocks.values()), transitions, eps, blendMode);
    }
  }

  /**
   * Returns the builder value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static Builder builder() {
    return new Builder();
  }

  /**
   * Defines the side values used by the Repulsor core Repulsor coordination layer. Use this type
   * from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public enum Side {
    LEFT,
    RIGHT,
    BOTTOM,
    TOP
  }

  /**
   * Provides transition zone functionality for the Repulsor core Repulsor coordination layer. Use
   * this type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public static final class TransitionZone {
    /**
     * Configuration value for from. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final String from;

    /**
     * Configuration value for to. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final String to;

    /**
     * Configuration value for side in to. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public final Side sideInTo;

    /**
     * Configuration value for thickness. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    public final double thickness;

    /**
     * Configuration value for scale. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    public final double scale;

    private TransitionZone(String from, String to, Side sideInTo, double thickness, double scale) {
      this.from = from;
      this.to = to;
      this.sideInTo = sideInTo;
      this.thickness = thickness;
      this.scale = scale;
    }
  }

  private static final class Incoming {
    /**
     * Configuration value for from. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final Block from;

    /**
     * Configuration value for to. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final Block to;

    /**
     * Configuration value for side. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final Side side;

    /**
     * Configuration value for thickness. The valid range and tuning source are defined by the
     * owning subsystem or field profile.
     */
    final double thickness;

    /**
     * Configuration value for scale. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final double scale;

    Incoming(Block from, Block to, Side side, double thickness, double scale) {
      this.from = from;
      this.to = to;
      this.side = side;
      this.thickness = thickness;
      this.scale = scale;
    }

    boolean contains(Translation2d p) {
      if (!to.contains(p)) return false;
      if (thickness <= 1e-12) return false;

      double x = p.getX();
      double y = p.getY();
      switch (side) {
        case LEFT:
          return (x - to.x0()) <= thickness;
        case RIGHT:
          return (to.x1() - x) <= thickness;
        case BOTTOM:
          return (y - to.y0()) <= thickness;
        case TOP:
          return (to.y1() - y) <= thickness;
        default:
          return false;
      }
    }

    double t(Translation2d p) {
      if (thickness <= 1e-12) return 1.0;
      double x = p.getX();
      double y = p.getY();
      double d;
      switch (side) {
        case LEFT:
          d = x - to.x0();
          break;
        case RIGHT:
          d = to.x1() - x;
          break;
        case BOTTOM:
          d = y - to.y0();
          break;
        case TOP:
          d = to.y1() - y;
          break;
        default:
          d = thickness;
          break;
      }
      return clamp(d / thickness, 0.0, 1.0);
    }

    double influence(Translation2d p) {
      if (!contains(p)) return 0.0;
      return 1.0 - t(p);
    }

    double blendedHeatAt(Translation2d p) {
      return lerp(from.heat, to.heat, t(p));
    }
  }

  private final List<Block> blocks;
  private final List<Transition> transitions;
  private final double eps;
  private final BlendMode blendMode;

  private final Map<String, Block> byUid = new HashMap<>();
  private final Map<String, List<Incoming>> incomingByTo = new HashMap<>();

  private Heatmap(
      List<Block> blocks, List<Transition> transitions, double eps, BlendMode blendMode) {
    this.blocks = Collections.unmodifiableList(new ArrayList<>(blocks));
    this.transitions = Collections.unmodifiableList(new ArrayList<>(transitions));
    this.eps = eps;
    this.blendMode = blendMode;

    for (Block b : this.blocks) {
      Block prev = byUid.putIfAbsent(b.uid, b);
      if (prev != null) throw new IllegalArgumentException("Duplicate block uid: " + b.uid);
    }

    buildIncoming();
  }

  /**
   * Returns the blocks value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public List<Block> blocks() {
    return blocks;
  }

  /**
   * Returns the transitions value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public List<Transition> transitions() {
    return transitions;
  }

  /**
   * Returns the block value maintained by this Repulsor component.
   *
   * @param uid value used by this operation.
   * @return value produced by this operation.
   */
  public Block block(String uid) {
    return byUid.get(uid);
  }

  /**
   * Returns the total heat value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double totalHeat() {
    double sum = 0.0;
    for (Block b : blocks) sum += b.heat;
    return sum;
  }

  /**
   * Returns the block at value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return value produced by this operation.
   */
  public Block blockAt(Translation2d p) {
    Objects.requireNonNull(p, "p");
    for (Block b : blocks) {
      if (b.contains(p)) return b;
    }
    return null;
  }

  /**
   * Returns the heat at value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return value produced by this operation.
   */
  public double heatAt(Translation2d p) {
    Objects.requireNonNull(p, "p");
    Block b = blockAt(p);
    if (b == null) return 0.0;

    List<Incoming> incoming = incomingByTo.get(b.uid);
    if (incoming == null || incoming.isEmpty()) return b.heat;

    switch (blendMode) {
      case MIN_T:
        return heatAtMinT(p, b, incoming);
      case WEIGHTED_SUM:
        return heatAtWeightedSum(p, b, incoming);
      case MAX_EDGE_INFLUENCE:
      default:
        return heatAtMaxInfluence(p, b, incoming);
    }
  }

  /**
   * Returns the are touching value maintained by this Repulsor component.
   *
   * @param aUid value used by this operation.
   * @param bUid value used by this operation.
   * @return value produced by this operation.
   */
  public boolean areTouching(String aUid, String bUid) {
    Block a = byUid.get(aUid);
    Block b = byUid.get(bUid);
    if (a == null || b == null) return false;
    return detectTouch(a, b, eps) != null;
  }

  /**
   * Returns the zones for to value maintained by this Repulsor component.
   *
   * @param toUid value used by this operation.
   * @return list of transition zone values produced by this operation.
   */
  public List<TransitionZone> zonesForTo(String toUid) {
    List<Incoming> inc = incomingByTo.get(toUid);
    if (inc == null || inc.isEmpty()) return Collections.emptyList();
    List<TransitionZone> out = new ArrayList<>(inc.size());
    for (Incoming i : inc) {
      out.add(new TransitionZone(i.from.uid, i.to.uid, i.side, i.thickness, i.scale));
    }
    return Collections.unmodifiableList(out);
  }

  /**
   * Returns the zones value maintained by this Repulsor component.
   *
   * @return list of transition zone values produced by this operation.
   */
  public List<TransitionZone> zones() {
    List<TransitionZone> out = new ArrayList<>();
    for (Map.Entry<String, List<Incoming>> e : incomingByTo.entrySet()) {
      for (Incoming i : e.getValue()) {
        out.add(new TransitionZone(i.from.uid, i.to.uid, i.side, i.thickness, i.scale));
      }
    }
    return Collections.unmodifiableList(out);
  }

  private double heatAtMinT(Translation2d p, Block to, List<Incoming> incoming) {
    double bestT = Double.POSITIVE_INFINITY;
    double best = to.heat;
    for (Incoming inc : incoming) {
      if (!inc.contains(p)) continue;
      double t = inc.t(p);
      if (t < bestT) {
        bestT = t;
        best = lerp(inc.from.heat, to.heat, t);
      }
    }
    return best;
  }

  private double heatAtMaxInfluence(Translation2d p, Block to, List<Incoming> incoming) {
    double bestInf = 0.0;
    double bestHeat = to.heat;
    for (Incoming inc : incoming) {
      double inf = inc.influence(p);
      if (inf > bestInf) {
        bestInf = inf;
        bestHeat = inc.blendedHeatAt(p);
      }
    }
    return bestHeat;
  }

  private double heatAtWeightedSum(Translation2d p, Block to, List<Incoming> incoming) {
    double base = to.heat;
    double wBase = 1.0;
    double sum = base * wBase;
    double wSum = wBase;

    for (Incoming inc : incoming) {
      double inf = inc.influence(p);
      if (inf <= 0.0) continue;
      double h = inc.blendedHeatAt(p);
      double w = inf;
      sum += h * w;
      wSum += w;
    }
    return (wSum <= 1e-12) ? base : (sum / wSum);
  }

  private void buildIncoming() {
    incomingByTo.clear();
    for (Transition tr : transitions) {
      Block from = byUid.get(tr.from);
      Block to = byUid.get(tr.to);
      if (from == null || to == null) continue;

      Touch touch = detectTouch(from, to, eps);
      if (touch == null) continue;

      double s = clamp(tr.scale, 0.0, 1.0);
      double thickness;
      switch (touch.side) {
        case LEFT:
        case RIGHT:
          thickness = s * to.w();
          break;
        case TOP:
        case BOTTOM:
          thickness = s * to.h();
          break;
        default:
          thickness = 0.0;
          break;
      }

      Incoming inc = new Incoming(from, to, touch.side, Math.max(0.0, thickness), s);
      incomingByTo.computeIfAbsent(to.uid, k -> new ArrayList<>()).add(inc);
    }
  }

  private static final class Touch {
    /**
     * Configuration value for side. The valid range and tuning source are defined by the owning
     * subsystem or field profile.
     */
    final Side side;

    Touch(Side side) {
      this.side = side;
    }
  }

  private static Touch detectTouch(Block from, Block to, double eps) {
    double fx0 = from.x0(), fx1 = from.x1();
    double fy0 = from.y0(), fy1 = from.y1();
    double tx0 = to.x0(), tx1 = to.x1();
    double ty0 = to.y0(), ty1 = to.y1();

    boolean yOverlap = overlapPositive(fy0, fy1, ty0, ty1);
    boolean xOverlap = overlapPositive(fx0, fx1, tx0, tx1);

    if (yOverlap && nearlyEqual(fx1, tx0, eps)) return new Touch(Side.LEFT);
    if (yOverlap && nearlyEqual(fx0, tx1, eps)) return new Touch(Side.RIGHT);
    if (xOverlap && nearlyEqual(fy1, ty0, eps)) return new Touch(Side.BOTTOM);
    if (xOverlap && nearlyEqual(fy0, ty1, eps)) return new Touch(Side.TOP);

    return null;
  }

  private static boolean overlapPositive(double a0, double a1, double b0, double b1) {
    double lo = Math.max(Math.min(a0, a1), Math.min(b0, b1));
    double hi = Math.min(Math.max(a0, a1), Math.max(b0, b1));
    return hi - lo > 1e-12;
  }

  private static boolean nearlyEqual(double a, double b, double eps) {
    return Math.abs(a - b) <= eps;
  }

  private static double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * clamp(t, 0.0, 1.0);
  }

  private static String requireUid(String uid) {
    if (uid == null) throw new IllegalArgumentException("uid is null");
    String u = uid.trim();
    if (u.isEmpty()) throw new IllegalArgumentException("uid is empty");
    return u;
  }
}
