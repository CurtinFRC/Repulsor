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

package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElementModel;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameObject;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Pipe;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.PrimitiveObject;

/**
 * Provides field map builder functionality for the Repulsor field/profile definition layer used to
 * tune Repulsor for a specific game. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class FieldMapBuilder {
  /**
   * Defines the category spec values used by the Repulsor field/profile definition layer used to
   * tune Repulsor for a specific game. Use this type from robot code, field profiles, or tests when
   * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
   * method documents robot-relative motion.
   */
  public enum CategorySpec {
    kScore,
    kCollect,
    kEndgame
  }

  /**
   * Provides element spec functionality for the Repulsor field/profile definition layer used to
   * tune Repulsor for a specific game. Use this type from robot code, field profiles, or tests when
   * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
   * method documents robot-relative motion.
   */
  public static final class ElementSpec {
    Alliance alliance = Alliance.kBlue;
    int capacity = 1;
    Pose3d pose = new Pose3d();
    final List<PrimitiveObject> primitives = new ArrayList<>();
    Predicate<GameObject> filter = go -> true;
    RepulsorSetpoint related;
    CategorySpec category = CategorySpec.kScore;
  }

  private final FieldTrackerCore ft;
  private final List<GameElement> elements = new ArrayList<>();
  private ElementSpec spec;

  /**
   * Returns the field map builder value maintained by this Repulsor component.
   *
   * @param ft value used by this operation.
   */
  public FieldMapBuilder(FieldTrackerCore ft) {
    this.ft = Objects.requireNonNull(ft);
  }

  private ElementSpec s() {
    if (spec == null) throw new IllegalStateException("Call begin() first");
    return spec;
  }

  /**
   * Returns the begin value maintained by this Repulsor component.
   *
   * @return field map builder result for begin.
   */
  public FieldMapBuilder begin() {
    spec = new ElementSpec();
    return this;
  }

  /**
   * Returns the alliance value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @return field map builder result for alliance.
   */
  public FieldMapBuilder alliance(Alliance a) {
    s().alliance = a;
    return this;
  }

  /**
   * Returns the capacity value maintained by this Repulsor component.
   *
   * @param c value used by this operation.
   * @return field map builder result for capacity.
   */
  public FieldMapBuilder capacity(int c) {
    s().capacity = c;
    return this;
  }

  /**
   * Returns the pose value maintained by this Repulsor component.
   *
   * @param p value used by this operation.
   * @return field map builder result for pose.
   */
  public FieldMapBuilder pose(Pose3d p) {
    s().pose = p;
    return this;
  }

  /**
   * Returns the rotate value maintained by this Repulsor component.
   *
   * @param rollRad value used by this operation.
   * @param pitchRad value used by this operation.
   * @param yawRad value used by this operation.
   * @return field map builder result for rotate.
   */
  public FieldMapBuilder rotate(double rollRad, double pitchRad, double yawRad) {
    Pose3d p = s().pose;
    s().pose = new Pose3d(p.getX(), p.getY(), p.getZ(), new Rotation3d(rollRad, pitchRad, yawRad));
    return this;
  }

  /**
   * Returns the translate value maintained by this Repulsor component.
   *
   * @param dx distance or field-coordinate value in meters.
   * @param dy distance or field-coordinate value in meters.
   * @param dz value used by this operation.
   * @return field map builder result for translate.
   */
  public FieldMapBuilder translate(double dx, double dy, double dz) {
    Pose3d p = s().pose;
    s().pose = new Pose3d(p.getX() + dx, p.getY() + dy, p.getZ() + dz, p.getRotation());
    return this;
  }

  /**
   * Returns the category value maintained by this Repulsor component.
   *
   * @param c value used by this operation.
   * @return field map builder result for category.
   */
  public FieldMapBuilder category(CategorySpec c) {
    s().category = c != null ? c : CategorySpec.kScore;
    return this;
  }

  /**
   * Returns the primitive pipe value maintained by this Repulsor component.
   *
   * @param radius value used by this operation.
   * @param rollRad value used by this operation.
   * @param pitchRad value used by this operation.
   * @param yawRad value used by this operation.
   * @return field map builder result for primitive pipe.
   */
  public FieldMapBuilder primitivePipe(
      Distance radius, double rollRad, double pitchRad, double yawRad) {
    Pose3d p = s().pose;
    s().primitives
        .add(
            new Pipe(
                new Pose3d(p.getX(), p.getY(), p.getZ(), new Rotation3d(rollRad, pitchRad, yawRad)),
                radius,
                Radians.of(yawRad)));
    return this;
  }

  /**
   * Returns the primitive floor square value maintained by this Repulsor component.
   *
   * @param sideMeters distance or field-coordinate value in meters.
   * @param zMinMeters distance or field-coordinate value in meters.
   * @param zMaxMeters distance or field-coordinate value in meters.
   * @return field map builder result for primitive floor square.
   */
  public FieldMapBuilder primitiveFloorSquare(
      double sideMeters, double zMinMeters, double zMaxMeters) {
    if (sideMeters <= 0) throw new IllegalArgumentException("sideMeters must be > 0");
    if (zMaxMeters < zMinMeters)
      throw new IllegalArgumentException("zMaxMeters must be >= zMinMeters");
    Pose3d p = s().pose;
    double cx = p.getX();
    double cy = p.getY();
    double half = sideMeters * 0.5;
    s().primitives
        .add(
            new PrimitiveObject() {
              @Override
              public boolean intersects(Pose3d pos) {
                double x = pos.getX();
                double y = pos.getY();
                double z = pos.getZ();
                if (z < zMinMeters || z > zMaxMeters) return false;
                return Math.abs(x - cx) <= half && Math.abs(y - cy) <= half;
              }
            });
    return this;
  }

  /**
   * Returns the filter value maintained by this Repulsor component.
   *
   * @param f value used by this operation.
   * @return field map builder result for filter.
   */
  public FieldMapBuilder filter(Predicate<GameObject> f) {
    s().filter = f != null ? f : (go -> true);
    return this;
  }

  private static String canonType(Object raw) {
    if (raw == null) return "";
    String t = String.valueOf(raw).trim();
    if (t.isEmpty()) return "";
    t = t.replace(' ', '_').replace('-', '_');
    if (t.length() >= 2
        && (t.charAt(0) == 'k' || t.charAt(0) == 'K')
        && Character.isUpperCase(t.charAt(1))) {
      t = t.substring(1);
    }
    t = t.toLowerCase();
    if (t.startsWith("k_")) t = t.substring(2);
    if (t.startsWith("k")) t = t.substring(1);
    return t;
  }

  /**
   * Returns the filter type value maintained by this Repulsor component.
   *
   * @param allowed value used by this operation.
   * @return field map builder result for filter type.
   */
  public FieldMapBuilder filterType(String... allowed) {
    Set<String> set = new HashSet<>();
    if (allowed != null) {
      for (String a : allowed) {
        if (a == null) continue;
        String c = canonType(a);
        if (!c.isEmpty()) set.add(c);
      }
    }
    if (set.isEmpty()) {
      s().filter = go -> true;
      return this;
    }
    s().filter =
        go -> {
          if (go == null) return false;
          String c = canonType(go.getType());
          return set.contains(c);
        };
    return this;
  }

  /**
   * Returns the related value maintained by this Repulsor component.
   *
   * @param sp value used by this operation.
   * @return field map builder result for related.
   */
  public FieldMapBuilder related(RepulsorSetpoint sp) {
    s().related = sp;
    return this;
  }

  /**
   * Returns the add value maintained by this Repulsor component.
   *
   * @return field map builder result for add.
   */
  public FieldMapBuilder add() {
    ElementSpec es = s();
    PrimitiveObject[] prim = es.primitives.toArray(new PrimitiveObject[0]);
    GameElementModel model = new GameElementModel(es.pose, prim);
    elements.add(
        new GameElement(es.alliance, es.capacity, model, es.filter, es.related, es.category));
    spec = null;
    return this;
  }

  /**
   * Returns the bulk value maintained by this Repulsor component.
   *
   * @param poses value used by this operation.
   * @param alliance value used by this operation.
   * @param capacity distance or field-coordinate value in meters.
   * @param radius value used by this operation.
   * @param yawRad value used by this operation.
   * @param filter value used by this operation.
   * @param related value used by this operation.
   * @param category distance or field-coordinate value in meters.
   * @return field map builder result for bulk.
   */
  public FieldMapBuilder bulk(
      List<Pose3d> poses,
      Alliance alliance,
      int capacity,
      Distance radius,
      double yawRad,
      Predicate<GameObject> filter,
      List<RepulsorSetpoint> related,
      CategorySpec category) {
    int n = poses != null ? poses.size() : 0;
    int r = related != null ? related.size() : 0;
    int m = r > 0 ? Math.min(n, r) : n;
    for (int i = 0; i < m; i++) {
      begin()
          .alliance(alliance)
          .capacity(capacity)
          .pose(poses.get(i))
          .primitivePipe(radius, 0, 0, yawRad)
          .filter(filter)
          .related(r > 0 ? related.get(i) : null)
          .category(category)
          .add();
    }
    return this;
  }

  /**
   * Returns the mirror x value maintained by this Repulsor component.
   *
   * @param xAxis value used by this operation.
   * @return field map builder result for mirror x.
   */
  public FieldMapBuilder mirrorX(double xAxis) {
    List<GameElement> mirrored = new ArrayList<>();
    for (GameElement e : elements) {
      Pose3d p = e.getModel().getPosition();
      Pose3d mp = new Pose3d(2 * xAxis - p.getX(), p.getY(), p.getZ(), p.getRotation());
      PrimitiveObject[] prims = e.getModel().getComposition();
      List<PrimitiveObject> nprims = new ArrayList<>();
      for (PrimitiveObject pr : prims) {
        if (pr instanceof Pipe) {
          Pipe pipe = (Pipe) pr;
          Pose3d pp = pipe.getPosition();
          nprims.add(
              new Pipe(
                  new Pose3d(2 * xAxis - pp.getX(), pp.getY(), pp.getZ(), pp.getRotation()),
                  pipe.getRadius(),
                  pipe.getAngle()));
        }
      }
      GameElementModel model = new GameElementModel(mp, nprims.toArray(new PrimitiveObject[0]));
      GameElement m =
          new GameElement(
              e.getAlliance(),
              e.getMaxContained(),
              model,
              e::filter,
              e.getRelatedPoint().orElse(null),
              e.getCategory());
      mirrored.add(m);
    }
    elements.addAll(mirrored);
    return this;
  }

  /**
   * Builds the WPILib command sequence for the current behaviour context.
   *
   * @return game element[] result for build.
   */
  public GameElement[] build() {
    return elements.toArray(new GameElement[0]);
  }

  /**
   * Returns the small value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static Distance small() {
    return Meters.of(0.20);
  }

  /**
   * Returns the medium value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static Distance medium() {
    return Meters.of(0.30);
  }

  /**
   * Returns the large value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static Distance large() {
    return Meters.of(0.40);
  }

  /**
   * Returns the tiny value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public static Distance tiny() {
    return Meters.of(0.10);
  }
}
