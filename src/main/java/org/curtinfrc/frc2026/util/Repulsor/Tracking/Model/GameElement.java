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

package org.curtinfrc.frc2026.util.Repulsor.Tracking.Model;

import java.util.Optional;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

/**
 * Provides game element functionality for the Repulsor typed model layer for field objects and
 * prediction inputs. Use this type from robot code, field profiles, or tests when integrating the
 * corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
public class GameElement {
  private final GameObject[] containedStorage;
  private int containedCount;
  private int maxContained;

  private GameElementModel model;
  private Predicate<GameObject> filter;
  private Optional<RepulsorSetpoint> relatedPoint = Optional.empty();
  private CategorySpec category;
  private Alliance alliance;

  private GameObject[] cachedExact;
  private boolean dirtyCache;

  /**
   * Returns the game element value maintained by this Repulsor component.
   *
   * @param model value used by this operation.
   */
  public GameElement(GameElementModel model) {
    this(Alliance.kBlue, 10, model, g -> true, null, CategorySpec.kScore);
  }

  /**
   * Returns the game element value maintained by this Repulsor component.
   *
   * @param alliance value used by this operation.
   * @param maxContained value used by this operation.
   * @param model value used by this operation.
   * @param filter value used by this operation.
   * @param relatedPoint value used by this operation.
   * @param category distance or field-coordinate value in meters.
   */
  public GameElement(
      Alliance alliance,
      int maxContained,
      GameElementModel model,
      Predicate<GameObject> filter,
      RepulsorSetpoint relatedPoint,
      CategorySpec category) {
    if (alliance == null) throw new IllegalArgumentException("Alliance cannot be null");
    if (maxContained < 0) throw new IllegalArgumentException("Max contained cannot be negative");
    this.alliance = alliance;
    this.maxContained = maxContained;
    this.model = model;
    this.filter = (filter != null) ? filter : (go -> true);
    if (relatedPoint != null) this.relatedPoint = Optional.ofNullable(relatedPoint);
    this.category = category;
    this.containedStorage = new GameObject[Math.max(1, maxContained)];
    this.containedCount = 0;
    this.cachedExact = new GameObject[0];
    this.dirtyCache = true;
  }

  /**
   * Returns the filter value maintained by this Repulsor component.
   *
   * @param gameObject value used by this operation.
   * @return value produced by this operation.
   */
  public boolean filter(GameObject gameObject) {
    if (gameObject == null) throw new IllegalArgumentException("GameObject cannot be null");
    return filter.test(gameObject);
  }

  /**
   * Returns the get alliance value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public Alliance getAlliance() {
    return alliance;
  }

  /**
   * Returns the get category value maintained by this Repulsor component.
   *
   * @return category spec result for get category.
   */
  public CategorySpec getCategory() {
    return category;
  }

  /**
   * Updates set alliance state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param alliance value used by this operation.
   */
  public void setAlliance(Alliance alliance) {
    if (alliance == null) throw new IllegalArgumentException("Alliance cannot be null");
    this.alliance = alliance;
  }

  /**
   * Returns the get max contained value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public int getMaxContained() {
    return maxContained;
  }

  /**
   * Updates set max contained state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param maxContained value used by this operation.
   */
  public void setMaxContained(int maxContained) {
    if (maxContained < 0) throw new IllegalArgumentException("Max contained cannot be negative");
    this.maxContained = maxContained;
    if (containedCount > maxContained) {
      containedCount = maxContained;
      dirtyCache = true;
    }
  }

  /**
   * Returns the get contained value maintained by this Repulsor component.
   *
   * @return game object[] result for get contained.
   */
  public GameObject[] getContained() {
    if (!dirtyCache && cachedExact.length == containedCount) {
      return cachedExact.clone();
    }
    GameObject[] out = new GameObject[containedCount];
    if (containedCount > 0) {
      System.arraycopy(containedStorage, 0, out, 0, containedCount);
    }
    cachedExact = out;
    dirtyCache = false;
    return out.clone();
  }

  /**
   * Updates set contained state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param contained value used by this operation.
   */
  public void setContained(GameObject[] contained) {
    clearContained();
    if (contained == null || contained.length == 0) return;
    int n = Math.min(contained.length, maxContained);
    for (int i = 0; i < n; i++) {
      containedStorage[i] = contained[i];
    }
    containedCount = n;
    dirtyCache = true;
  }

  /**
   * Returns the get contained value maintained by this Repulsor component.
   *
   * @param index distance or field-coordinate value in meters.
   * @return game object result for get contained.
   */
  public GameObject getContained(int index) {
    if (index < 0 || index >= containedCount) {
      throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + containedCount);
    }
    return containedStorage[index];
  }

  /**
   * Returns the get contained count value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public int getContainedCount() {
    return containedCount;
  }

  /**
   * Returns the is at capacity value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isAtCapacity() {
    return containedCount >= maxContained;
  }

  /**
   * Returns the get model value maintained by this Repulsor component.
   *
   * @return game element model result for get model.
   */
  public GameElementModel getModel() {
    return model;
  }

  /**
   * Updates set model state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param model value used by this operation.
   */
  public void setModel(GameElementModel model) {
    this.model = model;
  }

  /**
   * Returns the get related point value maintained by this Repulsor component.
   *
   * @return optional repulsor setpoint produced by this operation.
   */
  public Optional<RepulsorSetpoint> getRelatedPoint() {
    return relatedPoint;
  }

  /**
   * Updates set related point state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param newPoint value used by this operation.
   */
  public void setRelatedPoint(RepulsorSetpoint newPoint) {
    this.relatedPoint = Optional.ofNullable(newPoint);
  }

  /**
   * Updates set filter state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param filter value used by this operation.
   */
  public void setFilter(Predicate<GameObject> filter) {
    this.filter = (filter != null) ? filter : (go -> true);
  }

  /**
   * Updates set category state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param category distance or field-coordinate value in meters.
   */
  public void setCategory(CategorySpec category) {
    this.category = category;
  }

  /**
   * Updates clear contained state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   */
  public void clearContained() {
    if (containedCount != 0) {
      for (int i = 0; i < containedCount; i++) {
        containedStorage[i] = null;
      }
      containedCount = 0;
      dirtyCache = true;
    }
  }

  /**
   * Returns the try add value maintained by this Repulsor component.
   *
   * @param obj value used by this operation.
   * @return value produced by this operation.
   */
  public boolean tryAdd(GameObject obj) {
    if (obj == null) return false;
    if (containedCount >= maxContained) return false;
    containedStorage[containedCount++] = obj;
    dirtyCache = true;
    return true;
  }
}
