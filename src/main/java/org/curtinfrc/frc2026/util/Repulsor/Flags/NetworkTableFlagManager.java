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

package org.curtinfrc.frc2026.util.Repulsor.Flags;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Set;

/**
 * Provides network table flag manager functionality for the Repulsor runtime flag source used to
 * enable and disable behaviours. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class NetworkTableFlagManager<E extends Enum<E>> extends FlagManager<E> {
  private final NetworkTable table;
  private final EnumMap<E, NetworkTableEntry> entries;

  /**
   * Returns the network table flag manager value maintained by this Repulsor component.
   *
   * @param enumClass value used by this operation.
   * @param tablePath value used by this operation.
   */
  public NetworkTableFlagManager(Class<E> enumClass, String tablePath) {
    this.table = NetworkTableInstance.getDefault().getTable(tablePath);
    this.entries = new EnumMap<>(enumClass);
    for (E f : enumClass.getEnumConstants()) {
      entries.put(f, table.getEntry(f.name()));
      entries.get(f).setDefaultBoolean(false);
    }
  }

  /**
   * Returns the get active flags value maintained by this Repulsor component.
   *
   * @return enum set of e result for get active flags.
   */
  @Override
  public EnumSet<E> getActiveFlags() {
    EnumSet<E> active = EnumSet.noneOf(entries.keySet().iterator().next().getDeclaringClass());
    for (var e : entries.entrySet()) {
      if (e.getValue().getBoolean(false)) active.add(e.getKey());
    }
    return active;
  }

  /**
   * Runs add flag in the Repulsor runtime.
   *
   * @param flag value used by this operation.
   */
  @Override
  public void addFlag(E flag) {
    setFlag(flag, true);
  }

  /**
   * Runs remove flag in the Repulsor runtime.
   *
   * @param flag value used by this operation.
   */
  @Override
  public void removeFlag(E flag) {
    setFlag(flag, false);
  }

  /**
   * Runs toggle flag in the Repulsor runtime.
   *
   * @param flag value used by this operation.
   */
  @Override
  public void toggleFlag(E flag) {
    setFlag(flag, !entries.get(flag).getBoolean(false));
  }

  /**
   * Updates clear flags state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   */
  @Override
  public void clearFlags() {
    for (E f : entries.keySet()) setFlag(f, false);
  }

  /**
   * Runs add all in the Repulsor runtime.
   *
   * @param flags value used by this operation.
   */
  @Override
  public void addAll(Set<E> flags) {
    for (E f : flags) setFlag(f, true);
  }

  /**
   * Runs remove all in the Repulsor runtime.
   *
   * @param flags value used by this operation.
   */
  @Override
  public void removeAll(Set<E> flags) {
    for (E f : flags) setFlag(f, false);
  }

  /**
   * Updates set flag state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   *
   * @param flag value used by this operation.
   * @param value value used by this operation.
   */
  public void setFlag(E flag, boolean value) {
    entries.get(flag).setBoolean(value);
  }

  /**
   * Returns the get table value maintained by this Repulsor component.
   *
   * @return network table result for get table.
   */
  public NetworkTable getTable() {
    return table;
  }
}
