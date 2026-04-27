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

package org.curtinfrc.frc2026.util.Repulsor.Target;

/**
 * Provides sticky target state functionality for the Repulsor sticky-target filtering and
 * target-selection layer. Use this type from robot code, field profiles, or tests when integrating
 * the corresponding Repulsor subsystem. Coordinates are field-relative unless a method documents
 * robot-relative motion.
 */
final class StickyTargetState<T> {
  T lastOut;

  /**
   * Configuration value for last out change sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  double lastOutChangeSec = -1e9;

  /**
   * Configuration value for sticky invalid since sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  double stickyInvalidSinceSec = -1e9;

  T bestValidKey;

  /**
   * Configuration value for best missing since sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  double bestMissingSinceSec = -1e9;

  /**
   * Configuration value for best valid since sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  double bestValidSinceSec = -1e9;

  T lastBestSeen;

  /**
   * Configuration value for best last change sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  double bestLastChangeSec = -1e9;

  /**
   * Configuration value for flicker since sec. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  double flickerSinceSec = -1e9;

  T sticky;

  /**
   * Configuration value for sticky since sec. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  double stickySinceSec = -1e9;

  /**
   * Configuration value for sticky last best seen sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  double stickyLastBestSeenSec = -1e9;

  T candidate;

  /**
   * Configuration value for candidate since sec. Time values use seconds and should be tuned
   * against measured robot loop and mechanism latency.
   */
  double candidateSinceSec = -1e9;

  T lastSticky;

  /**
   * Configuration value for last switch sec. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  double lastSwitchSec = -1e9;

  /**
   * Configuration value for last update sec. Time values use seconds and should be tuned against
   * measured robot loop and mechanism latency.
   */
  double lastUpdateSec = 0.0;

  /**
   * Updates clear state or telemetry as part of the Repulsor runtime loop. This may mutate local
   * state, NetworkTables output, planner caches, or command-side runtime state depending on the
   * owning type.
   */
  void clear() {
    lastOut = null;
    lastOutChangeSec = -1e9;

    stickyInvalidSinceSec = -1e9;

    bestValidKey = null;
    bestMissingSinceSec = -1e9;
    bestValidSinceSec = -1e9;

    lastBestSeen = null;
    bestLastChangeSec = -1e9;
    flickerSinceSec = -1e9;

    sticky = null;
    stickySinceSec = -1e9;
    stickyLastBestSeenSec = -1e9;

    candidate = null;
    candidateSinceSec = -1e9;

    lastSticky = null;
    lastSwitchSec = -1e9;

    lastUpdateSec = 0.0;
  }
}
