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

package org.curtinfrc.frc2026.util.Repulsor.Setpoints;

import org.curtinfrc.frc2026.util.Repulsor.Setpoints.Specific.*;

/**
 * Provides setpoints functionality for the Repulsor game setpoint abstraction layer for
 * field-relative goals and mechanisms. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public class Setpoints {
  /**
   * Provides rebuilt2026 functionality for the Repulsor game setpoint abstraction layer for
   * field-relative goals and mechanisms. Use this type from robot code, field profiles, or tests
   * when integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
   * method documents robot-relative motion.
   */
  public static class Rebuilt2026 extends _Rebuilt2026 {
    private Rebuilt2026() {}
  }

  /**
   * Provides reefscape2025 functionality for the Repulsor game setpoint abstraction layer for
   * field-relative goals and mechanisms. Use this type from robot code, field profiles, or tests
   * when integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a
   * method documents robot-relative motion.
   */
  public static class Reefscape2025 extends _Reefscape2025 {
    private Reefscape2025() {}
  }
}
