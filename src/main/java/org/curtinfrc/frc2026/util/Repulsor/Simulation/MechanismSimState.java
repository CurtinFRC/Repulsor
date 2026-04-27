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

package org.curtinfrc.frc2026.util.Repulsor.Simulation;

/**
 * Mutable state container for a single degree of freedom mechanism simulation.
 *
 * <p>Units follow the mechanism conventions selected by the sim. Position and velocity are in the
 * output units of the mechanism, such as radians for rotary mechanisms or meters for linear
 * mechanisms.
 */
public final class MechanismSimState {
  private double position;
  private double velocity;
  private double acceleration;
  private double appliedVoltage;
  private double motorCurrentAmps;
  private double motorTorqueNm;
  private double outputEffort;
  private double externalEffort;
  private double frictionEffort;
  private double pressureEffort;
  private boolean lowerLimitTriggered;
  private boolean upperLimitTriggered;

  /** Creates a new state with all values set to zero. */
  public MechanismSimState() {}

  /**
   * Creates a new state by copying another state.
   *
   * @param other state to copy
   */
  public MechanismSimState(MechanismSimState other) {
    copyFrom(other);
  }

  /**
   * Returns a copy of this state.
   *
   * @return copied state
   */
  public MechanismSimState copy() {
    return new MechanismSimState(this);
  }

  /**
   * Returns the mechanism position in output units.
   *
   * @return position in output units
   */
  public double getPosition() {
    return position;
  }

  /**
   * Returns the mechanism velocity in output units per second.
   *
   * @return velocity in output units per second
   */
  public double getVelocity() {
    return velocity;
  }

  /**
   * Returns the mechanism acceleration in output units per second squared.
   *
   * @return acceleration in output units per second squared
   */
  public double getAcceleration() {
    return acceleration;
  }

  /**
   * Returns the applied motor voltage after saturation.
   *
   * @return applied voltage in volts
   */
  public double getAppliedVoltage() {
    return appliedVoltage;
  }

  /**
   * Returns the simulated motor current draw.
   *
   * @return motor current in amps
   */
  public double getMotorCurrentAmps() {
    return motorCurrentAmps;
  }

  /**
   * Returns the simulated motor torque.
   *
   * @return motor torque in newton meters
   */
  public double getMotorTorqueNm() {
    return motorTorqueNm;
  }

  /**
   * Returns the net output effort after loads and friction.
   *
   * @return output effort in torque or force units
   */
  public double getOutputEffort() {
    return outputEffort;
  }

  /**
   * Returns the summed external effort from loads and contacts before friction.
   *
   * @return external effort in torque or force units
   */
  public double getExternalEffort() {
    return externalEffort;
  }

  /**
   * Returns the friction effort applied by the friction model.
   *
   * @return friction effort in torque or force units
   */
  public double getFrictionEffort() {
    return frictionEffort;
  }

  /**
   * Returns the effort contributed by a pressure model if one is configured.
   *
   * @return pressure effort in torque or force units
   */
  public double getPressureEffort() {
    return pressureEffort;
  }

  /**
   * Returns whether the lower travel limit is triggered.
   *
   * @return true if the lower limit is triggered
   */
  public boolean isLowerLimitTriggered() {
    return lowerLimitTriggered;
  }

  /**
   * Returns whether the upper travel limit is triggered.
   *
   * @return true if the upper limit is triggered
   */
  public boolean isUpperLimitTriggered() {
    return upperLimitTriggered;
  }

  /**
   * Runs copy from in the Repulsor runtime.
   *
   * @param other value used by this operation.
   */
  void copyFrom(MechanismSimState other) {
    if (other == null) {
      return;
    }
    position = other.position;
    velocity = other.velocity;
    acceleration = other.acceleration;
    appliedVoltage = other.appliedVoltage;
    motorCurrentAmps = other.motorCurrentAmps;
    motorTorqueNm = other.motorTorqueNm;
    outputEffort = other.outputEffort;
    externalEffort = other.externalEffort;
    frictionEffort = other.frictionEffort;
    pressureEffort = other.pressureEffort;
    lowerLimitTriggered = other.lowerLimitTriggered;
    upperLimitTriggered = other.upperLimitTriggered;
  }

  /**
   * Updates set position state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param position value used by this operation.
   */
  void setPosition(double position) {
    this.position = position;
  }

  /**
   * Updates set velocity state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param velocity velocity input, normally field-relative unless the caller documents
   *     robot-relative motion.
   */
  void setVelocity(double velocity) {
    this.velocity = velocity;
  }

  /**
   * Updates set acceleration state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param acceleration value used by this operation.
   */
  void setAcceleration(double acceleration) {
    this.acceleration = acceleration;
  }

  /**
   * Updates set applied voltage state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param appliedVoltage value used by this operation.
   */
  void setAppliedVoltage(double appliedVoltage) {
    this.appliedVoltage = appliedVoltage;
  }

  /**
   * Updates set motor current amps state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param motorCurrentAmps value used by this operation.
   */
  void setMotorCurrentAmps(double motorCurrentAmps) {
    this.motorCurrentAmps = motorCurrentAmps;
  }

  /**
   * Updates set motor torque nm state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param motorTorqueNm value used by this operation.
   */
  void setMotorTorqueNm(double motorTorqueNm) {
    this.motorTorqueNm = motorTorqueNm;
  }

  /**
   * Updates set output effort state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param outputEffort value used by this operation.
   */
  void setOutputEffort(double outputEffort) {
    this.outputEffort = outputEffort;
  }

  /**
   * Updates set external effort state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param externalEffort value used by this operation.
   */
  void setExternalEffort(double externalEffort) {
    this.externalEffort = externalEffort;
  }

  /**
   * Updates set friction effort state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param frictionEffort value used by this operation.
   */
  void setFrictionEffort(double frictionEffort) {
    this.frictionEffort = frictionEffort;
  }

  /**
   * Updates set pressure effort state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param pressureEffort value used by this operation.
   */
  void setPressureEffort(double pressureEffort) {
    this.pressureEffort = pressureEffort;
  }

  /**
   * Updates set lower limit triggered state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param lowerLimitTriggered value used by this operation.
   */
  void setLowerLimitTriggered(boolean lowerLimitTriggered) {
    this.lowerLimitTriggered = lowerLimitTriggered;
  }

  /**
   * Updates set upper limit triggered state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param upperLimitTriggered value used by this operation.
   */
  void setUpperLimitTriggered(boolean upperLimitTriggered) {
    this.upperLimitTriggered = upperLimitTriggered;
  }
}
