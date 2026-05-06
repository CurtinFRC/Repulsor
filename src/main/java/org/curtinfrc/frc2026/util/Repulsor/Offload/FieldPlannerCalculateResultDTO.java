package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Provides field planner calculate result dto functionality for the Repulsor offload serialization
 * and native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class FieldPlannerCalculateResultDTO {
  private double goalX;
  private double goalY;
  private double vxMetersPerSecond;
  private double vyMetersPerSecond;
  private double omegaRadians;
  private boolean hasErrMeters;
  private double errMeters;
  private double activeGoalX;
  private double activeGoalY;
  private double activeGoalThetaRadians;
  private boolean pathBlocked;
  private boolean globalFallbackActive;
  private boolean reactiveBypassActive;
  private boolean reactiveBypassPinned;
  private boolean forceThroughActive;
  private boolean robotIntersecting;
  private boolean stuckAbort;
  private boolean waypointActiveStage;
  private boolean waypointUsingBypass;
  private int waypointStagedModeTicks;
  private boolean hasGlobalFallbackWaypoint;
  private double globalFallbackWaypointX;
  private double globalFallbackWaypointY;
  private double globalFallbackWaypointThetaRadians;
  private boolean globalFallbackFound;
  private boolean globalFallbackTimedOut;
  private boolean globalFallbackExhaustedNodeBudget;
  private int globalFallbackExpandedNodes;
  private int globalFallbackGeneratedNodes;
  private int globalFallbackPathNodes;
  private long globalFallbackElapsedNanos;

  /**
   * Returns the get goal x value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getGoalX() {
    return goalX;
  }

  /**
   * Updates set goal x state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param goalX distance or field-coordinate value in meters.
   */
  public void setGoalX(double goalX) {
    this.goalX = goalX;
  }

  /**
   * Returns the get goal y value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getGoalY() {
    return goalY;
  }

  /**
   * Updates set goal y state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param goalY distance or field-coordinate value in meters.
   */
  public void setGoalY(double goalY) {
    this.goalY = goalY;
  }

  /**
   * Returns the get vx meters per second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getVxMetersPerSecond() {
    return vxMetersPerSecond;
  }

  /**
   * Updates set vx meters per second state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param vxMetersPerSecond distance or field-coordinate value in meters.
   */
  public void setVxMetersPerSecond(double vxMetersPerSecond) {
    this.vxMetersPerSecond = vxMetersPerSecond;
  }

  /**
   * Returns the get vy meters per second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getVyMetersPerSecond() {
    return vyMetersPerSecond;
  }

  /**
   * Updates set vy meters per second state or telemetry as part of the Repulsor runtime loop. This
   * may mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param vyMetersPerSecond distance or field-coordinate value in meters.
   */
  public void setVyMetersPerSecond(double vyMetersPerSecond) {
    this.vyMetersPerSecond = vyMetersPerSecond;
  }

  /**
   * Returns the get omega radians value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getOmegaRadians() {
    return omegaRadians;
  }

  /**
   * Updates set omega radians state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param omegaRadians value used by this operation.
   */
  public void setOmegaRadians(double omegaRadians) {
    this.omegaRadians = omegaRadians;
  }

  /**
   * Returns the is has err meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isHasErrMeters() {
    return hasErrMeters;
  }

  /**
   * Updates set has err meters state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param hasErrMeters distance or field-coordinate value in meters.
   */
  public void setHasErrMeters(boolean hasErrMeters) {
    this.hasErrMeters = hasErrMeters;
  }

  /**
   * Returns the get err meters value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getErrMeters() {
    return errMeters;
  }

  /**
   * Updates set err meters state or telemetry as part of the Repulsor runtime loop. This may mutate
   * local state, NetworkTables output, planner caches, or command-side runtime state depending on
   * the owning type.
   *
   * @param errMeters distance or field-coordinate value in meters.
   */
  public void setErrMeters(double errMeters) {
    this.errMeters = errMeters;
  }

  /**
   * Returns the get active goal x value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getActiveGoalX() {
    return activeGoalX;
  }

  /**
   * Updates set active goal x state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param activeGoalX distance or field-coordinate value in meters.
   */
  public void setActiveGoalX(double activeGoalX) {
    this.activeGoalX = activeGoalX;
  }

  /**
   * Returns the get active goal y value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getActiveGoalY() {
    return activeGoalY;
  }

  /**
   * Updates set active goal y state or telemetry as part of the Repulsor runtime loop. This may
   * mutate local state, NetworkTables output, planner caches, or command-side runtime state
   * depending on the owning type.
   *
   * @param activeGoalY distance or field-coordinate value in meters.
   */
  public void setActiveGoalY(double activeGoalY) {
    this.activeGoalY = activeGoalY;
  }

  /**
   * Returns the get active goal theta radians value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public double getActiveGoalThetaRadians() {
    return activeGoalThetaRadians;
  }

  /**
   * Updates set active goal theta radians state or telemetry as part of the Repulsor runtime loop.
   * This may mutate local state, NetworkTables output, planner caches, or command-side runtime
   * state depending on the owning type.
   *
   * @param activeGoalThetaRadians value used by this operation.
   */
  public void setActiveGoalThetaRadians(double activeGoalThetaRadians) {
    this.activeGoalThetaRadians = activeGoalThetaRadians;
  }

  public boolean isPathBlocked() {
    return pathBlocked;
  }

  public void setPathBlocked(boolean pathBlocked) {
    this.pathBlocked = pathBlocked;
  }

  public boolean isGlobalFallbackActive() {
    return globalFallbackActive;
  }

  public void setGlobalFallbackActive(boolean globalFallbackActive) {
    this.globalFallbackActive = globalFallbackActive;
  }

  public boolean isReactiveBypassActive() {
    return reactiveBypassActive;
  }

  public void setReactiveBypassActive(boolean reactiveBypassActive) {
    this.reactiveBypassActive = reactiveBypassActive;
  }

  public boolean isReactiveBypassPinned() {
    return reactiveBypassPinned;
  }

  public void setReactiveBypassPinned(boolean reactiveBypassPinned) {
    this.reactiveBypassPinned = reactiveBypassPinned;
  }

  public boolean isForceThroughActive() {
    return forceThroughActive;
  }

  public void setForceThroughActive(boolean forceThroughActive) {
    this.forceThroughActive = forceThroughActive;
  }

  public boolean isRobotIntersecting() {
    return robotIntersecting;
  }

  public void setRobotIntersecting(boolean robotIntersecting) {
    this.robotIntersecting = robotIntersecting;
  }

  public boolean isStuckAbort() {
    return stuckAbort;
  }

  public void setStuckAbort(boolean stuckAbort) {
    this.stuckAbort = stuckAbort;
  }

  public boolean isWaypointActiveStage() {
    return waypointActiveStage;
  }

  public void setWaypointActiveStage(boolean waypointActiveStage) {
    this.waypointActiveStage = waypointActiveStage;
  }

  public boolean isWaypointUsingBypass() {
    return waypointUsingBypass;
  }

  public void setWaypointUsingBypass(boolean waypointUsingBypass) {
    this.waypointUsingBypass = waypointUsingBypass;
  }

  public int getWaypointStagedModeTicks() {
    return waypointStagedModeTicks;
  }

  public void setWaypointStagedModeTicks(int waypointStagedModeTicks) {
    this.waypointStagedModeTicks = waypointStagedModeTicks;
  }

  public boolean isHasGlobalFallbackWaypoint() {
    return hasGlobalFallbackWaypoint;
  }

  public void setHasGlobalFallbackWaypoint(boolean hasGlobalFallbackWaypoint) {
    this.hasGlobalFallbackWaypoint = hasGlobalFallbackWaypoint;
  }

  public double getGlobalFallbackWaypointX() {
    return globalFallbackWaypointX;
  }

  public void setGlobalFallbackWaypointX(double globalFallbackWaypointX) {
    this.globalFallbackWaypointX = globalFallbackWaypointX;
  }

  public double getGlobalFallbackWaypointY() {
    return globalFallbackWaypointY;
  }

  public void setGlobalFallbackWaypointY(double globalFallbackWaypointY) {
    this.globalFallbackWaypointY = globalFallbackWaypointY;
  }

  public double getGlobalFallbackWaypointThetaRadians() {
    return globalFallbackWaypointThetaRadians;
  }

  public void setGlobalFallbackWaypointThetaRadians(double globalFallbackWaypointThetaRadians) {
    this.globalFallbackWaypointThetaRadians = globalFallbackWaypointThetaRadians;
  }

  public boolean isGlobalFallbackFound() {
    return globalFallbackFound;
  }

  public void setGlobalFallbackFound(boolean globalFallbackFound) {
    this.globalFallbackFound = globalFallbackFound;
  }

  public boolean isGlobalFallbackTimedOut() {
    return globalFallbackTimedOut;
  }

  public void setGlobalFallbackTimedOut(boolean globalFallbackTimedOut) {
    this.globalFallbackTimedOut = globalFallbackTimedOut;
  }

  public boolean isGlobalFallbackExhaustedNodeBudget() {
    return globalFallbackExhaustedNodeBudget;
  }

  public void setGlobalFallbackExhaustedNodeBudget(boolean globalFallbackExhaustedNodeBudget) {
    this.globalFallbackExhaustedNodeBudget = globalFallbackExhaustedNodeBudget;
  }

  public int getGlobalFallbackExpandedNodes() {
    return globalFallbackExpandedNodes;
  }

  public void setGlobalFallbackExpandedNodes(int globalFallbackExpandedNodes) {
    this.globalFallbackExpandedNodes = globalFallbackExpandedNodes;
  }

  public int getGlobalFallbackGeneratedNodes() {
    return globalFallbackGeneratedNodes;
  }

  public void setGlobalFallbackGeneratedNodes(int globalFallbackGeneratedNodes) {
    this.globalFallbackGeneratedNodes = globalFallbackGeneratedNodes;
  }

  public int getGlobalFallbackPathNodes() {
    return globalFallbackPathNodes;
  }

  public void setGlobalFallbackPathNodes(int globalFallbackPathNodes) {
    this.globalFallbackPathNodes = globalFallbackPathNodes;
  }

  public long getGlobalFallbackElapsedNanos() {
    return globalFallbackElapsedNanos;
  }

  public void setGlobalFallbackElapsedNanos(long globalFallbackElapsedNanos) {
    this.globalFallbackElapsedNanos = globalFallbackElapsedNanos;
  }
}
