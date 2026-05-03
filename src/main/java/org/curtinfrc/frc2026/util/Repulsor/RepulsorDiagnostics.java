package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;

/** Lightweight diagnostics helpers for warning paths that must not spam the robot loop. */
public final class RepulsorDiagnostics {
  private static final ConcurrentMap<String, Long> LAST_WARNING_NS = new ConcurrentHashMap<>();

  private RepulsorDiagnostics() {}

  /** Emits a warning at most once per key. */
  public static void warnOnce(String key, String message) {
    warnThrottled(key, message, Double.POSITIVE_INFINITY);
  }

  /** Emits a warning at most every {@code periodSeconds} for the supplied key. */
  public static void warnThrottled(String key, String message, double periodSeconds) {
    long now = System.nanoTime();
    long periodNs =
        Double.isFinite(periodSeconds)
            ? Math.max(0L, (long) (periodSeconds * 1_000_000_000.0))
            : Long.MAX_VALUE;

    Long previous = LAST_WARNING_NS.get(key);
    if (previous != null && now - previous < periodNs) {
      return;
    }
    LAST_WARNING_NS.put(key, now);

    String full = "[Repulsor] " + message;
    System.err.println(full);
    try {
      NetworkTableInstance.getDefault()
          .getTable("Repulsor/Diagnostics")
          .getEntry(key)
          .setString(full);
    } catch (RuntimeException ignored) {
      // Diagnostics must never affect control-loop behavior.
    }
  }

  /** Returns the number of warning keys emitted. Intended for tests. */
  public static int warningKeyCount() {
    return LAST_WARNING_NS.size();
  }
}
