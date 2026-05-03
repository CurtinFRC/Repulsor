package org.curtinfrc.frc2026.util.Repulsor.Offload;

/**
 * Explicit execution context for Repulsor offload workers.
 *
 * <p>Older code inferred offload execution from the worker thread name. That is fragile because a
 * server implementation detail can change planner behavior. Entrypoints should wrap work in {@link
 * #runWorker(ThrowingSupplier)} and runtime code should call {@link #isWorker()}.
 */
public final class OffloadExecutionContext {
  private static final ThreadLocal<Boolean> WORKER = ThreadLocal.withInitial(() -> Boolean.FALSE);

  private OffloadExecutionContext() {}

  /** Returns true when the current call stack is executing inside an offload worker entrypoint. */
  public static boolean isWorker() {
    return WORKER.get().booleanValue();
  }

  /**
   * Compatibility fallback for generated/legacy entrypoints that have not yet been wrapped. Prefer
   * {@link #isWorker()} for behavior decisions.
   */
  public static boolean isWorkerOrLegacyThread() {
    return isWorker() || Thread.currentThread().getName().startsWith("offload-server-worker");
  }

  /** Runs a void action with the worker marker set for this thread. */
  public static void runWorker(ThrowingRunnable action) {
    boolean previous = isWorker();
    WORKER.set(Boolean.TRUE);
    try {
      action.run();
    } finally {
      WORKER.set(previous);
    }
  }

  /** Runs a value-producing action with the worker marker set for this thread. */
  public static <T> T runWorker(ThrowingSupplier<T> action) {
    boolean previous = isWorker();
    WORKER.set(Boolean.TRUE);
    try {
      return action.get();
    } finally {
      WORKER.set(previous);
    }
  }

  @FunctionalInterface
  public interface ThrowingSupplier<T> {
    T get();
  }

  @FunctionalInterface
  public interface ThrowingRunnable {
    void run();
  }
}
