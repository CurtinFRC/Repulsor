package org.curtinfrc.frc2026.util.Repulsor.Offload;

@SuppressWarnings("unused")
/**
 * Provides sample math offload entrypoints functionality for the Repulsor offload serialization and
 * native/JNI entrypoint boundary. Use this type from robot code, field profiles, or tests when
 * integrating the corresponding Repulsor subsystem. Coordinates are field-relative unless a method
 * documents robot-relative motion.
 */
public final class SampleMathOffloadEntrypoints {
  private SampleMathOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.SAMPLE_DOUBLE_VALUE,
      version = 1,
      timeoutMs = 250,
      fallback = true)
  /**
   * Returns the double value value maintained by this Repulsor component.
   *
   * @param input value used by this operation.
   * @return value produced by this operation.
   */
  public static int doubleValue(int input) {
    return input * 2;
  }

  @Offloadable(
      id = OffloadTaskIds.SAMPLE_WORKER_THREAD_PROBE,
      version = 1,
      timeoutMs = 250,
      fallback = true)
  /**
   * Returns the runs on offload worker thread value maintained by this Repulsor component.
   *
   * @param marker value used by this operation.
   * @return value produced by this operation.
   */
  public static boolean runsOnOffloadWorkerThread(int marker) {
    return marker == 1 && Thread.currentThread().getName().startsWith("offload-server-worker");
  }
}
