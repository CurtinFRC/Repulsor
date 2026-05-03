package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import java.util.HashMap;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.Track;

/** Owns tracked robot intent maps for predictive collection scoring. */
public final class PredictiveTrackStore {
  private final HashMap<Integer, Track> allies = new HashMap<>();
  private final HashMap<Integer, Track> enemies = new HashMap<>();

  public HashMap<Integer, Track> allies() {
    return allies;
  }

  public HashMap<Integer, Track> enemies() {
    return enemies;
  }

  public void clear() {
    allies.clear();
    enemies.clear();
  }
}
