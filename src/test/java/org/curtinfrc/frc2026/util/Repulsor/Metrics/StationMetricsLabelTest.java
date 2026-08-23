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

package org.curtinfrc.frc2026.util.Repulsor.Metrics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import org.curtinfrc.frc2026.util.Repulsor.Metrics.HPStationMetrics.Labels;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class StationMetricsLabelTest {
  @AfterEach
  void restoreLabels() {
    HPStationMetrics.resetToDefaults();
  }

  @Test
  void defaultLabelsPreserveHistoricalTopicLayout() {
    assertEquals(Labels.defaults(), HPStationMetrics.labels());
    MetricRecorder<Double> left = HPStationMetrics.recorder("Left");
    assertEquals("hp/Left/pickupTimeSeconds", left.getName());
    MetricRecorder<Double> right = HPStationMetrics.recorder("Right");
    assertEquals("hp/Right/pickupTimeSeconds", right.getName());
  }

  @Test
  void injectedLabelsRouteTopicsUnderCustomRoot() {
    HPStationMetrics.configure(new Labels("station", "grabSeconds"));
    MetricRecorder<Double> left = HPStationMetrics.recorder("Left");
    assertEquals("station/Left/grabSeconds", left.getName());
    MetricRecorder<Double> right = HPStationMetrics.recorder("Right");
    assertEquals("station/Right/grabSeconds", right.getName());
  }

  @Test
  void resetToDefaultsRestoresOriginalRootWithoutCallSiteChanges() {
    HPStationMetrics.configure(new Labels("station", "grabSeconds"));
    HPStationMetrics.recorder("Left");
    HPStationMetrics.resetToDefaults();
    assertEquals("hp/Left/pickupTimeSeconds", HPStationMetrics.recorder("Left").getName());
  }

  @Test
  void sameKeyAndStableLabelsReuseRecorderInstance() {
    assertSame(HPStationMetrics.recorder("Center"), HPStationMetrics.recorder("Center"));
  }

  @Test
  void blankOrNullLabelComponentsFallBackToSeasonDefaults() {
    HPStationMetrics.configure(new Labels(null, "  "));
    assertEquals("hp/Mid/pickupTimeSeconds", HPStationMetrics.recorder("Mid").getName());
  }
}
