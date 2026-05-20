#pragma once

#include <Arduino.h>

// Computes average of fastest topPercent laps from values[0..count-1].
// Uses scratch buffer of at least count elements.
uint32_t timingComputeBestHalfAverage(const uint32_t *values, uint32_t *scratch, uint16_t count, uint8_t topPercent);

// Returns adaptive fake-lap floor derived from median.
uint32_t timingComputeFakeLapFloor(uint32_t medianMs, uint32_t outlierFastAbsMs, uint8_t outlierFastRatioPct);

struct TimingOutlierHistory {
  uint32_t *lapMs = nullptr;
  uint8_t *writeIndex = nullptr;
  uint8_t *count = nullptr;
  uint8_t capacity = 0;
};

// Appends `lapMs` to ring history and returns true if lap looks like an outlier.
bool timingDetectAndStoreFakeLap(TimingOutlierHistory history, uint32_t lapMs, uint8_t historyDepth,
                                 uint32_t outlierFastAbsMs, uint8_t outlierFastRatioPct);
