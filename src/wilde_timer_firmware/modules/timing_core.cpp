#include "timing_core.h"

namespace {
uint32_t timingComputeMedianMs(uint32_t *vals, uint8_t count) {
  if (!vals || count == 0) {
    return 0;
  }
  for (uint8_t i = 1; i < count; ++i) {
    const uint32_t key = vals[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && vals[j] > key) {
      vals[j + 1] = vals[j];
      --j;
    }
    vals[j + 1] = key;
  }
  if ((count & 1U) != 0U) {
    return vals[count / 2U];
  }
  const uint32_t a = vals[(count / 2U) - 1U];
  const uint32_t b = vals[count / 2U];
  return static_cast<uint32_t>((static_cast<uint64_t>(a) + static_cast<uint64_t>(b)) / 2ULL);
}
}  // namespace

uint32_t timingComputeBestHalfAverage(const uint32_t *values, uint32_t *scratch, uint16_t count, uint8_t topPercent) {
  if (!values || !scratch || count == 0) {
    return 0;
  }

  for (uint16_t i = 0; i < count; ++i) {
    scratch[i] = values[i];
  }

  // In-place insertion sort: efficient enough for small rolling windows.
  for (uint16_t i = 1; i < count; ++i) {
    uint32_t key = scratch[i];
    int32_t j = static_cast<int32_t>(i) - 1;
    while (j >= 0 && scratch[j] > key) {
      scratch[j + 1] = scratch[j];
      j--;
    }
    scratch[j + 1] = key;
  }

  uint16_t bestCount = static_cast<uint16_t>((static_cast<uint32_t>(count) * static_cast<uint32_t>(topPercent)) / 100U);
  if (bestCount == 0) {
    bestCount = 1;
  }

  uint64_t sum = 0;
  for (uint16_t i = 0; i < bestCount; ++i) {
    sum += static_cast<uint64_t>(scratch[i]);
  }
  return static_cast<uint32_t>(sum / bestCount);
}

uint32_t timingComputeFakeLapFloor(uint32_t medianMs, uint32_t outlierFastAbsMs, uint8_t outlierFastRatioPct) {
  const uint32_t absFloor = (medianMs > outlierFastAbsMs) ? (medianMs - outlierFastAbsMs) : 0U;
  const uint32_t ratioFloor = static_cast<uint32_t>((static_cast<uint64_t>(medianMs) * outlierFastRatioPct) / 100ULL);
  return (absFloor > ratioFloor) ? absFloor : ratioFloor;
}

bool timingDetectAndStoreFakeLap(TimingOutlierHistory history, uint32_t lapMs, uint8_t historyDepth,
                                 uint32_t outlierFastAbsMs, uint8_t outlierFastRatioPct) {
  if (!history.lapMs || !history.writeIndex || !history.count || history.capacity == 0) {
    return false;
  }

  if (historyDepth > history.capacity) {
    historyDepth = history.capacity;
  }

  uint32_t recent[64] = {};
  uint8_t count = 0;
  for (uint8_t i = 0; i < *history.count && count < historyDepth; ++i) {
    const uint16_t idx16 =
        (static_cast<uint16_t>(*history.writeIndex) + static_cast<uint16_t>(history.capacity) - 1U - i) %
        static_cast<uint16_t>(history.capacity);
    const uint8_t idx = static_cast<uint8_t>(idx16);
    const uint32_t ms = history.lapMs[idx];
    if (ms == 0) {
      continue;
    }
    recent[count++] = ms;
  }

  bool isFake = false;
  if (count >= 4U) {
    const uint32_t medianMs = timingComputeMedianMs(recent, count);
    const uint32_t floorMs = timingComputeFakeLapFloor(medianMs, outlierFastAbsMs, outlierFastRatioPct);
    isFake = lapMs < floorMs;
  }

  history.lapMs[*history.writeIndex] = lapMs;
  *history.writeIndex = static_cast<uint8_t>((*history.writeIndex + 1U) % history.capacity);
  if (*history.count < history.capacity) {
    (*history.count)++;
  }

  return isFake;
}
