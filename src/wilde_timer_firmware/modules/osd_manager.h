#pragma once

#include <Arduino.h>

bool osdIsElementEnabled(uint8_t col, uint8_t row);
bool osdIsElementVisibleNow(uint8_t col, uint8_t row, bool showDuringRace, bool raceInProgress);
size_t osdBuildVisibleText(uint8_t col, uint8_t maxColInclusive, const char *text, char *out, size_t outSize);

using OsdSendTextFn = bool (*)(uint8_t row, uint8_t col, const char *text);

struct OsdRaceLapsRenderCache {
  uint8_t prevRenderedCount = 0;
  uint8_t prevRow = 0;
  uint8_t prevCol = 0;
};

bool osdSendRaceLapsColumn(const uint32_t *lapsMs, uint8_t lapCount, bool allowRender,
                           uint8_t row, uint8_t col, uint8_t maxRow,
                           OsdRaceLapsRenderCache &cache, OsdSendTextFn sendTextFn);

bool osdSendComposedStatus(
    bool &forceFullRefresh,
    uint8_t maxColInclusive,
    const char *channelText,
    const char *rssiText,
    const char *lapPeakRssiText,
    const char *rssiThrUpperText,
    const char *rssiThrLowerText,
    const char *bestLapText,
    const char *bestLapRaceText,
    const char *best3Text,
    const char *best3RaceText,
    const char *lapPopupText,
    const char *waitVtxAdminText,
    uint8_t channelRow, uint8_t channelCol, bool channelShowDuringRace,
    uint8_t rssiRow, uint8_t rssiCol, bool rssiShowDuringRace,
    uint8_t lapPeakRow, uint8_t lapPeakCol, bool lapPeakShowDuringRace,
    uint8_t thrUpperRow, uint8_t thrUpperCol, bool thrUpperShowDuringRace,
    uint8_t thrLowerRow, uint8_t thrLowerCol, bool thrLowerShowDuringRace,
    uint8_t bestLapRow, uint8_t bestLapCol, bool bestLapShowDuringRace,
    uint8_t bestLapRaceRow, uint8_t bestLapRaceCol, bool bestLapRaceShowDuringRace,
    uint8_t best3Row, uint8_t best3Col, bool best3ShowDuringRace,
    uint8_t best3RaceRow, uint8_t best3RaceCol, bool best3RaceShowDuringRace,
    uint8_t lapPopupRow, uint8_t lapPopupCol, bool lapPopupShowDuringRace,
    uint8_t waitRow, uint8_t waitCol,
    bool raceInProgress,
    OsdSendTextFn sendTextFn);
