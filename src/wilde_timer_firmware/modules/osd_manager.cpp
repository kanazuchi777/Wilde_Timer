#include "osd_manager.h"

#include <cstring>
#include <cstdio>

bool osdIsElementEnabled(uint8_t col, uint8_t row) {
  return !(col == 0 && row == 0);
}

bool osdIsElementVisibleNow(uint8_t col, uint8_t row, bool showDuringRace, bool raceInProgress) {
  if (!osdIsElementEnabled(col, row)) {
    return false;
  }
  if (raceInProgress && !showDuringRace) {
    return false;
  }
  return true;
}

size_t osdBuildVisibleText(uint8_t col, uint8_t maxColInclusive, const char *text, char *out, size_t outSize) {
  if (!out || outSize == 0) {
    return 0;
  }
  out[0] = '\0';
  if (!text || text[0] == '\0') {
    return 0;
  }

  const uint8_t canvasCols = static_cast<uint8_t>(maxColInclusive + 1U);
  if (col >= canvasCols) {
    return 0;
  }
  const size_t maxVisible = static_cast<size_t>(canvasCols - col);
  size_t n = strlen(text);
  if (n > maxVisible) {
    n = maxVisible;
  }
  if (n > outSize - 1U) {
    n = outSize - 1U;
  }
  if (n == 0) {
    return 0;
  }
  memcpy(out, text, n);
  out[n] = '\0';
  return n;
}

bool osdSendRaceLapsColumn(const uint32_t *lapsMs, uint8_t lapCount, bool allowRender,
                           uint8_t row, uint8_t col, uint8_t maxRow,
                           OsdRaceLapsRenderCache &cache, OsdSendTextFn sendTextFn) {
  if (!sendTextFn) {
    return false;
  }

  bool changed = false;
  auto clearPrevRows = [&]() {
    for (uint8_t i = 0; i < cache.prevRenderedCount; ++i) {
      sendTextFn(static_cast<uint8_t>(cache.prevRow + i), cache.prevCol, "                ");
      changed = true;
    }
    cache.prevRenderedCount = 0;
    cache.prevRow = 0;
    cache.prevCol = 0;
  };

  const bool enabled = allowRender && osdIsElementEnabled(col, row);
  if (!enabled) {
    if (cache.prevRenderedCount > 0) {
      clearPrevRows();
    }
    return changed;
  }

  if (row > maxRow) {
    if (cache.prevRenderedCount > 0) {
      clearPrevRows();
    }
    return changed;
  }

  const uint8_t visibleRows = static_cast<uint8_t>(maxRow - row + 1U);
  const uint8_t renderCount = static_cast<uint8_t>(min(static_cast<unsigned>(lapCount),
                                                        static_cast<unsigned>(visibleRows)));
  const uint8_t startIdx = (lapCount > renderCount) ? static_cast<uint8_t>(lapCount - renderCount) : 0;
  const bool layoutChanged = (row != cache.prevRow) || (col != cache.prevCol);

  if (layoutChanged && cache.prevRenderedCount > 0) {
    clearPrevRows();
  }

  for (uint8_t i = 0; i < renderCount; ++i) {
    const uint8_t lapIdx = static_cast<uint8_t>(startIdx + i);
    const uint32_t lapMs = lapsMs ? lapsMs[lapIdx] : 0U;
    const uint32_t sec = lapMs / 1000U;
    const uint32_t cs = (lapMs % 1000U) / 10U;
    char line[24] = {};
    snprintf(line, sizeof(line), "L%u:%02lu.%02lu", static_cast<unsigned>(lapIdx + 1U),
             static_cast<unsigned long>(sec), static_cast<unsigned long>(cs));
    sendTextFn(static_cast<uint8_t>(row + i), col, line);
    changed = true;
  }

  if (!layoutChanged && cache.prevRenderedCount > renderCount) {
    for (uint8_t i = renderCount; i < cache.prevRenderedCount; ++i) {
      sendTextFn(static_cast<uint8_t>(row + i), col, "                ");
      changed = true;
    }
  }

  cache.prevRenderedCount = renderCount;
  cache.prevRow = row;
  cache.prevCol = col;
  return changed;
}

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
    OsdSendTextFn sendTextFn) {
  if (!sendTextFn) {
    return false;
  }

  static bool sLapPopupWasVisible = false;
  static bool sWaitVtxWasVisible = false;
  static char sLastChannel[24] = {};
  static char sLastRssi[24] = {};
  static char sLastLapPeakRssi[24] = {};
  static char sLastThrUpper[24] = {};
  static char sLastThrLower[24] = {};
  static char sLastBestLap[24] = {};
  static char sLastBestLapRace[24] = {};
  static char sLastBest3[24] = {};
  static char sLastBest3Race[24] = {};
  static char sLastLapPopup[32] = {};
  static char sLastWaitVtx[24] = {};
  static unsigned long sLastDisplayMs = 0;

  if (forceFullRefresh) {
    memset(sLastChannel, 0, sizeof(sLastChannel));
    memset(sLastRssi, 0, sizeof(sLastRssi));
    memset(sLastLapPeakRssi, 0, sizeof(sLastLapPeakRssi));
    memset(sLastThrUpper, 0, sizeof(sLastThrUpper));
    memset(sLastThrLower, 0, sizeof(sLastThrLower));
    memset(sLastBestLap, 0, sizeof(sLastBestLap));
    memset(sLastBestLapRace, 0, sizeof(sLastBestLapRace));
    memset(sLastBest3, 0, sizeof(sLastBest3));
    memset(sLastBest3Race, 0, sizeof(sLastBest3Race));
    memset(sLastLapPopup, 0, sizeof(sLastLapPopup));
    memset(sLastWaitVtx, 0, sizeof(sLastWaitVtx));
    sLapPopupWasVisible = true;
    sWaitVtxWasVisible = true;
    forceFullRefresh = false;
  }

  auto sendField = [&](uint8_t row, uint8_t col, const char *text, char *cache, size_t cacheSize) -> bool {
    char visible[32] = {};
    const size_t newLen = osdBuildVisibleText(col, maxColInclusive, text, visible, sizeof(visible));
    if (newLen == 0) {
      return false;
    }
    const size_t prevLen = strlen(cache);
    if (!sendTextFn(row, col, visible)) {
      return false;
    }

    if (prevLen > newLen) {
      const uint8_t clearCol = static_cast<uint8_t>(col + newLen);
      char spaces[24] = {};
      size_t tail = prevLen - newLen;
      if (tail > sizeof(spaces) - 1U) {
        tail = sizeof(spaces) - 1U;
      }
      for (size_t i = 0; i < tail; ++i) {
        spaces[i] = ' ';
      }
      spaces[tail] = '\0';
      char visibleSpaces[24] = {};
      if (osdBuildVisibleText(clearCol, maxColInclusive, spaces, visibleSpaces, sizeof(visibleSpaces)) > 0) {
        sendTextFn(row, clearCol, visibleSpaces);
      }
    }

    snprintf(cache, cacheSize, "%s", visible);
    return true;
  };

  struct OsdField {
    uint8_t row;
    uint8_t col;
    const char *text;
    char *cache;
    size_t cacheSize;
    bool enabled;
  };

  OsdField fields[] = {
      {channelRow, channelCol, channelText, sLastChannel, sizeof(sLastChannel),
       osdIsElementVisibleNow(channelCol, channelRow, channelShowDuringRace, raceInProgress)},
      {rssiRow, rssiCol, rssiText, sLastRssi, sizeof(sLastRssi),
       osdIsElementVisibleNow(rssiCol, rssiRow, rssiShowDuringRace, raceInProgress)},
      {lapPeakRow, lapPeakCol, lapPeakRssiText, sLastLapPeakRssi, sizeof(sLastLapPeakRssi),
       osdIsElementVisibleNow(lapPeakCol, lapPeakRow, lapPeakShowDuringRace, raceInProgress)},
      {thrUpperRow, thrUpperCol, rssiThrUpperText, sLastThrUpper, sizeof(sLastThrUpper),
       osdIsElementVisibleNow(thrUpperCol, thrUpperRow, thrUpperShowDuringRace, raceInProgress)},
      {thrLowerRow, thrLowerCol, rssiThrLowerText, sLastThrLower, sizeof(sLastThrLower),
       osdIsElementVisibleNow(thrLowerCol, thrLowerRow, thrLowerShowDuringRace, raceInProgress)},
      {bestLapRow, bestLapCol, bestLapText, sLastBestLap, sizeof(sLastBestLap),
       osdIsElementVisibleNow(bestLapCol, bestLapRow, bestLapShowDuringRace, raceInProgress)},
      {bestLapRaceRow, bestLapRaceCol, bestLapRaceText, sLastBestLapRace, sizeof(sLastBestLapRace),
       osdIsElementVisibleNow(bestLapRaceCol, bestLapRaceRow, bestLapRaceShowDuringRace, raceInProgress)},
      {best3Row, best3Col, best3Text, sLastBest3, sizeof(sLastBest3),
       osdIsElementVisibleNow(best3Col, best3Row, best3ShowDuringRace, raceInProgress)},
      {best3RaceRow, best3RaceCol, best3RaceText, sLastBest3Race, sizeof(sLastBest3Race),
       osdIsElementVisibleNow(best3RaceCol, best3RaceRow, best3RaceShowDuringRace, raceInProgress)},
  };

  bool sentAnyText = false;
  const uint8_t fieldCount = static_cast<uint8_t>(sizeof(fields) / sizeof(fields[0]));
  for (uint8_t idx = 0; idx < fieldCount; ++idx) {
    const OsdField &f = fields[idx];
    if (!f.enabled) {
      if (f.cache[0] != '\0') {
        sendTextFn(f.row, f.col, "                ");
        f.cache[0] = '\0';
        sentAnyText = true;
      }
      continue;
    }
    if (sendField(f.row, f.col, f.text, f.cache, f.cacheSize)) {
      sentAnyText = true;
    }
  }

  bool popupChanged = false;
  const bool lapPopupEnabled = osdIsElementVisibleNow(lapPopupCol, lapPopupRow, lapPopupShowDuringRace, raceInProgress);
  if (lapPopupEnabled && lapPopupText && lapPopupText[0] != '\0') {
    if (sendTextFn(lapPopupRow, lapPopupCol, lapPopupText)) {
      snprintf(sLastLapPopup, sizeof(sLastLapPopup), "%s", lapPopupText);
      popupChanged = true;
    }
    sLapPopupWasVisible = true;
  } else if (sLapPopupWasVisible) {
    if (sendTextFn(lapPopupRow, lapPopupCol, "                ")) {
      sLapPopupWasVisible = false;
      sLastLapPopup[0] = '\0';
      popupChanged = true;
    }
  }

  bool waitChanged = false;
  if (waitVtxAdminText && waitVtxAdminText[0] != '\0') {
    if (sendTextFn(waitRow, waitCol, waitVtxAdminText)) {
      snprintf(sLastWaitVtx, sizeof(sLastWaitVtx), "%s", waitVtxAdminText);
      waitChanged = true;
    }
    sWaitVtxWasVisible = true;
  } else if (sWaitVtxWasVisible) {
    if (sendTextFn(waitRow, waitCol, "                ")) {
      sWaitVtxWasVisible = false;
      sLastWaitVtx[0] = '\0';
      waitChanged = true;
    }
  }

  const unsigned long now = millis();
  const bool heartbeatDisplay = (now - sLastDisplayMs) >= 1500UL;
  if (sentAnyText || popupChanged || waitChanged || heartbeatDisplay) {
    sLastDisplayMs = now;
    return true;
  }
  return false;
}
