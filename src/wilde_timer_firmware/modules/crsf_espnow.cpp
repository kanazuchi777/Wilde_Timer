#include "crsf_espnow.h"

#include <cstdio>

uint8_t crsfCrc8DvbS2(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; ++i) {
    if (crc & 0x80) {
      crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
    } else {
      crc <<= 1;
    }
  }
  return crc;
}

size_t crsfBuildMspV2Command(uint16_t function, const uint8_t *payload, uint16_t payloadSize,
                             uint8_t *out, size_t outMax, size_t maxPayload) {
  const size_t needed = 3 + 1 + 2 + 2 + payloadSize + 1;
  if (outMax < needed || payloadSize > maxPayload) {
    return 0;
  }

  size_t p = 0;
  out[p++] = '$';
  out[p++] = 'X';
  out[p++] = '<';

  out[p++] = 0;  // flags
  out[p++] = static_cast<uint8_t>(function & 0xFF);
  out[p++] = static_cast<uint8_t>((function >> 8) & 0xFF);
  out[p++] = static_cast<uint8_t>(payloadSize & 0xFF);
  out[p++] = static_cast<uint8_t>((payloadSize >> 8) & 0xFF);

  uint8_t crc = 0;
  for (size_t i = 3; i < p; ++i) {
    crc = crsfCrc8DvbS2(crc, out[i]);
  }

  for (uint16_t i = 0; i < payloadSize; ++i) {
    out[p++] = payload[i];
    crc = crsfCrc8DvbS2(crc, payload[i]);
  }

  out[p++] = crc;
  return p;
}

uint16_t crsfRawToUs(uint16_t raw) {
  // ELRS/CRSF nominal raw range is about 172..1811 -> ~988..2012us.
  if (raw < 172) {
    raw = 172;
  } else if (raw > 1811) {
    raw = 1811;
  }
  return static_cast<uint16_t>(988U + ((static_cast<uint32_t>(raw - 172U) * 1024U) / 1639U));
}

bool crsfIsSupportedAddress(uint8_t b) {
  switch (b) {
    case 0x00:  // broadcast
    case 0xCE:  // VTX
    case 0xC8:  // flight controller
    case 0xEA:  // radio transmitter
    case 0xEC:  // receiver
    case 0xEE:  // transmitter module / radio side
    case 0xEF:  // ELRS Lua (non-standard, used by ExpressLRS tooling)
      return true;
    default:
      return false;
  }
}

bool crsfTryMapVtxAdminCodeToChannelIndex(uint8_t code, uint8_t &index) {
  // Raceband R1..R8
  if (code >= 0x20 && code <= 0x27) {
    index = static_cast<uint8_t>(code - 0x20);
    return true;
  }
  // Fatshark band F1..F8
  if (code >= 0x18 && code <= 0x1F) {
    index = static_cast<uint8_t>(8U + (code - 0x18));
    return true;
  }
  return false;
}

uint8_t crsfChannelIndexToVtxAdminCode(uint8_t index) {
  if (index < 8) {
    return static_cast<uint8_t>(0x20U + index);  // R1..R8
  }
  return static_cast<uint8_t>(0x18U + (index - 8U));  // F1..F8
}

uint8_t crsfChannelIndexToMspTableIndex48(uint8_t index) {
  if (index < 8) {
    return static_cast<uint8_t>(32U + index);  // R1..R8
  }
  return static_cast<uint8_t>(24U + (index - 8U));  // F1..F8
}

bool crsfTryExtractVtxAdminRaceChannelCode(const uint8_t *payload, uint8_t payloadLen, uint8_t &outCode,
                                           uint8_t timerChannelCount) {
  if (!payload || timerChannelCount == 0) {
    return false;
  }

  auto normalizeVtxRaceChannelCode = [](uint8_t raw, uint8_t &normalizedCode) -> bool {
    uint8_t idx = 0;
    if (crsfTryMapVtxAdminCodeToChannelIndex(raw, idx)) {
      normalizedCode = raw;
      return true;
    }
    return false;
  };

  if (payloadLen >= 6) {
    uint8_t ch = 0;
    if (normalizeVtxRaceChannelCode(payload[5], ch)) {
      outCode = ch;
      return true;
    }
  }
  if (payloadLen >= 5) {
    uint8_t ch = 0;
    if (normalizeVtxRaceChannelCode(payload[4], ch)) {
      outCode = ch;
      return true;
    }
  }
  if (payloadLen >= 4) {
    uint8_t ch = 0;
    if (normalizeVtxRaceChannelCode(payload[3], ch)) {
      outCode = ch;
      return true;
    }
  }

  bool seen[16] = {};
  uint8_t last = 0;
  uint8_t uniqueCount = 0;
  for (uint8_t i = 0; i < payloadLen; ++i) {
    uint8_t ch = 0;
    if (normalizeVtxRaceChannelCode(payload[i], ch)) {
      uint8_t idx = 0;
      if (!crsfTryMapVtxAdminCodeToChannelIndex(ch, idx) || idx >= timerChannelCount || idx >= 16U) {
        continue;
      }
      if (!seen[idx]) {
        seen[idx] = true;
        last = ch;
        ++uniqueCount;
      }
    }
  }
  if (uniqueCount == 1) {
    outCode = last;
    return true;
  }
  return false;
}

bool espnowSendWithRetry(const uint8_t *destMac,
                         const uint8_t *packet,
                         size_t packetLen,
                         uint16_t function,
                         uint16_t payloadSize,
                         uint8_t maxAttempts,
                         unsigned long minSpacingMs,
                         unsigned long errLogPeriodMs) {
  if (!destMac || !packet || packetLen == 0 || maxAttempts == 0) {
    return false;
  }

  static unsigned long sLastEspNowSendMs = 0;
  static unsigned long sLastEspNowErrLogMs = 0;
  const unsigned long now = millis();
  const unsigned long elapsed = now - sLastEspNowSendMs;
  if (elapsed < minSpacingMs) {
    delay(minSpacingMs - elapsed);
  }

  for (uint8_t attempt = 0; attempt < maxAttempts; ++attempt) {
    const esp_err_t err = esp_now_send(destMac, packet, packetLen);
    if (err == ESP_OK) {
      sLastEspNowSendMs = millis();
      return true;
    }

    bool retryable = false;
#if defined(ESP_ERR_ESPNOW_NO_MEM)
    if (err == ESP_ERR_ESPNOW_NO_MEM) {
      retryable = true;
    }
#endif
#if defined(ESP_ERR_ESPNOW_FULL)
    if (err == ESP_ERR_ESPNOW_FULL) {
      retryable = true;
    }
#endif

    if (retryable && (attempt + 1U) < maxAttempts) {
      delay(static_cast<unsigned long>(2U + attempt * 2U));
      continue;
    }

    const unsigned long logNow = millis();
    if (logNow - sLastEspNowErrLogMs >= errLogPeriodMs) {
      sLastEspNowErrLogMs = logNow;
      Serial.printf("esp_now_send failed: err=%d fn=0x%04X len=%u\n",
                    static_cast<int>(err),
                    static_cast<unsigned>(function),
                    static_cast<unsigned>(payloadSize));
    }
    return false;
  }
  return false;
}
