#pragma once

#include <Arduino.h>
#include <esp_now.h>

uint8_t crsfCrc8DvbS2(uint8_t crc, uint8_t data);

// Builds MSPv2 frame in `out`, returns bytes written or 0 on error.
size_t crsfBuildMspV2Command(uint16_t function, const uint8_t *payload, uint16_t payloadSize,
                             uint8_t *out, size_t outMax, size_t maxPayload);

uint16_t crsfRawToUs(uint16_t raw);
bool crsfIsSupportedAddress(uint8_t b);
bool crsfTryMapVtxAdminCodeToChannelIndex(uint8_t code, uint8_t &index);
uint8_t crsfChannelIndexToVtxAdminCode(uint8_t index);
uint8_t crsfChannelIndexToMspTableIndex48(uint8_t index);
bool crsfTryExtractVtxAdminRaceChannelCode(const uint8_t *payload, uint8_t payloadLen, uint8_t &outCode,
                                           uint8_t timerChannelCount);

bool espnowSendWithRetry(const uint8_t *destMac,
                         const uint8_t *packet,
                         size_t packetLen,
                         uint16_t function,
                         uint16_t payloadSize,
                         uint8_t maxAttempts,
                         unsigned long minSpacingMs,
                         unsigned long errLogPeriodMs);
