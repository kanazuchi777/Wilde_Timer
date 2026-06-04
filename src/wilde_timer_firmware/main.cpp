#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <SD.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>
#include <cctype>
#include <cstdlib>
#if defined(CONFIG_IDF_TARGET_ESP32S3) && SOC_USB_OTG_SUPPORTED && !ARDUINO_USB_MODE
#include <USB.h>
#include <USBMSC.h>
#endif
#include "modules/parse_utils.h"
#include "modules/osd_manager.h"
#include "modules/timing_core.h"
#include "modules/crsf_espnow.h"

// ELRS RX sniff (diagnostics only)
static const int kElrsRxPin = 16;
static const int kElrsTxPin = 17;  // optional, kept for full UART init
static const uint32_t kElrsBaud = 420000;

// ExpressLRS Backpack commands
static const uint16_t MSP_ELRS_BIND = 0x0009;
static const uint16_t MSP_ELRS_SET_OSD = 0x00B6;
static const uint16_t MSP_SET_VTX_CONFIG = 0x0059;

// Backpack addressing
static uint8_t kBackpackUid[6] = {};
static uint8_t kResolvedUidMac[6] = {};
static const uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Board pins
// ESP32-S3 mapping for this project wiring.
// RX5808: RSSI=GPIO4, DATA/CH1=GPIO10, SEL/CH2=GPIO11, CLK/CH3=GPIO12
// microSD: CS=GPIO39, SCK=GPIO36, MOSI=GPIO35, MISO=GPIO37
#if defined(CONFIG_IDF_TARGET_ESP32S3)
static const int kRssiInputPin = 4;
static const int kRx5808DataPin = 10;
static const int kRx5808ClkPin = 12;
static const int kRx5808SelPin = 11;
static const int kSdSpiSckPin = 36;
static const int kSdSpiMisoPin = 37;
static const int kSdSpiMosiPin = 35;
static const int kSdSpiCsPin = 39;
#else
// Default mapping for ESP32 DevKit V1 (adjust to your wiring if needed):
// RSSI -> GPIO34 (ADC input-only), DATA -> GPIO23, CLK -> GPIO18, SEL -> GPIO5
static const int kRssiInputPin = 34;
static const int kRx5808DataPin = 23;
static const int kRx5808ClkPin = 18;
static const int kRx5808SelPin = 5;
// microSD SPI defaults (shares SCK/MOSI with RX5808 to avoid extra wires)
static const int kSdSpiSckPin = 25;
static const int kSdSpiMisoPin = 27;
static const int kSdSpiMosiPin = 26;
static const int kSdSpiCsPin = 33;
#endif
static const uint32_t kSdSpiHz = 1000000UL;
static const char *kSdConfigPath = "/config.txt";
static const char *kSdLogsDirPath = "/LOGS";
static const char *kSdLapsPath = "/LOGS/laps.csv";
#if defined(CONFIG_IDF_TARGET_ESP32S3) && SOC_USB_OTG_SUPPORTED && !ARDUINO_USB_MODE
static USBMSC gUsbMsc;
static bool gUsbMscEnabled = false;
static bool gUsbMscHostActive = false;
#endif

// RX5808 timing
static const unsigned long kRx5808MinTuneMs = 35;
static const unsigned long kRx5808MinBusMs = 30;
static unsigned long gLastRx5808BusMs = 0;
static bool gRecentFreqChange = false;
static unsigned long gFreqChangeMs = 0;

// Timer channels
struct RaceChannel {
  const char *name;
  uint16_t freqMhz;
};
static const uint8_t kTimerChannelCount = 16;
// Keep R1..R8 first to preserve existing AUX mapping (0..7 -> R1..R8).
static const RaceChannel kTimerChannels[kTimerChannelCount] = {
    {"R1", 5658}, {"R2", 5695}, {"R3", 5732}, {"R4", 5769},
    {"R5", 5806}, {"R6", 5843}, {"R7", 5880}, {"R8", 5917},
    {"F1", 5740}, {"F2", 5760}, {"F3", 5780}, {"F4", 5800},
    {"F5", 5820}, {"F6", 5840}, {"F7", 5860}, {"F8", 5880},
};

// Scan/lap settings
// ============================ USER SETTINGS ============================
// Edit only this block for day-to-day tuning.
static uint8_t gCfgOsdMainRow = 17;                       // OSD row position
static uint8_t gCfgOsdMainCol = 13;                       // OSD column position
static uint8_t gCfgLapPopupRow = 12;                  // LAST lap popup row position
static uint8_t gCfgLapPopupCol = 20;                  // LAST lap popup column position
static constexpr uint8_t kOsdWaitVtxAdminRow = 13;
static constexpr uint8_t kOsdWaitVtxAdminCol = 17;
static uint8_t gCfgOsdChannelRow = 16;
static uint8_t gCfgOsdChannelCol = 9;
static bool gCfgOsdChannelShowDuringRace = true;
static uint8_t gCfgOsdRssiRow = 16;
static uint8_t gCfgOsdRssiCol = 12;
static bool gCfgOsdRssiShowDuringRace = true;
// Last lap gate-peak RSSI shown right after current RSSI by default.
static uint8_t gCfgOsdLapPeakRssiRow = 16;
static uint8_t gCfgOsdLapPeakRssiCol = 18;
static bool gCfgOsdLapPeakRssiShowDuringRace = true;
// Upper/lower RSSI thresholds (ENTER/EXIT) shown after current + lap-peak RSSI by default.
static uint8_t gCfgOsdRssiThrUpperRow = 16;
static uint8_t gCfgOsdRssiThrUpperCol = 25;
static bool gCfgOsdRssiThrUpperShowDuringRace = true;
static uint8_t gCfgOsdRssiThrLowerRow = 16;
static uint8_t gCfgOsdRssiThrLowerCol = 32;
static bool gCfgOsdRssiThrLowerShowDuringRace = true;
static uint8_t gCfgOsdBestLapRow = 17;
static uint8_t gCfgOsdBestLapCol = 9;
static bool gCfgOsdBestLapShowDuringRace = true;
static uint8_t gCfgOsdBestLapRaceRow = 17;
static uint8_t gCfgOsdBestLapRaceCol = 21;
static bool gCfgOsdBestLapRaceShowDuringRace = true;
static uint8_t gCfgOsdBest3Row = 17;
static uint8_t gCfgOsdBest3Col = 33;
static bool gCfgOsdBest3ShowDuringRace = true;
static uint8_t gCfgOsdBest3RaceRow = 0;
static uint8_t gCfgOsdBest3RaceCol = 42;
static bool gCfgOsdBest3RaceShowDuringRace = true;
static uint8_t gCfgOsdRaceLapsRow = 1;
static uint8_t gCfgOsdRaceLapsCol = 42;
static bool gCfgOsdRaceLapsShowDuringRace = true;
static bool gCfgLapPopupShowDuringRace = true;
static uint8_t gCfgLockThresholdRssi = 100;                     // RSSI lock threshold for scan-to-lock gating paths
static int8_t gCfgEnterOffsetRssi = -25;                      // Enter offset from lock RSSI (e.g. -25)
static int8_t gCfgExitOffsetRssi = -40;                     // Exit offset from lock RSSI (e.g. -40)
static unsigned long gCfgMinLapIntervalMs = 8000;    // Minimum time between laps
static unsigned long gCfgPostLockIgnoreMs = 6000;   // Legacy config key (currently not used by runtime logic)
// RX5808 mode: 0=AUTO detect, 1=FORCE ON, 2=FORCE OFF
static uint8_t gCfgRx5808ModeSelect = 0;
static bool gCfgSdLapLoggingEnabled = true;                     // true=write valid laps to /LOGS/laps.csv on microSD
static char gCfgChannelSelectSource[12] = "AUX7";              // "ADMIN" or "AUX7"
static uint16_t gCfgAuxRangeMinUs[8] = {1540, 1565, 1625, 1665, 1725, 1765, 1825, 1860};
static uint16_t gCfgAuxRangeMaxUs[8] = {1560, 1620, 1660, 1720, 1760, 1820, 1860, 1920};
static char gCfgArmSource[12] = "AUX1";                         // "NONE" or "AUX1"
static uint16_t gCfgArmActiveMinUs = 1700;                      // ARM active range min (us)
static uint16_t gCfgArmActiveMaxUs = 2100;                      // ARM active range max (us)
static unsigned long gCfgNewRaceAfterDisarmMs = 10000;          // Start new race on ARM only if disarmed >= this long

// Limits (values are clamped automatically if out of range).
static const uint8_t kOsdRowMin = 0;
static const uint8_t kOsdRowMax = 17;
static const uint8_t kOsdColMin = 0;
static const uint8_t kOsdColMax = 50;
static const uint8_t kCfgThresholdMin = 60;
static const uint8_t kCfgThresholdMax = 230;
static const uint8_t kEnterThresholdMin = 80;
static const uint8_t kEnterThresholdMax = 100;
static const uint8_t kExitThresholdMin = 60;
static const uint8_t kExitThresholdMax = 75;
static const int8_t kCfgOffsetMin = -60;
static const int8_t kCfgOffsetMax = 60;
static const unsigned long kCfgCooldownMinMs = 1000;
static const unsigned long kCfgCooldownMaxMs = 60000;
static const uint8_t kExitConfirmBelowSamples = 100;
// =====================================================================

static const unsigned long kScanStepMs = 2;
// Timing mode RSSI sampling period. Lower value -> more frequent gate checks.
static const unsigned long kRssiSamplePeriodMs = 2;
static const unsigned long kRssiLogPeriodMs = 500;
static const unsigned long kLpWindowMs = 5000;
static const unsigned long kLpBucketMs = 100;
static const uint8_t kLpBucketCount = static_cast<uint8_t>(kLpWindowMs / kLpBucketMs);
static const bool kLogCrsfChannelChanges = false;
static const bool kLogCrsfVtxPayload = false;
static const bool kLogScanDetails = false;
static const bool kLogTimingRssi = false;
static const bool kLogLapRejects = false;
static const unsigned long kNewRaceAfterDisarmMinMs = 0;
static const unsigned long kNewRaceAfterDisarmMaxMs = 300000;

// ESP-NOW link maintenance
static const size_t kMaxPayload = 80;
static const uint8_t kEspNowDefaultWifiChannel = 1;
static const unsigned long kBindIntervalMs = 500;
static const unsigned long kProbeIntervalDisconnectedMs = 1000;
static const unsigned long kProbeIntervalConnectedMs = 1500;
static const unsigned long kProbeTimeoutMs = 400;
static const unsigned long kLinkTimeoutMs = 5000;
static const unsigned long kTxTimeoutMs = 300;

// OSD placement
static const unsigned long kOsdStatusPeriodMs = 1500;
static const unsigned long kOsdMainRefreshPeriodMs = 1500;
static const unsigned long kLockInfoDisplayMs = 3000;
static const unsigned long kLapPopupDisplayMs = 3000;
static const unsigned long kBestLapBlinkToggleMs = 500;
static const uint8_t kBestLapBlinkCount = 5;
static const unsigned long kBestLapPostBlinkHoldMs = 3000;
static const uint8_t kOutlierHistoryLaps = 7;
static const uint32_t kOutlierFastAbsMs = 5000;   // 5s faster than median
static const uint8_t kOutlierFastRatioPct = 75;   // <75% of median is suspicious

enum TxKind : uint8_t {
  TXK_NONE = 0,
  TXK_BIND_BROADCAST,
  TXK_PROBE_UNICAST
};

enum TimerMode : uint8_t {
  MODE_SCAN = 0,
  MODE_TIMING
};

volatile TxKind gPendingTxKind = TXK_NONE;
volatile bool gProbeInFlight = false;
volatile unsigned long gProbeStartedMs = 0;
volatile bool gLinkConnected = false;
volatile unsigned long gLastLinkSeenMs = 0;
volatile bool gTxAwaiting = false;
volatile unsigned long gTxStartedMs = 0;
uint8_t gEspNowWifiChannel = kEspNowDefaultWifiChannel;

TimerMode gTimerMode = MODE_SCAN;
uint8_t gBestScanRssi = 0;
unsigned long gLastScanStepMs = 0;
bool gVtxCalActive = false;
uint8_t gVtxCalIndex = 0;
bool gVtxCalPeakArmed = false;
bool gRaceNeedsFirstGateCalibration = true;

uint8_t gLockedIndex = 0;
uint8_t gCurrentRssi = 0;
uint8_t gEnterRssi = 140;
uint8_t gExitRssi = 120;
uint8_t gBelowExitStreak = 0;

uint8_t gRssiPeak = 0;
uint32_t gRssiPeakTimeMs = 0;
uint8_t gLastLapRssiPeak = 0;
uint8_t gLpBucketMax[kLpBucketCount] = {};
unsigned long gLpBucketStartMs[kLpBucketCount] = {};
bool gRaceStarted = false;
uint32_t gLastLapPeakMs = 0;
uint32_t gRaceStartMs = 0;
uint32_t gLockAcquiredMs = 0;
uint16_t gLapCount = 0;
uint32_t gSessionLapCount = 0;
uint32_t gLastLapMs = 0;
unsigned long gLastRssiSampleMs = 0;
unsigned long gLastRssiLogMs = 0;
unsigned long gLastOsdStatusMs = 0;
unsigned long gLockInfoUntilMs = 0;
unsigned long gLapPopupUntilMs = 0;
unsigned long gBestLapBlinkStartMs = 0;
bool gBootOsdClearPending = true;
bool gOsdForceFullRefresh = true;

static const uint8_t kLapHistorySize = 32;
static const uint16_t kSessionLapStatsWindow = 100;
static const uint8_t kSessionLapTopPercent = 50;
uint32_t gCurrentRaceLapsMs[kLapHistorySize] = {};
uint8_t gCurrentRaceLapCount = 0;
uint32_t gLastCompletedRaceLapsMs[kLapHistorySize] = {};
uint8_t gLastCompletedRaceLapCount = 0;
uint32_t gAllLapHistoryMs[kLapHistorySize] = {};
uint8_t gAllLapHistoryWrite = 0;
uint8_t gAllLapHistoryCount = 0;
uint32_t gSessionLapStatsMs[kSessionLapStatsWindow] = {};
uint32_t gSessionLapSortScratchMs[kSessionLapStatsWindow] = {};
uint16_t gSessionLapStatsWrite = 0;
uint16_t gSessionLapStatsCount = 0;
uint32_t gBestHalfSessionAvgMs = 0;
uint32_t gBest3ConsecutiveAllTimeMs = 0;
uint32_t gBest3ConsecutiveRaceMs = 0;
uint32_t gRecentRaceLapWindow[3] = {0, 0, 0};
uint8_t gRecentRaceLapWindowCount = 0;
uint32_t gBestLapSessionMs = 0;
uint32_t gBestLapRaceMs = 0;

// Active runtime settings (after clamp/validation)
uint8_t gStrongSignalRssi = 120;
int8_t gEnterRssiOffset = 15;
int8_t gExitRssiOffset = -15;
unsigned long gMinLapIntervalMs = 10000;
uint8_t gExitConfirmSamples = kExitConfirmBelowSamples;
bool gRx5808Enabled = false;
bool gSdReady = false;
bool gSdLoggingRuntimeDisabled = false;
enum ChannelSelectSource : uint8_t {
  CHANNEL_SELECT_SOURCE_ADMIN = 0,
  CHANNEL_SELECT_SOURCE_AUX = 1
};
ChannelSelectSource gChannelSelectSource = CHANNEL_SELECT_SOURCE_ADMIN;
uint8_t gAuxSelectNumber = 7;       // AUX7 by default
uint8_t gAuxSelectCrsfIndex = 10;   // AUX7 -> CH11 -> index 10
uint16_t gAuxRangeMinUs[8] = {};
uint16_t gAuxRangeMaxUs[8] = {};
int gLastAuxRequestedIdx = -1;
bool gArmSourceEnabled = false;
uint8_t gArmAuxNumber = 5;       // AUX5 by default
uint8_t gArmAuxCrsfIndex = 8;    // AUX5 -> CH9 -> index 8
uint16_t gArmActiveMinUs = 1700;
uint16_t gArmActiveMaxUs = 2100;
bool gArmActive = false;
bool gLastArmActive = false;
bool gArmStateKnown = false;
unsigned long gNewRaceAfterDisarmMs = 20000;
uint32_t gRaceCount = 0;
uint32_t gCurrentRaceNo = 0;
unsigned long gLastDisarmMs = 0;
bool gLongDisarmLogged = false;
struct PendingLapLog {
  uint32_t bootMs;
  uint32_t raceNo;
  uint16_t lapNo;
  uint32_t lapMs;
  char channel[4];
  uint8_t isNewBest;
};
static const uint8_t kPendingLapLogCapacity = 64;
PendingLapLog gPendingLapLogs[kPendingLapLogCapacity] = {};
uint8_t gPendingLapLogHead = 0;
uint8_t gPendingLapLogCount = 0;
bool gPendingLapLogOverflow = false;
static const uint8_t kCrsfFrameTypeRcChannelsPacked = 0x16;
static const uint8_t kCrsfFrameTypeVtxAdmin = 0x7C;
static const uint8_t kCrsfMaxFrameSize = 64;
static const uint8_t kCrsfChannelCount = 16;
static const uint8_t kAuxFirstChannelIdx = 4;  // AUX1 == CH5
static const uint16_t kCrsfAuxChangeThresholdUs = 20;
static const uint16_t kCrsfChChangeThresholdUs = 80;
static const unsigned long kCrsfRcWarnMs = 3000;
uint8_t gCrsfFrameBuf[kCrsfMaxFrameSize] = {};
uint8_t gCrsfFramePos = 0;
uint8_t gCrsfFrameExpected = 0;
bool gCrsfChannelsInitialized = false;
bool gCrsfNoDataWarningShown = false;
uint16_t gCrsfLastUs[kCrsfChannelCount] = {};
uint8_t gLastVtxAdminPayload[kCrsfMaxFrameSize] = {};
uint8_t gLastVtxAdminPayloadLen = 0;
int gLastVtxAdminChannelCode = -1;
bool gVtxAdminSeen = false;
bool gVtxChannelPending = false;
uint8_t gVtxChannelPendingIdx = 0;
bool gVtxAdminSyncPending = false;
uint8_t gVtxAdminSyncPendingIdx = 0;

void prepareSdBusIo() {
  // SD and RX5808 share SCK/MOSI lines on current wiring.
  // Release RX-driven GPIO ownership before SD SPI transactions.
  pinMode(kRx5808SelPin, OUTPUT);
  digitalWrite(kRx5808SelPin, HIGH);
  pinMode(kRx5808ClkPin, INPUT);
  pinMode(kRx5808DataPin, INPUT);
  pinMode(kSdSpiCsPin, OUTPUT);
  digitalWrite(kSdSpiCsPin, HIGH);
  SPI.begin(kSdSpiSckPin, kSdSpiMisoPin, kSdSpiMosiPin, kSdSpiCsPin);
  delay(2);
}

void prepareRx5808BusIo() {
  // Keep SD deselected while RX5808 bit-bangs shared lines.
  pinMode(kSdSpiCsPin, OUTPUT);
  digitalWrite(kSdSpiCsPin, HIGH);
  pinMode(kRx5808SelPin, OUTPUT);
  pinMode(kRx5808DataPin, OUTPUT);
  pinMode(kRx5808ClkPin, OUTPUT);
}

bool isArmReadyForCalibration() {
  if (!gArmSourceEnabled) {
    return true;
  }
  return gArmStateKnown && gArmActive;
}

bool isSupportedCrsfAddress(uint8_t b) {
  return crsfIsSupportedAddress(b);
}

bool tryMapVtxAdminCodeToChannelIndex(uint8_t code, uint8_t &index) {
  return crsfTryMapVtxAdminCodeToChannelIndex(code, index);
}

bool tryGetAdminChannelIndex(uint8_t &index) {
  if (!gVtxAdminSeen || gLastVtxAdminChannelCode < 0 || gLastVtxAdminChannelCode > 255) {
    return false;
  }
  return tryMapVtxAdminCodeToChannelIndex(static_cast<uint8_t>(gLastVtxAdminChannelCode), index);
}

uint8_t channelIndexToVtxAdminCode(uint8_t index) {
  return crsfChannelIndexToVtxAdminCode(index);
}

uint8_t channelIndexToMspTableIndex48(uint8_t index) {
  return crsfChannelIndexToMspTableIndex48(index);
}

bool tryExtractVtxAdminRaceChannelCode(const uint8_t *payload, uint8_t payloadLen, uint8_t &outCode) {
  return crsfTryExtractVtxAdminRaceChannelCode(payload, payloadLen, outCode, kTimerChannelCount);
}

void applyUserSettings() {
  gCfgOsdMainRow = static_cast<uint8_t>(constrain(gCfgOsdMainRow, kOsdRowMin, kOsdRowMax));
  gCfgOsdMainCol = static_cast<uint8_t>(constrain(gCfgOsdMainCol, kOsdColMin, kOsdColMax));
  gCfgLapPopupRow = static_cast<uint8_t>(constrain(gCfgLapPopupRow, kOsdRowMin, kOsdRowMax));
  gCfgLapPopupCol = static_cast<uint8_t>(constrain(gCfgLapPopupCol, kOsdColMin, kOsdColMax));
  gCfgOsdChannelRow = static_cast<uint8_t>(constrain(gCfgOsdChannelRow, kOsdRowMin, kOsdRowMax));
  gCfgOsdChannelCol = static_cast<uint8_t>(constrain(gCfgOsdChannelCol, kOsdColMin, kOsdColMax));
  gCfgOsdRssiRow = static_cast<uint8_t>(constrain(gCfgOsdRssiRow, kOsdRowMin, kOsdRowMax));
  gCfgOsdRssiCol = static_cast<uint8_t>(constrain(gCfgOsdRssiCol, kOsdColMin, kOsdColMax));
  gCfgOsdLapPeakRssiRow = static_cast<uint8_t>(constrain(gCfgOsdLapPeakRssiRow, kOsdRowMin, kOsdRowMax));
  gCfgOsdLapPeakRssiCol = static_cast<uint8_t>(constrain(gCfgOsdLapPeakRssiCol, kOsdColMin, kOsdColMax));
  gCfgOsdRssiThrUpperRow = static_cast<uint8_t>(constrain(gCfgOsdRssiThrUpperRow, kOsdRowMin, kOsdRowMax));
  gCfgOsdRssiThrUpperCol = static_cast<uint8_t>(constrain(gCfgOsdRssiThrUpperCol, kOsdColMin, kOsdColMax));
  gCfgOsdRssiThrLowerRow = static_cast<uint8_t>(constrain(gCfgOsdRssiThrLowerRow, kOsdRowMin, kOsdRowMax));
  gCfgOsdRssiThrLowerCol = static_cast<uint8_t>(constrain(gCfgOsdRssiThrLowerCol, kOsdColMin, kOsdColMax));
  gCfgOsdBestLapRow = static_cast<uint8_t>(constrain(gCfgOsdBestLapRow, kOsdRowMin, kOsdRowMax));
  gCfgOsdBestLapCol = static_cast<uint8_t>(constrain(gCfgOsdBestLapCol, kOsdColMin, kOsdColMax));
  gCfgOsdBestLapRaceRow = static_cast<uint8_t>(constrain(gCfgOsdBestLapRaceRow, kOsdRowMin, kOsdRowMax));
  gCfgOsdBestLapRaceCol = static_cast<uint8_t>(constrain(gCfgOsdBestLapRaceCol, kOsdColMin, kOsdColMax));
  gCfgOsdBest3Row = static_cast<uint8_t>(constrain(gCfgOsdBest3Row, kOsdRowMin, kOsdRowMax));
  gCfgOsdBest3Col = static_cast<uint8_t>(constrain(gCfgOsdBest3Col, kOsdColMin, kOsdColMax));
  gCfgOsdBest3RaceRow = static_cast<uint8_t>(constrain(gCfgOsdBest3RaceRow, kOsdRowMin, kOsdRowMax));
  gCfgOsdBest3RaceCol = static_cast<uint8_t>(constrain(gCfgOsdBest3RaceCol, kOsdColMin, kOsdColMax));
  gCfgOsdRaceLapsRow = static_cast<uint8_t>(constrain(gCfgOsdRaceLapsRow, kOsdRowMin, kOsdRowMax));
  gCfgOsdRaceLapsCol = static_cast<uint8_t>(constrain(gCfgOsdRaceLapsCol, kOsdColMin, kOsdColMax));
  gStrongSignalRssi = static_cast<uint8_t>(constrain(gCfgLockThresholdRssi, kCfgThresholdMin, kCfgThresholdMax));
  gEnterRssiOffset = static_cast<int8_t>(constrain(static_cast<int>(gCfgEnterOffsetRssi), static_cast<int>(kCfgOffsetMin),
                                                    static_cast<int>(kCfgOffsetMax)));
  gExitRssiOffset = static_cast<int8_t>(constrain(static_cast<int>(gCfgExitOffsetRssi), static_cast<int>(kCfgOffsetMin),
                                                   static_cast<int>(kCfgOffsetMax)));
  gMinLapIntervalMs = constrain(gCfgMinLapIntervalMs, kCfgCooldownMinMs, kCfgCooldownMaxMs);
  gExitConfirmSamples = kExitConfirmBelowSamples;

  // Channel select source: ADMIN or AUX<n> (e.g. AUX7).
  gChannelSelectSource = CHANNEL_SELECT_SOURCE_ADMIN;
  gAuxSelectNumber = 7;
  gAuxSelectCrsfIndex = static_cast<uint8_t>(kAuxFirstChannelIdx + 6);  // AUX7 -> CH11 index
  {
    char srcBuf[12] = {};
    snprintf(srcBuf, sizeof(srcBuf), "%s", gCfgChannelSelectSource);
    for (size_t i = 0; srcBuf[i] != '\0'; ++i) {
      srcBuf[i] = static_cast<char>(std::toupper(static_cast<unsigned char>(srcBuf[i])));
    }
    if (strcmp(srcBuf, "ADMIN") == 0) {
      gChannelSelectSource = CHANNEL_SELECT_SOURCE_ADMIN;
    } else if (strncmp(srcBuf, "AUX", 3) == 0) {
      char *end = nullptr;
      const long auxNo = strtol(srcBuf + 3, &end, 10);
      if (end != srcBuf + 3 && *end == '\0' && auxNo >= 1 && auxNo <= 12) {
        gChannelSelectSource = CHANNEL_SELECT_SOURCE_AUX;
        gAuxSelectNumber = static_cast<uint8_t>(auxNo);
        gAuxSelectCrsfIndex = static_cast<uint8_t>(kAuxFirstChannelIdx + gAuxSelectNumber - 1);
      }
    }
  }

  for (uint8_t i = 0; i < 8; ++i) {
    uint16_t lo = static_cast<uint16_t>(constrain(static_cast<int>(gCfgAuxRangeMinUs[i]), 800, 2200));
    uint16_t hi = static_cast<uint16_t>(constrain(static_cast<int>(gCfgAuxRangeMaxUs[i]), 800, 2200));
    if (hi < lo) {
      const uint16_t tmp = lo;
      lo = hi;
      hi = tmp;
    }
    gAuxRangeMinUs[i] = lo;
    gAuxRangeMaxUs[i] = hi;
  }
  if (gChannelSelectSource == CHANNEL_SELECT_SOURCE_AUX) {
    gVtxAdminSeen = false;
    gLastVtxAdminChannelCode = -1;
  }
  gLastAuxRequestedIdx = -1;

  // ARM source: NONE or AUX<n> (e.g. AUX5).
  gArmSourceEnabled = false;
  gArmAuxNumber = 5;
  gArmAuxCrsfIndex = static_cast<uint8_t>(kAuxFirstChannelIdx + 4);  // AUX5 -> CH9 index
  {
    char armBuf[12] = {};
    snprintf(armBuf, sizeof(armBuf), "%s", gCfgArmSource);
    for (size_t i = 0; armBuf[i] != '\0'; ++i) {
      armBuf[i] = static_cast<char>(std::toupper(static_cast<unsigned char>(armBuf[i])));
    }
    if (strncmp(armBuf, "AUX", 3) == 0) {
      char *end = nullptr;
      const long auxNo = strtol(armBuf + 3, &end, 10);
      if (end != armBuf + 3 && *end == '\0' && auxNo >= 1 && auxNo <= 12) {
        gArmSourceEnabled = true;
        gArmAuxNumber = static_cast<uint8_t>(auxNo);
        gArmAuxCrsfIndex = static_cast<uint8_t>(kAuxFirstChannelIdx + gArmAuxNumber - 1);
      }
    }
  }

  gArmActiveMinUs = static_cast<uint16_t>(constrain(static_cast<int>(gCfgArmActiveMinUs), 800, 2200));
  gArmActiveMaxUs = static_cast<uint16_t>(constrain(static_cast<int>(gCfgArmActiveMaxUs), 800, 2200));
  gNewRaceAfterDisarmMs =
      constrain(gCfgNewRaceAfterDisarmMs, kNewRaceAfterDisarmMinMs, kNewRaceAfterDisarmMaxMs);
  if (gArmActiveMaxUs < gArmActiveMinUs) {
    const uint16_t tmp = gArmActiveMinUs;
    gArmActiveMinUs = gArmActiveMaxUs;
    gArmActiveMaxUs = tmp;
  }
  if (!gArmSourceEnabled) {
    gArmActive = true;  // NONE => always allow timing
    gArmStateKnown = true;
  } else {
    gArmActive = false;
    gArmStateKnown = false;
  }
  gLastArmActive = gArmActive;
}

void runConfigSelfTest() {
  bool hasOverlap = false;
  bool hasOutOfOrder = false;
  bool hasGap = false;

  for (uint8_t i = 0; i < 8; ++i) {
    if (gAuxRangeMinUs[i] > gAuxRangeMaxUs[i]) {
      hasOutOfOrder = true;
      Serial.printf("CONFIG WARN: AUX range R%u invalid order: %u-%u\n",
                    static_cast<unsigned>(i + 1),
                    static_cast<unsigned>(gAuxRangeMinUs[i]),
                    static_cast<unsigned>(gAuxRangeMaxUs[i]));
    }
    if (i == 0) {
      continue;
    }
    const uint16_t prevMax = gAuxRangeMaxUs[i - 1];
    const uint16_t curMin = gAuxRangeMinUs[i];
    if (curMin <= prevMax) {
      hasOverlap = true;
      Serial.printf("CONFIG WARN: AUX overlap R%u(%u-%u) with R%u(%u-%u)\n",
                    static_cast<unsigned>(i),
                    static_cast<unsigned>(gAuxRangeMinUs[i - 1]),
                    static_cast<unsigned>(gAuxRangeMaxUs[i - 1]),
                    static_cast<unsigned>(i + 1),
                    static_cast<unsigned>(gAuxRangeMinUs[i]),
                    static_cast<unsigned>(gAuxRangeMaxUs[i]));
    } else if (curMin > static_cast<uint16_t>(prevMax + 1U)) {
      hasGap = true;
      Serial.printf("CONFIG INFO: AUX gap between R%u max=%u and R%u min=%u\n",
                    static_cast<unsigned>(i),
                    static_cast<unsigned>(prevMax),
                    static_cast<unsigned>(i + 1),
                    static_cast<unsigned>(curMin));
    }
  }

  if (!hasOutOfOrder && !hasOverlap && !hasGap) {
    Serial.println("CONFIG OK: AUX ranges are ordered and contiguous");
    return;
  }
  if (hasOutOfOrder || hasOverlap) {
    Serial.println("CONFIG WARN: AUX ranges may map channel ambiguously");
  } else {
    Serial.println("CONFIG INFO: AUX ranges are valid but have intentional gaps");
  }
}

uint8_t crc8DvbS2(uint8_t crc, uint8_t data) {
  return crsfCrc8DvbS2(crc, data);
}

void tryFlushPendingVtxAdminSync();

void processCrsfFrame(const uint8_t *frame, uint8_t totalLen) {
  if (totalLen < 6) {
    return;
  }
  const uint8_t len = frame[1];
  if (static_cast<uint8_t>(len + 2U) != totalLen || len < 3) {
    return;
  }

  const uint8_t crcRx = frame[totalLen - 1];
  uint8_t crc = 0;
  for (uint8_t i = 2; i < totalLen - 1; ++i) {
    crc = crc8DvbS2(crc, frame[i]);
  }
  if (crc != crcRx) {
    return;
  }

  const uint8_t type = frame[2];
  const uint8_t payloadLen = static_cast<uint8_t>(len - 2U);  // exclude TYPE and CRC
  const uint8_t destAddr = frame[0];
  if (type == kCrsfFrameTypeVtxAdmin) {
    const uint8_t *payload = &frame[3];
    bool changed = (payloadLen != gLastVtxAdminPayloadLen);
    if (!changed) {
      for (uint8_t i = 0; i < payloadLen; ++i) {
        if (gLastVtxAdminPayload[i] != payload[i]) {
          changed = true;
          break;
        }
      }
    }
    if (changed) {
      gLastVtxAdminPayloadLen = payloadLen;
      for (uint8_t i = 0; i < payloadLen; ++i) {
        gLastVtxAdminPayload[i] = payload[i];
      }
      if (kLogCrsfVtxPayload) {
        Serial.printf("CRSF VTX(0x7C) len=%u payload=", static_cast<unsigned>(payloadLen));
        for (uint8_t i = 0; i < payloadLen; ++i) {
          Serial.printf("%02X", payload[i]);
          if (i + 1 < payloadLen) {
            Serial.print(" ");
          }
        }
        Serial.println();
      }
      // If AUX-triggered channel change happened before we had a template payload,
      // replay it now as soon as first valid VTX ADMIN payload is seen.
      tryFlushPendingVtxAdminSync();
    }

    uint8_t chCode = 0;
    if (tryExtractVtxAdminRaceChannelCode(payload, payloadLen, chCode)) {
      if (gChannelSelectSource == CHANNEL_SELECT_SOURCE_ADMIN) {
        gVtxAdminSeen = true;
      }
      gLastVtxAdminChannelCode = static_cast<int>(chCode);
      if (gChannelSelectSource == CHANNEL_SELECT_SOURCE_ADMIN) {
        uint8_t requestedIdx = 0;
        if (tryMapVtxAdminCodeToChannelIndex(chCode, requestedIdx)) {
          const uint8_t activeIdx = (gVtxCalActive && gVtxCalIndex < kTimerChannelCount) ? gVtxCalIndex : gLockedIndex;
          if (activeIdx != requestedIdx || gTimerMode != MODE_SCAN) {
            gVtxChannelPendingIdx = requestedIdx;
            gVtxChannelPending = true;
          }
        }
      }
    }
  }

  // Fallback: on some links VTX info is routed in non-0x7C frames.
  // Keep it strict to avoid false-positive R1 locks from unrelated payload bytes.
  if (type != kCrsfFrameTypeVtxAdmin &&
      type != kCrsfFrameTypeRcChannelsPacked &&
      gChannelSelectSource == CHANNEL_SELECT_SOURCE_ADMIN) {
    const uint8_t *payload = &frame[3];
    uint8_t chCode = 0;
    if (destAddr == 0xCE &&
        tryExtractVtxAdminRaceChannelCode(payload, payloadLen, chCode) &&
        gLastVtxAdminChannelCode != static_cast<int>(chCode)) {
      gLastVtxAdminChannelCode = static_cast<int>(chCode);
      gVtxAdminSeen = true;
      uint8_t requestedIdx = 0;
      if (tryMapVtxAdminCodeToChannelIndex(chCode, requestedIdx)) {
        gVtxChannelPendingIdx = requestedIdx;
        gVtxChannelPending = true;
      }
    }
  }

  if (type != kCrsfFrameTypeRcChannelsPacked || payloadLen != 22) {
    return;
  }

  uint16_t channels[kCrsfChannelCount] = {};
  uint32_t bitBuf = 0;
  uint8_t bitsInBuf = 0;
  uint8_t src = 3;
  for (uint8_t ch = 0; ch < kCrsfChannelCount; ++ch) {
    while (bitsInBuf < 11 && src < totalLen - 1) {
      bitBuf |= static_cast<uint32_t>(frame[src++]) << bitsInBuf;
      bitsInBuf = static_cast<uint8_t>(bitsInBuf + 8);
    }
    channels[ch] = static_cast<uint16_t>(bitBuf & 0x07FFU);
    bitBuf >>= 11;
    bitsInBuf = static_cast<uint8_t>(bitsInBuf - 11);
  }

  if (!gCrsfChannelsInitialized) {
    for (uint8_t ch = 0; ch < kCrsfChannelCount; ++ch) {
      gCrsfLastUs[ch] = crsfRawToUs(channels[ch]);
    }
    if (gArmSourceEnabled && gArmAuxCrsfIndex < kCrsfChannelCount) {
      const uint16_t armUs = crsfRawToUs(channels[gArmAuxCrsfIndex]);
      gArmActive = (armUs >= gArmActiveMinUs && armUs <= gArmActiveMaxUs);
      gArmStateKnown = true;
      gLastArmActive = gArmActive;
    }
    gCrsfChannelsInitialized = true;
    gCrsfNoDataWarningShown = false;
    Serial.println("CRSF monitor ready (watching CH1..CH16, AUX starts at CH5)");
    return;
  }

  gCrsfNoDataWarningShown = false;

  for (uint8_t ch = 0; ch < kCrsfChannelCount; ++ch) {
    const uint16_t us = crsfRawToUs(channels[ch]);
    const int diff = static_cast<int>(us) - static_cast<int>(gCrsfLastUs[ch]);
    const uint16_t thresholdUs = (ch >= kAuxFirstChannelIdx) ? kCrsfAuxChangeThresholdUs : kCrsfChChangeThresholdUs;
    if (abs(diff) >= static_cast<int>(thresholdUs)) {
      if (kLogCrsfChannelChanges) {
        if (ch >= kAuxFirstChannelIdx) {
          Serial.printf("AUX%u (CH%u) -> %uus\n", static_cast<unsigned>(ch - kAuxFirstChannelIdx + 1),
                        static_cast<unsigned>(ch + 1), static_cast<unsigned>(us));
        } else {
          Serial.printf("CH%u -> %uus\n", static_cast<unsigned>(ch + 1), static_cast<unsigned>(us));
        }
      }
      gCrsfLastUs[ch] = us;
    }
  }

  if (gChannelSelectSource == CHANNEL_SELECT_SOURCE_AUX && gAuxSelectCrsfIndex < kCrsfChannelCount) {
    const uint16_t auxUs = crsfRawToUs(channels[gAuxSelectCrsfIndex]);
    int mappedIdx = -1;
    for (uint8_t i = 0; i < 8; ++i) {
      if (auxUs >= gAuxRangeMinUs[i] && auxUs <= gAuxRangeMaxUs[i]) {
        mappedIdx = static_cast<int>(i);
        break;
      }
    }
    if (mappedIdx >= 0) {
      if (gLastAuxRequestedIdx != mappedIdx) {
        gLastAuxRequestedIdx = mappedIdx;
        gVtxChannelPendingIdx = static_cast<uint8_t>(mappedIdx);
        gVtxChannelPending = true;
        Serial.printf("AUX%u=%uus -> %s\n", static_cast<unsigned>(gAuxSelectNumber),
                      static_cast<unsigned>(auxUs), kTimerChannels[gVtxChannelPendingIdx].name);
      }
    } else {
      gLastAuxRequestedIdx = -1;
    }
  }

  if (gArmSourceEnabled && gArmAuxCrsfIndex < kCrsfChannelCount) {
    const uint16_t armUs = crsfRawToUs(channels[gArmAuxCrsfIndex]);
    const bool armNow = (armUs >= gArmActiveMinUs && armUs <= gArmActiveMaxUs);
    gArmActive = armNow;
    gArmStateKnown = true;
  }
}

void feedCrsfByte(uint8_t b) {
  if (gCrsfFramePos == 0) {
    // Accept common CRSF device addresses because external ELRS wiring may expose
    // different endpoints on UART depending on the receiver/module path.
    if (!isSupportedCrsfAddress(b)) {
      return;
    }
    gCrsfFrameBuf[gCrsfFramePos++] = b;
    gCrsfFrameExpected = 0;
    return;
  }

  if (gCrsfFramePos == 1) {
    gCrsfFrameBuf[gCrsfFramePos++] = b;
    if (b < 3 || static_cast<uint16_t>(b + 2U) > kCrsfMaxFrameSize) {
      gCrsfFramePos = 0;
      gCrsfFrameExpected = 0;
      return;
    }
    gCrsfFrameExpected = static_cast<uint8_t>(b + 2U);
    return;
  }

  if (gCrsfFramePos >= kCrsfMaxFrameSize) {
    gCrsfFramePos = 0;
    gCrsfFrameExpected = 0;
    return;
  }

  gCrsfFrameBuf[gCrsfFramePos++] = b;
  if (gCrsfFrameExpected > 0 && gCrsfFramePos == gCrsfFrameExpected) {
    processCrsfFrame(gCrsfFrameBuf, gCrsfFrameExpected);
    gCrsfFramePos = 0;
    gCrsfFrameExpected = 0;
  }
}

size_t buildMspV2Command(uint16_t function, const uint8_t *payload, uint16_t payloadSize, uint8_t *out, size_t outMax) {
  return crsfBuildMspV2Command(function, payload, payloadSize, out, outMax, kMaxPayload);
}

void onDataSent(
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
    const wifi_tx_info_t *tx_info,
#else
    const uint8_t *mac_addr,
#endif
    esp_now_send_status_t status) {
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  (void)tx_info;
#else
  (void)mac_addr;
#endif

  static unsigned long sLastSendFailLogMs = 0;
  if (status != ESP_NOW_SEND_SUCCESS) {
    const unsigned long now = millis();
    if (now - sLastSendFailLogMs >= 2000UL) {
      sLastSendFailLogMs = now;
      Serial.println("ESP-NOW send callback: FAIL");
    }
  }

  // Any successful unicast telemetry/OSD packet proves the link is alive.
  // Exclude bind broadcast from link tracking.
  if (status == ESP_NOW_SEND_SUCCESS && gPendingTxKind != TXK_BIND_BROADCAST) {
    gLinkConnected = true;
    gLastLinkSeenMs = millis();
  }

  if (gPendingTxKind == TXK_PROBE_UNICAST) {
    gProbeInFlight = false;
  }
  gTxAwaiting = false;
  gPendingTxKind = TXK_NONE;
}

bool sendMspPacket(uint16_t function, const uint8_t *payload, uint16_t payloadSize) {
  uint8_t packet[128] = {};
  const size_t packetLen = buildMspV2Command(function, payload, payloadSize, packet, sizeof(packet));
  if (packetLen == 0) {
    return false;
  }
  return espnowSendWithRetry(kResolvedUidMac, packet, packetLen, function, payloadSize, 3, 2UL, 2000UL);
}

bool sendVtxAdminChannelToElrs(uint8_t channelIndex) {
  if (channelIndex >= kTimerChannelCount) {
    return false;
  }
  // We can only safely rewrite and replay when we have seen a valid VTX ADMIN payload.
  if (gLastVtxAdminPayloadLen < 6 || gLastVtxAdminPayloadLen > 58) {
    return false;
  }

  uint8_t payload[64] = {};
  memcpy(payload, gLastVtxAdminPayload, gLastVtxAdminPayloadLen);
  payload[5] = channelIndexToVtxAdminCode(channelIndex);

  uint8_t frame[64] = {};
  size_t n = 0;
  frame[n++] = 0xC8;  // CRSF sync for transmitted packets
  frame[n++] = static_cast<uint8_t>(gLastVtxAdminPayloadLen + 2U);  // type + payload + crc
  frame[n++] = kCrsfFrameTypeVtxAdmin;
  for (uint8_t i = 0; i < gLastVtxAdminPayloadLen; ++i) {
    frame[n++] = payload[i];
  }
  uint8_t crc = 0;
  for (size_t i = 2; i < n; ++i) {
    crc = crc8DvbS2(crc, frame[i]);
  }
  frame[n++] = crc;

  const size_t written = Serial1.write(frame, n);
  return written == n;
}

void tryFlushPendingVtxAdminSync() {
  if (!gVtxAdminSyncPending) {
    return;
  }
  if (gChannelSelectSource != CHANNEL_SELECT_SOURCE_AUX) {
    gVtxAdminSyncPending = false;
    return;
  }
  if (gVtxAdminSyncPendingIdx >= kTimerChannelCount) {
    gVtxAdminSyncPending = false;
    return;
  }
  if (sendVtxAdminChannelToElrs(gVtxAdminSyncPendingIdx)) {
    Serial.printf("Sent deferred VTX Admin channel update to goggles: %s (0x%02X)\n",
                  kTimerChannels[gVtxAdminSyncPendingIdx].name,
                  static_cast<unsigned>(channelIndexToVtxAdminCode(gVtxAdminSyncPendingIdx)));
    gVtxAdminSyncPending = false;
  }
}

bool sendVrxChannelIndex(uint8_t channelIndex) {
  if (channelIndex >= kTimerChannelCount) {
    return false;
  }
  // ExpressLRS VRX backpack handles MSP_SET_VTX_CONFIG and uses payload[0] as
  // 48-channel table index.
  const uint8_t index48 = channelIndexToMspTableIndex48(channelIndex);
  return sendMspPacket(MSP_SET_VTX_CONFIG, &index48, 1);
}

bool sendOsdText(uint8_t row, uint8_t col, const char *text) {
  uint8_t payload[96] = {};
  size_t n = 0;
  payload[n++] = 0x03;
  payload[n++] = row;
  payload[n++] = col;
  payload[n++] = 0x00;
  const size_t textLen = strlen(text);
  for (size_t i = 0; i < textLen && n < sizeof(payload); ++i) {
    payload[n++] = static_cast<uint8_t>(text[i]);
  }
  return sendMspPacket(MSP_ELRS_SET_OSD, payload, static_cast<uint16_t>(n));
}

bool sendOsdDisplay() {
  const uint8_t payload[] = {0x04};
  return sendMspPacket(MSP_ELRS_SET_OSD, payload, sizeof(payload));
}

void sendOsdMessageAt(uint8_t row, uint8_t col, const char *text) {
  // Do not clear full canvas for short status messages: it causes other
  // elements to disappear until the next composed redraw.
  const char *safeText = (text != nullptr) ? text : "";
  char out[32] = {};
  snprintf(out, sizeof(out), "%-16s", safeText);
  sendOsdText(row, col, out);
  sendOsdDisplay();
  // Status message may overlap composed fields. Force next composed pass to redraw all stats.
  gOsdForceFullRefresh = true;
}

void sendOsdMessage(const char *text) {
  sendOsdMessageAt(gCfgLapPopupRow, gCfgLapPopupCol, text);
}

void clearOsdElementsOnBoot() {
  static const char *kBlank = "                ";
  if (!(gCfgOsdMainCol == 0 && gCfgOsdMainRow == 0)) {
    sendOsdText(gCfgOsdMainRow, gCfgOsdMainCol, kBlank);
  }
  if (!(gCfgOsdChannelCol == 0 && gCfgOsdChannelRow == 0)) {
    sendOsdText(gCfgOsdChannelRow, gCfgOsdChannelCol, kBlank);
  }
  if (!(gCfgOsdRssiCol == 0 && gCfgOsdRssiRow == 0)) {
    sendOsdText(gCfgOsdRssiRow, gCfgOsdRssiCol, kBlank);
  }
  if (!(gCfgOsdLapPeakRssiCol == 0 && gCfgOsdLapPeakRssiRow == 0)) {
    sendOsdText(gCfgOsdLapPeakRssiRow, gCfgOsdLapPeakRssiCol, kBlank);
  }
  if (!(gCfgOsdRssiThrUpperCol == 0 && gCfgOsdRssiThrUpperRow == 0)) {
    sendOsdText(gCfgOsdRssiThrUpperRow, gCfgOsdRssiThrUpperCol, kBlank);
  }
  if (!(gCfgOsdRssiThrLowerCol == 0 && gCfgOsdRssiThrLowerRow == 0)) {
    sendOsdText(gCfgOsdRssiThrLowerRow, gCfgOsdRssiThrLowerCol, kBlank);
  }
  if (!(gCfgOsdBestLapCol == 0 && gCfgOsdBestLapRow == 0)) {
    sendOsdText(gCfgOsdBestLapRow, gCfgOsdBestLapCol, kBlank);
  }
  if (!(gCfgOsdBestLapRaceCol == 0 && gCfgOsdBestLapRaceRow == 0)) {
    sendOsdText(gCfgOsdBestLapRaceRow, gCfgOsdBestLapRaceCol, kBlank);
  }
  if (!(gCfgOsdBest3Col == 0 && gCfgOsdBest3Row == 0)) {
    sendOsdText(gCfgOsdBest3Row, gCfgOsdBest3Col, kBlank);
  }
  if (!(gCfgOsdBest3RaceCol == 0 && gCfgOsdBest3RaceRow == 0)) {
    sendOsdText(gCfgOsdBest3RaceRow, gCfgOsdBest3RaceCol, kBlank);
  }
  if (!(gCfgOsdRaceLapsCol == 0 && gCfgOsdRaceLapsRow == 0)) {
    for (uint8_t row = gCfgOsdRaceLapsRow; row <= kOsdRowMax; ++row) {
      sendOsdText(row, gCfgOsdRaceLapsCol, kBlank);
      if (row == kOsdRowMax) {
        break;
      }
    }
  }
  if (!(gCfgLapPopupCol == 0 && gCfgLapPopupRow == 0)) {
    sendOsdText(gCfgLapPopupRow, gCfgLapPopupCol, kBlank);
  }
  sendOsdText(kOsdWaitVtxAdminRow, kOsdWaitVtxAdminCol, kBlank);
  sendOsdDisplay();
  gOsdForceFullRefresh = true;
}

bool writeConfigToSd() {
  if (!gSdReady) {
    return false;
  }
  SD.remove(kSdConfigPath);
  File cfg = SD.open(kSdConfigPath, FILE_WRITE);
  if (!cfg) {
    return false;
  }

  cfg.println("# Wilde Timer config");
  cfg.println("osd_main_row=" + String(gCfgOsdMainRow));
  cfg.println("osd_main_col=" + String(gCfgOsdMainCol));
  cfg.println("gOsdChannel=[" + String(gCfgOsdChannelCol) + "," + String(gCfgOsdChannelRow) + "," +
              String(gCfgOsdChannelShowDuringRace ? 1 : 0) + "]");
  cfg.println("gOsdRssi=[" + String(gCfgOsdRssiCol) + "," + String(gCfgOsdRssiRow) + "," +
              String(gCfgOsdRssiShowDuringRace ? 1 : 0) + "]");
  cfg.println("gOsdLapPeakRssi=[" + String(gCfgOsdLapPeakRssiCol) + "," + String(gCfgOsdLapPeakRssiRow) + "," +
              String(gCfgOsdLapPeakRssiShowDuringRace ? 1 : 0) + "]");
  cfg.println("gOsdRssiThrUpper=[" + String(gCfgOsdRssiThrUpperCol) + "," + String(gCfgOsdRssiThrUpperRow) + "," +
              String(gCfgOsdRssiThrUpperShowDuringRace ? 1 : 0) + "]");
  cfg.println("gOsdRssiThrLower=[" + String(gCfgOsdRssiThrLowerCol) + "," + String(gCfgOsdRssiThrLowerRow) + "," +
              String(gCfgOsdRssiThrLowerShowDuringRace ? 1 : 0) + "]");
  cfg.println("gOsdBestLap=[" + String(gCfgOsdBestLapCol) + "," + String(gCfgOsdBestLapRow) + "," +
              String(gCfgOsdBestLapShowDuringRace ? 1 : 0) + "]");
  cfg.println("gOsdBestLap_race=[" + String(gCfgOsdBestLapRaceCol) + "," + String(gCfgOsdBestLapRaceRow) + "," +
              String(gCfgOsdBestLapRaceShowDuringRace ? 1 : 0) + "]");
  cfg.println("gOsdBest3=[" + String(gCfgOsdBest3Col) + "," + String(gCfgOsdBest3Row) + "," +
              String(gCfgOsdBest3ShowDuringRace ? 1 : 0) + "]");
  cfg.println("gOsdBest3_race=[" + String(gCfgOsdBest3RaceCol) + "," + String(gCfgOsdBest3RaceRow) + "," +
              String(gCfgOsdBest3RaceShowDuringRace ? 1 : 0) + "]");
  cfg.println("gOsdRaceLaps=[" + String(gCfgOsdRaceLapsCol) + "," + String(gCfgOsdRaceLapsRow) + "," +
              String(gCfgOsdRaceLapsShowDuringRace ? 1 : 0) + "]");
  cfg.println("gOsdLapPopup=[" + String(gCfgLapPopupCol) + "," + String(gCfgLapPopupRow) + "," +
              String(gCfgLapPopupShowDuringRace ? 1 : 0) + "]");
  cfg.println("lock_threshold_rssi=" + String(gCfgLockThresholdRssi));
  cfg.println("enter_offset_rssi=" + String(gCfgEnterOffsetRssi));
  cfg.println("exit_offset_rssi=" + String(gCfgExitOffsetRssi));
  cfg.println("min_lap_interval_ms=" + String(gCfgMinLapIntervalMs));
  cfg.println("post_lock_ignore_ms=" + String(gCfgPostLockIgnoreMs));
  cfg.println("rx5808_mode_select=" + String(gCfgRx5808ModeSelect));
  cfg.println("sd_lap_logging_enabled=" + String(gCfgSdLapLoggingEnabled ? 1 : 0));
  cfg.println("channel_select_source=" + String(gCfgChannelSelectSource));
  for (uint8_t i = 0; i < 8; ++i) {
    cfg.println("aux_range_r" + String(i + 1) + "=" + String(gCfgAuxRangeMinUs[i]) + "-" + String(gCfgAuxRangeMaxUs[i]));
  }
  cfg.println("arm_source=" + String(gCfgArmSource));
  cfg.println("arm_active_min_us=" + String(gCfgArmActiveMinUs));
  cfg.println("arm_active_max_us=" + String(gCfgArmActiveMaxUs));
  cfg.println("new_race_after_disarm_ms=" + String(gCfgNewRaceAfterDisarmMs));
  cfg.close();
  return true;
}

bool loadConfigFromSd(bool &hadMissingKeys) {
  hadMissingKeys = false;
  if (!gSdReady) {
    return false;
  }
  if (!SD.exists(kSdConfigPath)) {
    return false;
  }
  File cfg = SD.open(kSdConfigPath, FILE_READ);
  if (!cfg) {
    return false;
  }

  bool foundOsdMainRow = false;
  bool foundOsdMainCol = false;
  bool foundOsdChannel = false;
  bool foundOsdRssi = false;
  bool foundOsdLapPeakRssi = false;
  bool foundOsdRssiThrUpper = false;
  bool foundOsdRssiThrLower = false;
  bool foundOsdBestLap = false;
  bool foundOsdBestLapRace = false;
  bool foundOsdBest3 = false;
  bool foundOsdBest3Race = false;
  bool foundOsdRaceLaps = false;
  bool foundOsdLapPopup = false;
  bool foundLockThresholdRssi = false;
  bool foundEnterOffsetRssi = false;
  bool foundExitOffsetRssi = false;
  bool foundMinLapIntervalMs = false;
  bool foundPostLockIgnoreMs = false;
  bool foundRx5808ModeSelect = false;
  bool foundSdLapLoggingEnabled = false;
  bool foundChannelSelectSource = false;
  bool foundArmSource = false;
  bool foundArmActiveMinUs = false;
  bool foundArmActiveMaxUs = false;
  bool foundNewRaceAfterDisarmMs = false;
  bool foundAuxRange[8] = {};
  bool missingOsdShowDuringRaceFlag = false;

  char lineBuf[160];
  while (cfg.available()) {
    const size_t n = cfg.readBytesUntil('\n', lineBuf, sizeof(lineBuf) - 1);
    lineBuf[n] = '\0';
    char *line = trimInPlace(lineBuf);
    if (line[0] == '\0' || line[0] == '#' || line[0] == ';') {
      continue;
    }
    char *eq = strchr(line, '=');
    if (!eq) {
      continue;
    }
    *eq = '\0';
    char *key = trimInPlace(line);
    char *val = trimInPlace(eq + 1);

    long sl = 0;
    unsigned long ul = 0;
    bool bv = false;
    uint8_t pointCol = 0;
    uint8_t pointRow = 0;
    bool pointShowDuringRace = true;
    bool pointHasShowFlag = false;
    if (strcmp(key, "osd_main_row") == 0) {
      foundOsdMainRow = true;
      if (parseUnsignedLongStrict(val, ul)) {
      gCfgOsdMainRow = static_cast<uint8_t>(ul);
      }
    } else if (strcmp(key, "osd_main_col") == 0) {
      foundOsdMainCol = true;
      if (parseUnsignedLongStrict(val, ul)) {
      gCfgOsdMainCol = static_cast<uint8_t>(ul);
      }
    } else if (strcmp(key, "gOsdChannel") == 0) {
      foundOsdChannel = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgOsdChannelCol = pointCol;
        gCfgOsdChannelRow = pointRow;
        if (pointHasShowFlag) {
          gCfgOsdChannelShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "gOsdRssi") == 0) {
      foundOsdRssi = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgOsdRssiCol = pointCol;
        gCfgOsdRssiRow = pointRow;
        if (pointHasShowFlag) {
          gCfgOsdRssiShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "gOsdLapPeakRssi") == 0) {
      foundOsdLapPeakRssi = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgOsdLapPeakRssiCol = pointCol;
        gCfgOsdLapPeakRssiRow = pointRow;
        if (pointHasShowFlag) {
          gCfgOsdLapPeakRssiShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "gOsdRssiThrUpper") == 0) {
      foundOsdRssiThrUpper = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgOsdRssiThrUpperCol = pointCol;
        gCfgOsdRssiThrUpperRow = pointRow;
        if (pointHasShowFlag) {
          gCfgOsdRssiThrUpperShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "gOsdRssiThrLower") == 0) {
      foundOsdRssiThrLower = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgOsdRssiThrLowerCol = pointCol;
        gCfgOsdRssiThrLowerRow = pointRow;
        if (pointHasShowFlag) {
          gCfgOsdRssiThrLowerShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "gOsdBestLap") == 0 || strcmp(key, "gOsdStat2") == 0) {
      foundOsdBestLap = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgOsdBestLapCol = pointCol;
        gCfgOsdBestLapRow = pointRow;
        if (pointHasShowFlag) {
          gCfgOsdBestLapShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "gOsdBestLap_race") == 0) {
      foundOsdBestLapRace = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgOsdBestLapRaceCol = pointCol;
        gCfgOsdBestLapRaceRow = pointRow;
        if (pointHasShowFlag) {
          gCfgOsdBestLapRaceShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "gOsdBest3") == 0) {
      foundOsdBest3 = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgOsdBest3Col = pointCol;
        gCfgOsdBest3Row = pointRow;
        if (pointHasShowFlag) {
          gCfgOsdBest3ShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "gOsdBest3_race") == 0) {
      foundOsdBest3Race = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgOsdBest3RaceCol = pointCol;
        gCfgOsdBest3RaceRow = pointRow;
        if (pointHasShowFlag) {
          gCfgOsdBest3RaceShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "gOsdRaceLaps") == 0) {
      foundOsdRaceLaps = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgOsdRaceLapsCol = pointCol;
        gCfgOsdRaceLapsRow = pointRow;
        if (pointHasShowFlag) {
          gCfgOsdRaceLapsShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "gOsdLapPopup") == 0) {
      foundOsdLapPopup = true;
      if (parseOsdPointWithFlagStrict(val, pointCol, pointRow, pointShowDuringRace, pointHasShowFlag)) {
        gCfgLapPopupCol = pointCol;
        gCfgLapPopupRow = pointRow;
        if (pointHasShowFlag) {
          gCfgLapPopupShowDuringRace = pointShowDuringRace;
        } else {
          missingOsdShowDuringRaceFlag = true;
        }
      }
    } else if (strcmp(key, "lock_threshold_rssi") == 0) {
      foundLockThresholdRssi = true;
      if (parseUnsignedLongStrict(val, ul)) {
      gCfgLockThresholdRssi = static_cast<uint8_t>(ul);
      }
    } else if (strcmp(key, "enter_offset_rssi") == 0) {
      foundEnterOffsetRssi = true;
      if (parseSignedLongStrict(val, sl)) {
      gCfgEnterOffsetRssi = static_cast<int8_t>(sl);
      }
    } else if (strcmp(key, "exit_offset_rssi") == 0) {
      foundExitOffsetRssi = true;
      if (parseSignedLongStrict(val, sl)) {
      gCfgExitOffsetRssi = static_cast<int8_t>(sl);
      }
    } else if (strcmp(key, "min_lap_interval_ms") == 0) {
      foundMinLapIntervalMs = true;
      if (parseUnsignedLongStrict(val, ul)) {
      gCfgMinLapIntervalMs = ul;
      }
    } else if (strcmp(key, "post_lock_ignore_ms") == 0) {
      foundPostLockIgnoreMs = true;
      if (parseUnsignedLongStrict(val, ul)) {
      gCfgPostLockIgnoreMs = ul;
      }
    } else if (strcmp(key, "rx5808_mode_select") == 0) {
      foundRx5808ModeSelect = true;
      if (parseUnsignedLongStrict(val, ul)) {
      gCfgRx5808ModeSelect = static_cast<uint8_t>(ul);
      }
    } else if (strcmp(key, "sd_lap_logging_enabled") == 0) {
      foundSdLapLoggingEnabled = true;
      if (parseBoolStrict(val, bv)) {
      gCfgSdLapLoggingEnabled = bv;
      }
    } else if (strcmp(key, "channel_select_source") == 0) {
      foundChannelSelectSource = true;
      snprintf(gCfgChannelSelectSource, sizeof(gCfgChannelSelectSource), "%s", val);
    } else if (strncmp(key, "aux_range_r", 11) == 0) {
      char *end = nullptr;
      const long rNo = strtol(key + 11, &end, 10);
      if (end != key + 11 && *end == '\0' && rNo >= 1 && rNo <= 8) {
        foundAuxRange[rNo - 1] = true;
        uint16_t lo = 0;
        uint16_t hi = 0;
        if (parseUsRangeStrict(val, lo, hi)) {
          const uint8_t idx = static_cast<uint8_t>(rNo - 1);
          gCfgAuxRangeMinUs[idx] = lo;
          gCfgAuxRangeMaxUs[idx] = hi;
        }
      }
    } else if (strcmp(key, "arm_source") == 0) {
      foundArmSource = true;
      snprintf(gCfgArmSource, sizeof(gCfgArmSource), "%s", val);
    } else if (strcmp(key, "arm_active_min_us") == 0) {
      foundArmActiveMinUs = true;
      if (parseUnsignedLongStrict(val, ul)) {
      gCfgArmActiveMinUs = static_cast<uint16_t>(ul);
      }
    } else if (strcmp(key, "arm_active_max_us") == 0) {
      foundArmActiveMaxUs = true;
      if (parseUnsignedLongStrict(val, ul)) {
      gCfgArmActiveMaxUs = static_cast<uint16_t>(ul);
      }
    } else if (strcmp(key, "new_race_after_disarm_ms") == 0) {
      foundNewRaceAfterDisarmMs = true;
      if (parseUnsignedLongStrict(val, ul)) {
      gCfgNewRaceAfterDisarmMs = ul;
      }
    }
  }

  cfg.close();
  if (!foundOsdChannel) {
    gCfgOsdChannelRow = gCfgOsdMainRow;
    gCfgOsdChannelCol = 1;
  }
  if (!foundOsdRssi) {
    gCfgOsdRssiRow = gCfgOsdMainRow;
    gCfgOsdRssiCol = static_cast<uint8_t>(constrain(static_cast<int>(gCfgOsdMainCol) - 4, kOsdColMin, kOsdColMax));
  }
  if (!foundOsdLapPeakRssi) {
    gCfgOsdLapPeakRssiRow = gCfgOsdRssiRow;
    gCfgOsdLapPeakRssiCol =
        static_cast<uint8_t>(constrain(static_cast<int>(gCfgOsdRssiCol) + 6, kOsdColMin, kOsdColMax));
  }
  if (!foundOsdRssiThrUpper) {
    gCfgOsdRssiThrUpperRow = gCfgOsdLapPeakRssiRow;
    gCfgOsdRssiThrUpperCol =
        static_cast<uint8_t>(constrain(static_cast<int>(gCfgOsdLapPeakRssiCol) + 7, kOsdColMin, kOsdColMax));
  }
  if (!foundOsdRssiThrLower) {
    gCfgOsdRssiThrLowerRow = gCfgOsdLapPeakRssiRow;
    gCfgOsdRssiThrLowerCol =
        static_cast<uint8_t>(constrain(static_cast<int>(gCfgOsdLapPeakRssiCol) + 14, kOsdColMin, kOsdColMax));
  }
  if (!foundOsdBestLap) {
    gCfgOsdBestLapRow = gCfgOsdMainRow;
    gCfgOsdBestLapCol = 12;
  }
  if (!foundOsdBestLapRace) {
    gCfgOsdBestLapRaceRow = gCfgOsdMainRow;
    gCfgOsdBestLapRaceCol = 24;
  }
  if (!foundOsdBest3) {
    gCfgOsdBest3Row = gCfgOsdMainRow;
    gCfgOsdBest3Col = 36;
  }
  if (!foundOsdRaceLaps) {
    gCfgOsdRaceLapsRow = static_cast<uint8_t>(constrain(static_cast<int>(gCfgOsdMainRow) - 12, kOsdRowMin, kOsdRowMax));
    gCfgOsdRaceLapsCol = static_cast<uint8_t>(constrain(static_cast<int>(gCfgOsdMainCol) + 21, kOsdColMin, kOsdColMax));
  }
  if (!foundOsdBest3Race) {
    gCfgOsdBest3RaceRow = static_cast<uint8_t>(constrain(static_cast<int>(gCfgOsdRaceLapsRow) - 1, kOsdRowMin, kOsdRowMax));
    gCfgOsdBest3RaceCol = static_cast<uint8_t>(constrain(static_cast<int>(gCfgOsdMainCol) + 21, kOsdColMin, kOsdColMax));
  }
  bool auxMissing = false;
  for (uint8_t i = 0; i < 8; ++i) {
    if (!foundAuxRange[i]) {
      auxMissing = true;
      break;
    }
  }
  hadMissingKeys = !foundOsdMainRow || !foundOsdMainCol ||
                   !foundOsdChannel || !foundOsdRssi || !foundOsdLapPeakRssi || !foundOsdRssiThrUpper || !foundOsdRssiThrLower ||
                   !foundOsdBestLap || !foundOsdBestLapRace ||
                   !foundOsdBest3 ||
                   !foundOsdBest3Race ||
                   !foundOsdRaceLaps ||
                   !foundOsdLapPopup ||
                   !foundLockThresholdRssi || !foundEnterOffsetRssi || !foundExitOffsetRssi || !foundMinLapIntervalMs ||
                   !foundPostLockIgnoreMs ||
                   !foundRx5808ModeSelect || !foundSdLapLoggingEnabled || !foundChannelSelectSource ||
                   !foundArmSource || !foundArmActiveMinUs || !foundArmActiveMaxUs || !foundNewRaceAfterDisarmMs ||
                   missingOsdShowDuringRaceFlag ||
                   auxMissing;
  return true;
}

bool ensureLapsCsvHeaderOnSd() {
  if (!gSdReady || !gCfgSdLapLoggingEnabled) {
    return false;
  }
  if (!SD.exists(kSdLogsDirPath)) {
    if (!SD.mkdir(kSdLogsDirPath)) {
      return false;
    }
  }
  if (SD.exists(kSdLapsPath)) {
    return true;
  }
  File initFile = SD.open(kSdLapsPath, FILE_WRITE);
  if (!initFile) {
    return false;
  }
  initFile.println("boot_ms,race_no,lap_no,lap_ms,channel,is_new_best");
  initFile.close();
  return true;
}

bool initSdCardBus(bool verbose) {
  prepareSdBusIo();
  if (!SD.begin(kSdSpiCsPin, SPI, kSdSpiHz)) {
    gSdReady = false;
    if (verbose) {
      Serial.println("SD init failed (using hardcoded config)");
    }
    return false;
  }
  gSdReady = true;
  return true;
}

#if defined(CONFIG_IDF_TARGET_ESP32S3) && SOC_USB_OTG_SUPPORTED && !ARDUINO_USB_MODE
static int32_t onUsbMscWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  const uint32_t secSize = SD.sectorSize();
  if (!secSize || !buffer) {
    return -1;
  }
  if ((offset % secSize) != 0 || (bufsize % secSize) != 0) {
    return -1;
  }
  prepareSdBusIo();
  uint32_t sector = lba + (offset / secSize);
  const uint32_t blockCount = bufsize / secSize;
  for (uint32_t i = 0; i < blockCount; ++i) {
    if (!SD.writeRAW(buffer + (i * secSize), sector + i)) {
      return -1;
    }
  }
  return static_cast<int32_t>(bufsize);
}

static int32_t onUsbMscRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  const uint32_t secSize = SD.sectorSize();
  if (!secSize || !buffer) {
    return -1;
  }
  if ((offset % secSize) != 0 || (bufsize % secSize) != 0) {
    return -1;
  }
  prepareSdBusIo();
  uint32_t sector = lba + (offset / secSize);
  const uint32_t blockCount = bufsize / secSize;
  for (uint32_t i = 0; i < blockCount; ++i) {
    if (!SD.readRAW(static_cast<uint8_t *>(buffer) + (i * secSize), sector + i)) {
      return -1;
    }
  }
  return static_cast<int32_t>(bufsize);
}

static bool onUsbMscStartStop(uint8_t power_condition, bool start, bool load_eject) {
  (void)power_condition;
  gUsbMscHostActive = start;
  Serial.printf("USB MSC start/stop: start=%u eject=%u\n", static_cast<unsigned>(start), static_cast<unsigned>(load_eject));
  return true;
}

static void onUsbEvent(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  (void)arg;
  (void)event_data;
  if (event_base != ARDUINO_USB_EVENTS) {
    return;
  }
  switch (event_id) {
    case ARDUINO_USB_STARTED_EVENT:
      gUsbMscHostActive = true;
      Serial.println("USB connected: MSC active");
      break;
    case ARDUINO_USB_STOPPED_EVENT:
      gUsbMscHostActive = false;
      Serial.println("USB disconnected: timer active");
      break;
    default:
      break;
  }
}

void setupUsbMassStorageIfAvailable() {
  if (gUsbMscEnabled) {
    return;
  }
  if (!gSdReady) {
    Serial.println("USB MSC skipped: SD not ready");
    return;
  }

  const uint32_t sectorSize = static_cast<uint32_t>(SD.sectorSize());
  const uint32_t sectorCount = static_cast<uint32_t>(SD.numSectors());
  if (!sectorSize || !sectorCount) {
    Serial.println("USB MSC skipped: invalid SD geometry");
    return;
  }

  gUsbMsc.vendorID("WILDE");
  gUsbMsc.productID("TIMER_SD");
  gUsbMsc.productRevision("1.0");
  gUsbMsc.onRead(onUsbMscRead);
  gUsbMsc.onWrite(onUsbMscWrite);
  gUsbMsc.onStartStop(onUsbMscStartStop);
  gUsbMsc.mediaPresent(true);
  gUsbMsc.begin(sectorCount, static_cast<uint16_t>(sectorSize));
  USB.onEvent(onUsbEvent);
  USB.begin();
  gUsbMscEnabled = true;
  Serial.printf("USB MSC ready (read/write): sectors=%lu size=%lu\n",
                static_cast<unsigned long>(sectorCount), static_cast<unsigned long>(sectorSize));
}
#endif

bool reinitSdCardBus(bool verbose) {
  // Soft re-init: avoid SPI.end() because on some cores it can trigger
  // APB callback warnings when called repeatedly during runtime recover.
  SD.end();
  delay(5);
  return initSdCardBus(verbose);
}

bool recoverSdForLogging() {
  if (gSdLoggingRuntimeDisabled) {
    return false;
  }
  if (!reinitSdCardBus(false)) {
    return false;
  }
  if (gCfgSdLapLoggingEnabled) {
    if (!ensureLapsCsvHeaderOnSd()) {
      return false;
    }
  }
  return true;
}

void disableSdLoggingRuntime(const char *reason) {
  if (!gSdLoggingRuntimeDisabled) {
    if (reason && reason[0] != '\0') {
      Serial.printf("SD lap logging disabled until reboot: %s\n", reason);
    } else {
      Serial.println("SD lap logging disabled until reboot");
    }
  }
  gSdLoggingRuntimeDisabled = true;
  gSdReady = false;
  gPendingLapLogHead = 0;
  gPendingLapLogCount = 0;
  gPendingLapLogOverflow = false;
}

void enqueueLapLog(uint16_t lapNo, uint32_t lapMs, bool isNewBest) {
  uint32_t raceNo = gCurrentRaceNo;
  if (raceNo == 0) {
    raceNo = 1;
  }
  uint8_t insertIdx = 0;
  if (gPendingLapLogCount < kPendingLapLogCapacity) {
    insertIdx = static_cast<uint8_t>((gPendingLapLogHead + gPendingLapLogCount) % kPendingLapLogCapacity);
    gPendingLapLogCount++;
  } else {
    // Keep latest logs: overwrite oldest when buffer is full.
    insertIdx = gPendingLapLogHead;
    gPendingLapLogHead = static_cast<uint8_t>((gPendingLapLogHead + 1) % kPendingLapLogCapacity);
    gPendingLapLogOverflow = true;
  }

  PendingLapLog &e = gPendingLapLogs[insertIdx];
  e.bootMs = static_cast<uint32_t>(millis());
  e.raceNo = raceNo;
  e.lapNo = lapNo;
  e.lapMs = lapMs;
  snprintf(e.channel, sizeof(e.channel), "%s", kTimerChannels[gLockedIndex].name);
  e.isNewBest = isNewBest ? 1U : 0U;
}

void flushPendingLapLogsToSd() {
  if (!gCfgSdLapLoggingEnabled || gSdLoggingRuntimeDisabled || gPendingLapLogCount == 0) {
    return;
  }

  if (!gSdReady) {
    if (!recoverSdForLogging()) {
      disableSdLoggingRuntime("mount/recover failed (card missing?)");
      return;
    }
  }

  // Reassert shared-bus ownership for SD before file I/O.
  prepareSdBusIo();
  File f = SD.open(kSdLapsPath, FILE_APPEND);
  if (!f) {
    Serial.println("SD append failed: trying recover");
    gSdReady = false;
    if (!recoverSdForLogging()) {
      disableSdLoggingRuntime("append recover failed (card missing?)");
      return;
    }
    f = SD.open(kSdLapsPath, FILE_APPEND);
    if (!f) {
      Serial.println("SD append failed after recover");
      disableSdLoggingRuntime("append failed after recover");
      return;
    }
  }

  for (uint8_t i = 0; i < gPendingLapLogCount; ++i) {
    const uint8_t idx = static_cast<uint8_t>((gPendingLapLogHead + i) % kPendingLapLogCapacity);
    const PendingLapLog &e = gPendingLapLogs[idx];
    char line[128];
    snprintf(line, sizeof(line), "%lu,%lu,%u,%lu,%s,%u",
             static_cast<unsigned long>(e.bootMs),
             static_cast<unsigned long>(e.raceNo),
             static_cast<unsigned>(e.lapNo),
             static_cast<unsigned long>(e.lapMs),
             e.channel,
             static_cast<unsigned>(e.isNewBest));
    f.println(line);
  }
  f.close();

  if (gPendingLapLogOverflow) {
    Serial.println("Lap log buffer had overflow before flush");
  }
  gPendingLapLogHead = 0;
  gPendingLapLogCount = 0;
  gPendingLapLogOverflow = false;
}

void setupSdLogging() {
  gSdReady = false;
  gSdLoggingRuntimeDisabled = false;
  if (!initSdCardBus(true)) {
    if (gCfgSdLapLoggingEnabled) {
      disableSdLoggingRuntime("SD init failed at boot");
    }
    return;
  }

  bool hadMissingConfigKeys = false;
  if (loadConfigFromSd(hadMissingConfigKeys)) {
    Serial.println("Config loaded from /config.txt");
    if (hadMissingConfigKeys) {
      if (writeConfigToSd()) {
        Serial.println("Config file had missing keys: auto-filled /config.txt");
      } else {
        Serial.println("Config file had missing keys, but auto-fill failed");
      }
    }
  } else {
    if (writeConfigToSd()) {
      Serial.println("Config missing: created /config.txt with defaults");
    } else {
      Serial.println("Config write failed (continuing with hardcoded defaults)");
    }
  }
  if (gCfgSdLapLoggingEnabled) {
    if (ensureLapsCsvHeaderOnSd()) {
      Serial.println("SD ready (lap logging ON)");
    } else {
      Serial.println("SD ready, but /LOGS/laps.csv init failed");
    }
  } else {
    Serial.println("SD ready (lap logging OFF by config)");
  }
#if defined(CONFIG_IDF_TARGET_ESP32S3) && SOC_USB_OTG_SUPPORTED && !ARDUINO_USB_MODE
  setupUsbMassStorageIfAvailable();
#endif
}

void appendLapToSd(uint16_t lapNo, uint32_t lapMs, bool isNewBest) {
  if (!gCfgSdLapLoggingEnabled || gSdLoggingRuntimeDisabled || !gSdReady) {
    return;
  }
  enqueueLapLog(lapNo, lapMs, isNewBest);
}

void sendOsdMessageIfChanged(const char *text) {
  static char sLastStatusMsg[48] = {};
  static unsigned long sLastStatusMsgMs = 0;
  const char *safeText = (text != nullptr) ? text : "";
  const unsigned long now = millis();
  if (strcmp(sLastStatusMsg, safeText) == 0 &&
      now - sLastStatusMsgMs < kOsdMainRefreshPeriodMs) {
    return;
  }
  sendOsdMessage(safeText);
  snprintf(sLastStatusMsg, sizeof(sLastStatusMsg), "%s", safeText);
  sLastStatusMsgMs = now;
}

void appendCurrentRaceLap(uint32_t lapMs) {
  if (gCurrentRaceLapCount < kLapHistorySize) {
    gCurrentRaceLapsMs[gCurrentRaceLapCount++] = lapMs;
    return;
  }
  // Keep a rolling tail if race has more laps than buffer capacity.
  memmove(&gCurrentRaceLapsMs[0], &gCurrentRaceLapsMs[1], sizeof(uint32_t) * (kLapHistorySize - 1U));
  gCurrentRaceLapsMs[kLapHistorySize - 1U] = lapMs;
}

void archiveCurrentRaceLapsAsLastCompleted() {
  gLastCompletedRaceLapCount = gCurrentRaceLapCount;
  for (uint8_t i = 0; i < gCurrentRaceLapCount; ++i) {
    gLastCompletedRaceLapsMs[i] = gCurrentRaceLapsMs[i];
  }
}

void clearCurrentRaceLaps() {
  memset(gCurrentRaceLapsMs, 0, sizeof(gCurrentRaceLapsMs));
  gCurrentRaceLapCount = 0;
}

void pushLapHistory(uint32_t lapMs) {
  if (gRecentRaceLapWindowCount < 3) {
    gRecentRaceLapWindow[gRecentRaceLapWindowCount++] = lapMs;
  } else {
    gRecentRaceLapWindow[0] = gRecentRaceLapWindow[1];
    gRecentRaceLapWindow[1] = gRecentRaceLapWindow[2];
    gRecentRaceLapWindow[2] = lapMs;
  }
  if (gRecentRaceLapWindowCount == 3) {
    const uint32_t sum3Race = gRecentRaceLapWindow[0] + gRecentRaceLapWindow[1] + gRecentRaceLapWindow[2];
    if (gBest3ConsecutiveRaceMs == 0 || sum3Race < gBest3ConsecutiveRaceMs) {
      gBest3ConsecutiveRaceMs = sum3Race;
    }
    // S3 shows best R3 achieved across the whole session.
    if (gBest3ConsecutiveAllTimeMs == 0 || sum3Race < gBest3ConsecutiveAllTimeMs) {
      gBest3ConsecutiveAllTimeMs = sum3Race;
    }
  }
}

void updateBestHalfSessionAverage() {
  if (gSessionLapStatsCount == 0) {
    gBestHalfSessionAvgMs = 0;
    return;
  }

  for (uint16_t i = 0; i < gSessionLapStatsCount; ++i) {
    const uint16_t idx = static_cast<uint16_t>((gSessionLapStatsWrite + kSessionLapStatsWindow - gSessionLapStatsCount + i) %
                                               kSessionLapStatsWindow);
    gSessionLapSortScratchMs[i] = gSessionLapStatsMs[idx];
  }
  gBestHalfSessionAvgMs = timingComputeBestHalfAverage(gSessionLapSortScratchMs, gSessionLapSortScratchMs,
                                                       gSessionLapStatsCount, kSessionLapTopPercent);
}

uint32_t computeSessionWindowBestLapMs() {
  // Fastest lap within the rolling last-100 window. Only counted (non-fake)
  // laps land in gSessionLapStatsMs, so this is inherently glitch-free.
  // Valid entries are indices 0..count-1 (when the window is full, count == window),
  // and min() is order-independent, so the ring offset is irrelevant here.
  uint32_t best = 0;
  for (uint16_t i = 0; i < gSessionLapStatsCount; ++i) {
    const uint32_t v = gSessionLapStatsMs[i];
    if (v == 0) {
      continue;
    }
    if (best == 0 || v < best) {
      best = v;
    }
  }
  return best;
}

void appendSessionLapForBestHalfAverage(uint32_t lapMs) {
  gSessionLapStatsMs[gSessionLapStatsWrite] = lapMs;
  gSessionLapStatsWrite = static_cast<uint16_t>((gSessionLapStatsWrite + 1U) % kSessionLapStatsWindow);
  if (gSessionLapStatsCount < kSessionLapStatsWindow) {
    gSessionLapStatsCount++;
  }
  updateBestHalfSessionAverage();
}

bool isLikelyFakeLap(uint32_t lapMs) {
  TimingOutlierHistory history;
  history.lapMs = gAllLapHistoryMs;
  history.writeIndex = &gAllLapHistoryWrite;
  history.count = &gAllLapHistoryCount;
  history.capacity = kLapHistorySize;
  return timingDetectAndStoreFakeLap(history, lapMs, kOutlierHistoryLaps, kOutlierFastAbsMs, kOutlierFastRatioPct);
}

bool timerActive(unsigned long now, unsigned long untilMs) {
  // Wrap-safe check for millis()-based deadlines.
  return untilMs != 0 && static_cast<long>(untilMs - now) > 0;
}

bool sendRaceLapsColumnOsd(const uint32_t *lapsMs, uint8_t lapCount, bool allowRender) {
  static OsdRaceLapsRenderCache sCache;
  return osdSendRaceLapsColumn(lapsMs, lapCount, allowRender, gCfgOsdRaceLapsRow, gCfgOsdRaceLapsCol,
                               kOsdRowMax, sCache, sendOsdText);
}

bool sendOsdComposedStatus(const char *channelText, const char *rssiText, const char *lapPeakRssiText,
                           const char *rssiThrUpperText, const char *rssiThrLowerText,
                           const char *bestLapText,
                           const char *bestLapRaceText,
                           const char *best3Text, const char *best3RaceText,
                           const char *lapPopupText,
                           const char *waitVtxAdminText,
                           bool raceInProgress) {
  return osdSendComposedStatus(
      gOsdForceFullRefresh, kOsdColMax,
      channelText, rssiText, lapPeakRssiText, rssiThrUpperText, rssiThrLowerText,
      bestLapText, bestLapRaceText, best3Text, best3RaceText,
      lapPopupText, waitVtxAdminText,
      gCfgOsdChannelRow, gCfgOsdChannelCol, gCfgOsdChannelShowDuringRace,
      gCfgOsdRssiRow, gCfgOsdRssiCol, gCfgOsdRssiShowDuringRace,
      gCfgOsdLapPeakRssiRow, gCfgOsdLapPeakRssiCol, gCfgOsdLapPeakRssiShowDuringRace,
      gCfgOsdRssiThrUpperRow, gCfgOsdRssiThrUpperCol, gCfgOsdRssiThrUpperShowDuringRace,
      gCfgOsdRssiThrLowerRow, gCfgOsdRssiThrLowerCol, gCfgOsdRssiThrLowerShowDuringRace,
      gCfgOsdBestLapRow, gCfgOsdBestLapCol, gCfgOsdBestLapShowDuringRace,
      gCfgOsdBestLapRaceRow, gCfgOsdBestLapRaceCol, gCfgOsdBestLapRaceShowDuringRace,
      gCfgOsdBest3Row, gCfgOsdBest3Col, gCfgOsdBest3ShowDuringRace,
      gCfgOsdBest3RaceRow, gCfgOsdBest3RaceCol, gCfgOsdBest3RaceShowDuringRace,
      gCfgLapPopupRow, gCfgLapPopupCol, gCfgLapPopupShowDuringRace,
      kOsdWaitVtxAdminRow, kOsdWaitVtxAdminCol,
      raceInProgress, sendOsdText);
}

void sendLockedStatusOsd() {
  const unsigned long now = millis();
  const bool armActiveForTiming = !gArmSourceEnabled || (gArmStateKnown && gArmActive);
  const bool raceInProgress = (gTimerMode == MODE_TIMING) && armActiveForTiming;
  bool showLapPopup = timerActive(now, gLapPopupUntilMs) && gLastLapMs > 0;
  char lapPopup[32] = {};
  if (showLapPopup) {
    const uint32_t lastSec = gLastLapMs / 1000U;
    const uint32_t lastCs = (gLastLapMs % 1000U) / 10U;
    if (gBestHalfSessionAvgMs > 0) {
      const int32_t deltaMs = static_cast<int32_t>(gLastLapMs) - static_cast<int32_t>(gBestHalfSessionAvgMs);
      const uint32_t deltaAbsMs = static_cast<uint32_t>(deltaMs < 0 ? -deltaMs : deltaMs);
      const uint32_t deltaSec = deltaAbsMs / 1000U;
      const uint32_t deltaCs = (deltaAbsMs % 1000U) / 10U;
      const char deltaSign = (deltaMs < 0) ? '-' : '+';
      snprintf(lapPopup, sizeof(lapPopup), "L%02lu %02lu.%02lu %c%02lu.%02lu",
               static_cast<unsigned long>(gSessionLapCount),
               static_cast<unsigned long>(lastSec), static_cast<unsigned long>(lastCs), deltaSign,
               static_cast<unsigned long>(deltaSec), static_cast<unsigned long>(deltaCs));
    } else {
      snprintf(lapPopup, sizeof(lapPopup), "L%02lu %02lu.%02lu",
               static_cast<unsigned long>(gSessionLapCount),
               static_cast<unsigned long>(lastSec), static_cast<unsigned long>(lastCs));
    }
    if (gBestLapBlinkStartMs != 0) {
      const unsigned long elapsed = now - gBestLapBlinkStartMs;
      const unsigned long toggleIndex = elapsed / kBestLapBlinkToggleMs;
      const unsigned long blinkToggles = static_cast<unsigned long>(kBestLapBlinkCount) * 2UL;
      if (toggleIndex < blinkToggles) {
        const bool blinkOn = (toggleIndex % 2U) == 0U;
        if (!blinkOn) {
          showLapPopup = false;
        }
      } else {
        gBestLapBlinkStartMs = 0;
      }
    }
  } else if (gLapPopupUntilMs != 0) {
    gLapPopupUntilMs = 0;
    gBestLapBlinkStartMs = 0;
  }

  char channelText[8] = {};
  char rssiText[10] = {};
  char lapPeakRssiText[12] = {};
  char rssiThrUpperText[10] = {};
  char rssiThrLowerText[10] = {};
  char bestLapText[20] = "SF:00.00";
  char bestLapRaceText[20] = "RF:00.00";
  char best3Text[20] = "S3:00.00";
  char best3RaceText[20] = "R3:00.00";

  uint8_t channelDisplayIndex = gLockedIndex;
  if (gVtxCalActive && gVtxCalIndex < kTimerChannelCount) {
    // While calibrating (including pre-arm), show the currently requested channel.
    channelDisplayIndex = gVtxCalIndex;
  } else if (gTimerMode == MODE_SCAN &&
             gChannelSelectSource == CHANNEL_SELECT_SOURCE_ADMIN) {
    uint8_t adminIdx = 0;
    if (tryGetAdminChannelIndex(adminIdx)) {
      channelDisplayIndex = adminIdx;
    }
  } else if (gTimerMode == MODE_SCAN &&
             gChannelSelectSource == CHANNEL_SELECT_SOURCE_AUX &&
             gLastAuxRequestedIdx >= 0 &&
             gLastAuxRequestedIdx < 8) {
    channelDisplayIndex = static_cast<uint8_t>(gLastAuxRequestedIdx);
  }
  snprintf(channelText, sizeof(channelText), "%s", kTimerChannels[channelDisplayIndex].name);
  snprintf(rssiText, sizeof(rssiText), "R:%03u", static_cast<unsigned>(gCurrentRssi));
  snprintf(lapPeakRssiText, sizeof(lapPeakRssiText), "LP:%03u", static_cast<unsigned>(gLastLapRssiPeak));
  snprintf(rssiThrUpperText, sizeof(rssiThrUpperText), "T+:%03u", static_cast<unsigned>(gEnterRssi));
  snprintf(rssiThrLowerText, sizeof(rssiThrLowerText), "T-:%03u", static_cast<unsigned>(gExitRssi));

  const bool showLockInfoPlaceholders = timerActive(now, gLockInfoUntilMs) && !gRaceStarted && gLapCount == 0;
  if (showLockInfoPlaceholders) {
    // Keep placeholder times visible during lock-info window for easier OSD placement.
  } else {
    if (gLockInfoUntilMs != 0) {
      gLockInfoUntilMs = 0;
    }

    if (gBestLapSessionMs > 0) {
      const uint32_t bs = gBestLapSessionMs / 1000U;
      const uint32_t bc = (gBestLapSessionMs % 1000U) / 10U;
      snprintf(bestLapText, sizeof(bestLapText), "SF:%02lu.%02lu", static_cast<unsigned long>(bs),
               static_cast<unsigned long>(bc));
    }
    if (gBestLapRaceMs > 0) {
      const uint32_t brs = gBestLapRaceMs / 1000U;
      const uint32_t brc = (gBestLapRaceMs % 1000U) / 10U;
      snprintf(bestLapRaceText, sizeof(bestLapRaceText), "RF:%02lu.%02lu", static_cast<unsigned long>(brs),
               static_cast<unsigned long>(brc));
    }
    if (gBest3ConsecutiveAllTimeMs > 0) {
      const uint32_t b3s = gBest3ConsecutiveAllTimeMs / 1000U;
      const uint32_t b3c = (gBest3ConsecutiveAllTimeMs % 1000U) / 10U;
      snprintf(best3Text, sizeof(best3Text), "S3:%02lu.%02lu", static_cast<unsigned long>(b3s),
               static_cast<unsigned long>(b3c));
    }
    if (gBest3ConsecutiveRaceMs > 0) {
      const uint32_t b3rs = gBest3ConsecutiveRaceMs / 1000U;
      const uint32_t b3rc = (gBest3ConsecutiveRaceMs % 1000U) / 10U;
      snprintf(best3RaceText, sizeof(best3RaceText), "R3:%02lu.%02lu", static_cast<unsigned long>(b3rs),
               static_cast<unsigned long>(b3rc));
    }
  }

  uint8_t adminIdx = 0;
  const bool hasAdminChannel = tryGetAdminChannelIndex(adminIdx);
  const bool waitingForVtxAdmin = (gChannelSelectSource == CHANNEL_SELECT_SOURCE_ADMIN) && !hasAdminChannel;
  const bool composedChanged = sendOsdComposedStatus(channelText, rssiText, lapPeakRssiText, rssiThrUpperText, rssiThrLowerText,
                                                     bestLapText, bestLapRaceText, best3Text, best3RaceText,
                                                     showLapPopup ? lapPopup : nullptr,
                                                     waitingForVtxAdmin ? "WAIT VTX ADMIN" : nullptr,
                                                     raceInProgress);

  const uint32_t *lapsToRender = gCurrentRaceLapsMs;
  uint8_t lapsToRenderCount = gCurrentRaceLapCount;
  if (!raceInProgress && lapsToRenderCount == 0 && gLastCompletedRaceLapCount > 0) {
    lapsToRender = gLastCompletedRaceLapsMs;
    lapsToRenderCount = gLastCompletedRaceLapCount;
  }
  const bool raceLapsVisibleNow =
      osdIsElementVisibleNow(gCfgOsdRaceLapsCol, gCfgOsdRaceLapsRow, gCfgOsdRaceLapsShowDuringRace, raceInProgress);
  const bool raceLapsChanged = sendRaceLapsColumnOsd(lapsToRender, lapsToRenderCount, raceLapsVisibleNow);
  if (composedChanged || raceLapsChanged) {
    sendOsdDisplay();
  }
}

void sendLapPopupBlinkOnly(unsigned long now) {
  static bool sPopupWasVisible = false;
  static char sLastPopup[32] = {};

  const bool armActiveForTiming = !gArmSourceEnabled || (gArmStateKnown && gArmActive);
  const bool raceInProgress = (gTimerMode == MODE_TIMING) && armActiveForTiming;
  const bool lapPopupEnabled = osdIsElementVisibleNow(gCfgLapPopupCol, gCfgLapPopupRow,
                                                      gCfgLapPopupShowDuringRace, raceInProgress);

  bool popupActive = timerActive(now, gLapPopupUntilMs) && gLastLapMs > 0 && lapPopupEnabled;
  char lapPopup[32] = {};
  bool blinkOn = true;

  if (popupActive) {
    const uint32_t lastSec = gLastLapMs / 1000U;
    const uint32_t lastCs = (gLastLapMs % 1000U) / 10U;
    if (gBestHalfSessionAvgMs > 0) {
      const int32_t deltaMs = static_cast<int32_t>(gLastLapMs) - static_cast<int32_t>(gBestHalfSessionAvgMs);
      const uint32_t deltaAbsMs = static_cast<uint32_t>(deltaMs < 0 ? -deltaMs : deltaMs);
      const uint32_t deltaSec = deltaAbsMs / 1000U;
      const uint32_t deltaCs = (deltaAbsMs % 1000U) / 10U;
      const char deltaSign = (deltaMs < 0) ? '-' : '+';
      snprintf(lapPopup, sizeof(lapPopup), "L%02lu %02lu.%02lu %c%02lu.%02lu",
               static_cast<unsigned long>(gSessionLapCount),
               static_cast<unsigned long>(lastSec), static_cast<unsigned long>(lastCs), deltaSign,
               static_cast<unsigned long>(deltaSec), static_cast<unsigned long>(deltaCs));
    } else {
      snprintf(lapPopup, sizeof(lapPopup), "L%02lu %02lu.%02lu",
               static_cast<unsigned long>(gSessionLapCount),
               static_cast<unsigned long>(lastSec), static_cast<unsigned long>(lastCs));
    }

    if (gBestLapBlinkStartMs != 0) {
      const unsigned long elapsed = now - gBestLapBlinkStartMs;
      const unsigned long toggleIndex = elapsed / kBestLapBlinkToggleMs;
      const unsigned long blinkToggles = static_cast<unsigned long>(kBestLapBlinkCount) * 2UL;
      if (toggleIndex < blinkToggles) {
        blinkOn = (toggleIndex % 2U) == 0U;
      } else {
        gBestLapBlinkStartMs = 0;
      }
    }
  } else if (gLapPopupUntilMs != 0) {
    gLapPopupUntilMs = 0;
    gBestLapBlinkStartMs = 0;
  }

  const bool shouldShow = popupActive && blinkOn;
  if (shouldShow) {
    if (!sPopupWasVisible || strcmp(sLastPopup, lapPopup) != 0) {
      if (sendOsdText(gCfgLapPopupRow, gCfgLapPopupCol, lapPopup)) {
        sendOsdDisplay();
        snprintf(sLastPopup, sizeof(sLastPopup), "%s", lapPopup);
      }
    }
    sPopupWasVisible = true;
  } else if (sPopupWasVisible) {
    if (sendOsdText(gCfgLapPopupRow, gCfgLapPopupCol, "                ")) {
      sendOsdDisplay();
    }
    sPopupWasVisible = false;
    sLastPopup[0] = '\0';
  }
}

void sendBindBroadcast() {
  uint8_t bindPacket[64] = {};
  const size_t bindLen =
      buildMspV2Command(MSP_ELRS_BIND, kResolvedUidMac, sizeof(kResolvedUidMac), bindPacket, sizeof(bindPacket));
  if (bindLen == 0) {
    return;
  }
  gPendingTxKind = TXK_BIND_BROADCAST;
  gTxAwaiting = true;
  gTxStartedMs = millis();
  const esp_err_t bindRes = esp_now_send(kBroadcastMac, bindPacket, bindLen);
  if (bindRes != ESP_OK) {
    gPendingTxKind = TXK_NONE;
    gTxAwaiting = false;
  }
}

void sendLinkProbe() {
  const uint8_t payload[] = {0x04};  // OSD display opcode
  uint8_t packet[96] = {};
  const size_t packetLen = buildMspV2Command(MSP_ELRS_SET_OSD, payload, sizeof(payload), packet, sizeof(packet));
  if (packetLen == 0) {
    return;
  }

  gPendingTxKind = TXK_PROBE_UNICAST;
  gProbeInFlight = true;
  gProbeStartedMs = millis();
  gTxAwaiting = true;
  gTxStartedMs = gProbeStartedMs;

  const esp_err_t res = esp_now_send(kResolvedUidMac, packet, packetLen);
  if (res != ESP_OK) {
    gProbeInFlight = false;
    gPendingTxKind = TXK_NONE;
    gTxAwaiting = false;
  }
}

void setupEspNowWithBackpackUid() {
  // Avoid writing WiFi credentials/settings into NVS/flash.
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  gEspNowWifiChannel = kEspNowDefaultWifiChannel;
  Serial.printf("ESP-NOW fixed channel: CH%u\n", static_cast<unsigned>(gEspNowWifiChannel));
  WiFi.begin("network-name", "pass-to-network", gEspNowWifiChannel);  // same channel approach as ELRS backpack
  WiFi.disconnect();

#if defined(WIFI_PROTOCOL_LR) && defined(CONFIG_IDF_TARGET_ESP32S3)
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
#else
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
#endif

  uint8_t factoryMac[6] = {};
  esp_wifi_get_mac(WIFI_IF_STA, factoryMac);
  memcpy(kBackpackUid, factoryMac, sizeof(kBackpackUid));
  kBackpackUid[0] &= 0xFE;

  uint8_t localMac[6];
  memcpy(localMac, kBackpackUid, 6);
  localMac[0] &= 0xFE;
  memcpy(kResolvedUidMac, localMac, sizeof(kResolvedUidMac));
  esp_wifi_set_mac(WIFI_IF_STA, localMac);
  esp_wifi_set_channel(gEspNowWifiChannel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {
      delay(1000);
    }
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, localMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add unicast peer");
    while (true) {
      delay(1000);
    }
  }

  esp_now_peer_info_t bindPeer = {};
  memcpy(bindPeer.peer_addr, kBroadcastMac, 6);
  bindPeer.channel = 0;
  bindPeer.encrypt = false;
  esp_now_add_peer(&bindPeer);

  Serial.printf("Backpack UID/MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", localMac[0], localMac[1], localMac[2], localMac[3],
                localMac[4], localMac[5]);
}

void sendRX5808Bit(uint8_t bitValue) {
  digitalWrite(kRx5808DataPin, bitValue ? HIGH : LOW);
  delayMicroseconds(300);
  digitalWrite(kRx5808ClkPin, HIGH);
  delayMicroseconds(300);
  digitalWrite(kRx5808ClkPin, LOW);
  delayMicroseconds(300);
}

void setupRX5808() {
  prepareRx5808BusIo();
  pinMode(kRssiInputPin, INPUT);
  analogSetAttenuation(ADC_11db);

  digitalWrite(kRx5808SelPin, HIGH);
  digitalWrite(kRx5808ClkPin, LOW);
  digitalWrite(kRx5808DataPin, LOW);
}

void setRX5808Frequency(uint16_t freqMhz) {
  prepareRx5808BusIo();
  // Formula and bus timing based on StarForgeOS timing_core.cpp (RotorHazard-compatible).
  const unsigned long elapsedBus = millis() - gLastRx5808BusMs;
  if (elapsedBus < kRx5808MinBusMs) {
    delay(kRx5808MinBusMs - elapsedBus);
  }

  const uint16_t tf = static_cast<uint16_t>((freqMhz - 479U) / 2U);
  const uint16_t N = static_cast<uint16_t>(tf / 32U);
  const uint16_t A = static_cast<uint16_t>(tf % 32U);
  const uint16_t regVal = static_cast<uint16_t>((N << 7) + A);

  digitalWrite(kRx5808SelPin, HIGH);
  digitalWrite(kRx5808SelPin, LOW);

  // Register 0x1 (frequency), LSB-first
  sendRX5808Bit(1);
  sendRX5808Bit(0);
  sendRX5808Bit(0);
  sendRX5808Bit(0);
  sendRX5808Bit(1);  // write flag

  for (uint8_t i = 0; i < 16; ++i) {
    sendRX5808Bit((regVal >> i) & 0x01);
  }

  // Padding bits
  sendRX5808Bit(0);
  sendRX5808Bit(0);
  sendRX5808Bit(0);
  sendRX5808Bit(0);

  digitalWrite(kRx5808SelPin, HIGH);
  delay(2);
  digitalWrite(kRx5808ClkPin, LOW);
  digitalWrite(kRx5808DataPin, LOW);

  gRecentFreqChange = true;
  gFreqChangeMs = millis();
  gLastRx5808BusMs = gFreqChangeMs;
}

void ensureRx5808Frequency(uint16_t freqMhz) {
  static uint16_t sLastRx5808FreqMhz = 0;
  if (sLastRx5808FreqMhz == freqMhz) {
    return;
  }
  setRX5808Frequency(freqMhz);
  sLastRx5808FreqMhz = freqMhz;
}

uint8_t readRawRssi() {
  if (gRecentFreqChange) {
    const unsigned long elapsed = millis() - gFreqChangeMs;
    if (elapsed < kRx5808MinTuneMs) {
      delay(kRx5808MinTuneMs - elapsed);
    }
    gRecentFreqChange = false;
  }

  uint16_t adc = analogRead(kRssiInputPin);
  if (adc > 2047) {
    adc = 2047;
  }
  return static_cast<uint8_t>(adc >> 3);
}

uint8_t readAveragedRssi(uint8_t samples) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    sum += readRawRssi();
  }
  return static_cast<uint8_t>(sum / samples);
}

bool detectRx5808Presence() {
  // Conservative detection:
  // disable scanning only when RSSI looks clearly invalid (stuck near 0 or full scale).
  const uint8_t probeIdx[3] = {0, 3, 7};  // R1, R4, R8
  uint8_t minRssi = 255;
  uint8_t maxRssi = 0;
  uint16_t sum = 0;

  for (uint8_t i = 0; i < 3; ++i) {
    setRX5808Frequency(kTimerChannels[probeIdx[i]].freqMhz);
    const uint8_t rssi = readAveragedRssi(8);
    if (rssi < minRssi) {
      minRssi = rssi;
    }
    if (rssi > maxRssi) {
      maxRssi = rssi;
    }
    sum += rssi;
  }

  const uint8_t avgRssi = static_cast<uint8_t>(sum / 3);
  const uint8_t rangeRssi = static_cast<uint8_t>(maxRssi - minRssi);
  const bool looksMissing = (maxRssi <= 2) || (minRssi >= 252) ||
                            ((rangeRssi <= 1) && ((avgRssi <= 4) || (avgRssi >= 250)));

  Serial.printf("RX5808 detect: min=%u max=%u avg=%u range=%u -> %s\n", minRssi, maxRssi, avgRssi, rangeRssi,
                looksMissing ? "NOT FOUND" : "FOUND");
  return !looksMissing;
}

void applyThresholdsFromReferenceRssi(uint8_t referenceRssi) {
  int enter = static_cast<int>(referenceRssi) + static_cast<int>(gEnterRssiOffset);
  int exit = static_cast<int>(referenceRssi) + static_cast<int>(gExitRssiOffset);
  enter = constrain(enter, static_cast<int>(kEnterThresholdMin), static_cast<int>(kEnterThresholdMax));
  exit = constrain(exit, static_cast<int>(kExitThresholdMin), static_cast<int>(kExitThresholdMax));
  if (exit >= enter) {
    exit = enter - 5;
    if (exit < static_cast<int>(kExitThresholdMin)) {
      exit = kExitThresholdMin;
    }
  }
  gEnterRssi = static_cast<uint8_t>(enter);
  gExitRssi = static_cast<uint8_t>(exit);
}

void resetPeakCaptureState() {
  gRssiPeak = 0;
  gRssiPeakTimeMs = 0;
  gBelowExitStreak = 0;
}

void resetLpRollingPeakState() {
  memset(gLpBucketMax, 0, sizeof(gLpBucketMax));
  memset(gLpBucketStartMs, 0, sizeof(gLpBucketStartMs));
  gLastLapRssiPeak = 0;
}

void updateLpRollingPeak(unsigned long now, uint8_t rssi) {
  const unsigned long bucketStart = now - (now % kLpBucketMs);
  const uint8_t bucketIdx = static_cast<uint8_t>((bucketStart / kLpBucketMs) % kLpBucketCount);
  if (gLpBucketStartMs[bucketIdx] != bucketStart) {
    gLpBucketStartMs[bucketIdx] = bucketStart;
    gLpBucketMax[bucketIdx] = rssi;
  } else if (rssi > gLpBucketMax[bucketIdx]) {
    gLpBucketMax[bucketIdx] = rssi;
  }

  uint8_t windowMax = 0;
  for (uint8_t i = 0; i < kLpBucketCount; ++i) {
    const unsigned long age = now - gLpBucketStartMs[i];
    if (age < kLpWindowMs && gLpBucketMax[i] > windowMax) {
      windowMax = gLpBucketMax[i];
    }
  }
  gLastLapRssiPeak = windowMax;
}

void resetRaceLapStateForNewRace() {
  // Start a fresh race while preserving session-level history/stats.
  gRaceNeedsFirstGateCalibration = true;
  gRaceStarted = false;
  gLastLapPeakMs = 0;
  gRaceStartMs = 0;
  gLapCount = 0;
  gLastLapMs = 0;
  resetLpRollingPeakState();
  gLapPopupUntilMs = 0;
  gBestLapBlinkStartMs = 0;
  gBestLapRaceMs = 0;
  gBest3ConsecutiveRaceMs = 0;
  gRecentRaceLapWindow[0] = 0;
  gRecentRaceLapWindow[1] = 0;
  gRecentRaceLapWindow[2] = 0;
  gRecentRaceLapWindowCount = 0;
  clearCurrentRaceLaps();
  resetPeakCaptureState();
}

void lockToChannel(uint8_t index, uint8_t lockRssi) {
  gVtxCalActive = false;
  gVtxCalPeakArmed = false;
  gLockedIndex = index;
  gBestScanRssi = lockRssi;
  gTimerMode = MODE_TIMING;
  // Seed thresholds now, then perform one-shot per-race calibration on first gate pass peak.
  applyThresholdsFromReferenceRssi(lockRssi);
  gRaceNeedsFirstGateCalibration = true;

  resetPeakCaptureState();
  gRaceStarted = false;
  resetLpRollingPeakState();
  gRaceStartMs = millis();
  gLockAcquiredMs = millis();
  gLastLapPeakMs = 0;
  gLockInfoUntilMs = millis() + kLockInfoDisplayMs;

  char lockMsg[40];
  snprintf(lockMsg, sizeof(lockMsg), "LOCK %s %uMHz", kTimerChannels[gLockedIndex].name, kTimerChannels[gLockedIndex].freqMhz);
  Serial.println(lockMsg);
  Serial.printf("Thresholds (seed): enter=%u exit=%u minLap=%lums\n", gEnterRssi, gExitRssi,
                static_cast<unsigned long>(gMinLapIntervalMs));
  Serial.println("First-pass calibration: next gate peak will set enter/exit once for this race");
  Serial.printf("OSD lock info for %lus: %s E=%u X=%u\n", static_cast<unsigned long>(kLockInfoDisplayMs / 1000UL),
                kTimerChannels[gLockedIndex].name, static_cast<unsigned>(gEnterRssi), static_cast<unsigned>(gExitRssi));
  // Force immediate OSD update so stale "SCAN ..." text is replaced on any lock path.
  char osdLockMsg[20];
  snprintf(osdLockMsg, sizeof(osdLockMsg), "LOCK %s", kTimerChannels[gLockedIndex].name);
  sendOsdMessage(osdLockMsg);
  sendLockedStatusOsd();
}

bool applyVtxRequestedChannel(uint8_t index) {
  if (index >= kTimerChannelCount) {
    return false;
  }
  if (!gRx5808Enabled) {
    static unsigned long sLastRxDisabledLogMs = 0;
    const unsigned long now = millis();
    if (now - sLastRxDisabledLogMs >= 2000UL) {
      sLastRxDisabledLogMs = now;
      Serial.println("VTX channel apply deferred: RX5808 is disabled");
    }
    return false;
  }
  if (gTimerMode == MODE_TIMING && gLockedIndex == index) {
    return true;
  }

  const RaceChannel &ch = kTimerChannels[index];
  // On every channel change we force a fresh calibration on the new channel,
  // but keep current lap history and statistics intact.
  Serial.println("Channel changed: preserving lap/best stats");

  gTimerMode = MODE_SCAN;
  gVtxCalActive = true;
  gVtxCalIndex = index;
  gVtxCalPeakArmed = false;
  gBestScanRssi = 0;
  // Leaving timing lock-info mode: redraw composed OSD with current requested channel.
  gLockInfoUntilMs = 0;
  gLastScanStepMs = 0;  // allow immediate first calibration sample
  Serial.printf("Applying VTX channel: %s (%u MHz), starting calibration (RSSI must be > %u)\n",
                ch.name, ch.freqMhz, gStrongSignalRssi);
  sendLockedStatusOsd();
  bool vrxSynced = false;
  if (sendVrxChannelIndex(index)) {
    vrxSynced = true;
    Serial.printf("Sent MSP_SET_VTX_CONFIG index=%u for %s\n",
                  static_cast<unsigned>(channelIndexToMspTableIndex48(index)), ch.name);
  } else {
    Serial.println("Failed to send MSP_SET_VTX_CONFIG for VRX channel sync");
  }
  if (gChannelSelectSource == CHANNEL_SELECT_SOURCE_AUX && index < 8) {
    if (sendVtxAdminChannelToElrs(index)) {
      Serial.printf("Sent VTX Admin channel update to goggles: %s (0x%02X)\n",
                    ch.name, static_cast<unsigned>(channelIndexToVtxAdminCode(index)));
      gVtxAdminSyncPending = false;
    } else {
      gVtxAdminSyncPending = true;
      gVtxAdminSyncPendingIdx = index;
      Serial.printf("VTX Admin sync pending for %s (template not ready yet)\n", ch.name);
    }
  }
  // Report apply success only when critical VRX sync packet was accepted by ESP-NOW.
  // If false, caller keeps gVtxChannelPending=true and retries automatically.
  return vrxSynced;
}

void processScan(unsigned long now) {
  static bool sAdminChannelWasAvailable = false;
  if (now - gLastScanStepMs < kScanStepMs) {
    return;
  }
  gLastScanStepMs = now;

  if (gVtxCalActive) {
    sAdminChannelWasAvailable = false;
    const bool armReady = isArmReadyForCalibration();
    const RaceChannel &ch = kTimerChannels[gVtxCalIndex];
    const uint8_t vtxCalExitRssi = static_cast<uint8_t>(
        constrain(static_cast<int>(gStrongSignalRssi) + static_cast<int>(gExitRssiOffset), 0, 255));
    ensureRx5808Frequency(ch.freqMhz);
    const uint8_t rssi = readAveragedRssi(6);
    gCurrentRssi = rssi;
    updateLpRollingPeak(now, rssi);
    if (rssi > gBestScanRssi) {
      gBestScanRssi = rssi;
    }

    // VTX calibration capture:
    // 1) Arm peak capture when RSSI rises above lock threshold.
    // 2) Keep highest peak.
    // 3) Lock only after RSSI falls below dynamic exit threshold
    //    (lock_threshold_rssi + exit_offset_rssi).
    if (armReady) {
      if (!gVtxCalPeakArmed) {
        if (rssi > gStrongSignalRssi) {
          gVtxCalPeakArmed = true;
          gBestScanRssi = rssi;
          Serial.printf("VTX-CAL peak armed on %s (%u MHz): RSSI=%u > %u, waiting drop <= %u\n",
                        ch.name, ch.freqMhz, rssi,
                        static_cast<unsigned>(gStrongSignalRssi),
                        static_cast<unsigned>(vtxCalExitRssi));
        }
      } else if (rssi <= vtxCalExitRssi) {
        const uint8_t lockRssi = (gBestScanRssi > rssi) ? gBestScanRssi : rssi;
        Serial.printf("VTX-CAL LOCK on %s (%u MHz), RSSI=%u (peak=%u, exit<=%u)\n",
                      ch.name, ch.freqMhz, rssi, lockRssi,
                      static_cast<unsigned>(vtxCalExitRssi));
        lockToChannel(gVtxCalIndex, lockRssi);
      }
    } else if (kLogScanDetails) {
      Serial.printf("VTX-CAL waiting ARM ON for %s (%u MHz), RSSI=%u\n", ch.name, ch.freqMhz, rssi);
    } else if (gLinkConnected && now - gLastOsdStatusMs >= kOsdStatusPeriodMs) {
      gLastOsdStatusMs = now;
      gOsdForceFullRefresh = true;
      sendLockedStatusOsd();
    }
    return;
  }

  // ADMIN source mode: never fall back to R1..R8 sweep.
  // If channel code is known, operate only on that channel; otherwise wait.
  if (gChannelSelectSource == CHANNEL_SELECT_SOURCE_ADMIN) {
    uint8_t forcedIdx = 0;
    const bool hasAdminChannel = tryGetAdminChannelIndex(forcedIdx);
    if (hasAdminChannel) {
      const RaceChannel &forced = kTimerChannels[forcedIdx];
      ensureRx5808Frequency(forced.freqMhz);
      const uint8_t rssi = readAveragedRssi(6);
      gCurrentRssi = rssi;
      updateLpRollingPeak(now, rssi);
      gBestScanRssi = rssi;
      if (gLinkConnected) {
        char chMsg[42];
        snprintf(chMsg, sizeof(chMsg), "CH %s RSSI %u", forced.name, rssi);
        sendOsdMessageIfChanged(chMsg);
        if (!sAdminChannelWasAvailable || now - gLastOsdStatusMs >= kOsdStatusPeriodMs) {
          gLastOsdStatusMs = now;
          gOsdForceFullRefresh = true;
          sendLockedStatusOsd();
        }
      }
      // Lock through the same peak-then-drop calibration as the AUX path,
      // instead of locking immediately, so ADMIN and AUX channel sources
      // behave identically. The VTX-CAL block at the top of processScan()
      // arms on RSSI rise, keeps the peak, waits for ARM, and locks once
      // RSSI drops below the dynamic exit threshold.
      gVtxCalActive = true;
      gVtxCalIndex = forcedIdx;
      gVtxCalPeakArmed = false;
      gBestScanRssi = 0;
      gLastScanStepMs = 0;  // allow immediate first calibration sample
    } else if (now - gLastOsdStatusMs >= kOsdStatusPeriodMs) {
      gLastOsdStatusMs = now;
      gOsdForceFullRefresh = true;
      sendLockedStatusOsd();
    }
    sAdminChannelWasAvailable = hasAdminChannel;
    return;
  }
  sAdminChannelWasAvailable = false;

  // AUX source mode: do not use fallback sweep; wait until AUX range maps to R1..R8.
  if (gChannelSelectSource == CHANNEL_SELECT_SOURCE_AUX) {
    if (now - gLastOsdStatusMs >= kOsdStatusPeriodMs) {
      gLastOsdStatusMs = now;
      gOsdForceFullRefresh = true;
      sendLockedStatusOsd();
    }
    return;
  }

  // No fallback sweep mode: channel selection is driven only by VTX Admin or AUX mapping.
  if (now - gLastOsdStatusMs >= kOsdStatusPeriodMs) {
    gLastOsdStatusMs = now;
    gOsdForceFullRefresh = true;
    sendLockedStatusOsd();
  }
}

void processTiming(unsigned long now) {
  if (now - gLastRssiSampleMs < kRssiSamplePeriodMs) {
    return;
  }
  gLastRssiSampleMs = now;

  const uint8_t rssi = readAveragedRssi(1);
  gCurrentRssi = rssi;
  updateLpRollingPeak(now, rssi);
  if (kLogTimingRssi && now - gLastRssiLogMs >= kRssiLogPeriodMs) {
    gLastRssiLogMs = now;
    Serial.printf("RSSI %s: %u (enter=%u exit=%u)\n", kTimerChannels[gLockedIndex].name, rssi, gEnterRssi, gExitRssi);
  }

  if (gArmSourceEnabled && gArmStateKnown) {
    if (!gArmActive) {
      if (gLastArmActive) {
        Serial.printf("ARM OFF (AUX%u): timing paused\n", static_cast<unsigned>(gArmAuxNumber));
        gLastDisarmMs = now;
        gLongDisarmLogged = false;
      }
      if (gLastDisarmMs > 0 && (now - gLastDisarmMs) >= gNewRaceAfterDisarmMs) {
        if (!gLongDisarmLogged) {
          Serial.println("DISARM gap reached: flushing lap logs to SD");
          gLongDisarmLogged = true;
        }
        flushPendingLapLogsToSd();
      }
      gLastArmActive = false;
      // Pause timing while disarmed, but keep lap progress state so short
      // disarm/rearm (e.g. turtle mode) can continue the same lap.
      resetPeakCaptureState();
      return;
    }
    if (!gLastArmActive) {
      Serial.printf("ARM ON (AUX%u): timing active\n", static_cast<unsigned>(gArmAuxNumber));
      bool startNewRace = false;
      if (gCurrentRaceNo == 0) {
        startNewRace = true;
      } else if (gLastDisarmMs > 0 && (now - gLastDisarmMs) >= gNewRaceAfterDisarmMs) {
        startNewRace = true;
      }
      if (startNewRace) {
        if (gCurrentRaceLapCount > 0) {
          archiveCurrentRaceLapsAsLastCompleted();
        }
        gRaceCount++;
        gCurrentRaceNo = gRaceCount;
        Serial.printf("New race #%lu (disarm gap rule: %lus)\n",
                      static_cast<unsigned long>(gCurrentRaceNo),
                      static_cast<unsigned long>(gNewRaceAfterDisarmMs / 1000UL));
        resetRaceLapStateForNewRace();
        gRaceStartMs = now;
      }
      gLongDisarmLogged = false;
      gLastArmActive = true;
      resetPeakCaptureState();
    }
  }

  // StarForge-style peak capture (without median filter):
  // 1) Track highest RSSI while above enter threshold.
  // 2) Consider peak captured only when RSSI falls below both peak and exit threshold.
  if (rssi >= gEnterRssi && rssi > gRssiPeak) {
    gRssiPeak = rssi;
    gRssiPeakTimeMs = now;
  }

  if (rssi <= gExitRssi) {
    if (gBelowExitStreak < 255) {
      gBelowExitStreak++;
    }
  } else {
    gBelowExitStreak = 0;
  }

  const bool exitedGateConfirmed = (rssi <= gExitRssi) && (gBelowExitStreak >= gExitConfirmSamples);
  const bool peakCaptured = (rssi < gRssiPeak) && exitedGateConfirmed;

  if (peakCaptured && gRssiPeak > 0) {
    if (!gRaceStarted) {
      if (gRaceNeedsFirstGateCalibration) {
        if (!isArmReadyForCalibration()) {
          Serial.println("Gate ignored for calibration: waiting ARM ON");
          resetPeakCaptureState();
          return;
        }
        if (gRssiPeak < gStrongSignalRssi) {
          Serial.printf("Gate ignored for calibration: peak %u < min %u\n",
                        static_cast<unsigned>(gRssiPeak),
                        static_cast<unsigned>(gStrongSignalRssi));
          resetPeakCaptureState();
          return;
        }
        applyThresholdsFromReferenceRssi(gRssiPeak);
        gRaceNeedsFirstGateCalibration = false;
        Serial.printf("Race first-pass calibration: peak=%u -> enter=%u exit=%u\n",
                      gRssiPeak, gEnterRssi, gExitRssi);
      }
      // First valid gate pass starts race timing from race-start reference.
      gRaceStarted = true;
      if (gLastLapPeakMs == 0) {
        uint32_t firstLapRefMs = gRaceStartMs;
        if (firstLapRefMs == 0) {
          firstLapRefMs = gLockAcquiredMs;
        }
        gLastLapPeakMs = (firstLapRefMs > 0) ? firstLapRefMs : gRssiPeakTimeMs;
      }
    }

    const uint32_t timeSinceLastLap = gRssiPeakTimeMs - gLastLapPeakMs;
    if (timeSinceLastLap >= gMinLapIntervalMs) {
      const uint32_t lapMs = timeSinceLastLap;
      // Ignore extremely long laps (>60s by default via kCfgCooldownMaxMs):
      // keep timing reference in sync, but do not count/store/display this lap.
      if (lapMs > kCfgCooldownMaxMs) {
        gLastLapPeakMs = gRssiPeakTimeMs;
        Serial.printf("Lap hidden (too long): %lums > %lums\n", static_cast<unsigned long>(lapMs),
                      static_cast<unsigned long>(kCfgCooldownMaxMs));
      } else {
        if (isLikelyFakeLap(lapMs)) {
          gLastLapPeakMs = gRssiPeakTimeMs;
          const uint32_t sec = lapMs / 1000U;
          const uint32_t cs = (lapMs % 1000U) / 10U;
          Serial.printf("Lap ignored as outlier: %02lu.%02lu (fake-lap filter)\n",
                        static_cast<unsigned long>(sec), static_cast<unsigned long>(cs));
          if (gLinkConnected) {
            char msg[24];
            snprintf(msg, sizeof(msg), "IGN %02lu.%02lu",
                     static_cast<unsigned long>(sec), static_cast<unsigned long>(cs));
            sendOsdMessageIfChanged(msg);
          }
          resetPeakCaptureState();
          return;
        }

        gLastLapPeakMs = gRssiPeakTimeMs;
        gLapCount++;
        gSessionLapCount++;
        gLastLapMs = lapMs;
        if (gCurrentRaceNo == 0) {
          gCurrentRaceNo = 1;
          if (gRaceCount == 0) {
            gRaceCount = 1;
          }
        }
        pushLapHistory(lapMs);
        appendCurrentRaceLap(lapMs);
        // SF tracks the fastest lap within the rolling last-100 window: a glitch
        // lap that slips through early drops out once 100 newer laps accumulate.
        const uint32_t prevWindowBestMs = computeSessionWindowBestLapMs();  // before adding this lap
        appendSessionLapForBestHalfAverage(lapMs);
        const bool isNewBestLap = (prevWindowBestMs == 0) || (lapMs < prevWindowBestMs);
        gBestLapSessionMs = computeSessionWindowBestLapMs();  // min over last 100 (incl. this lap)
        if (gBestLapRaceMs == 0 || lapMs < gBestLapRaceMs) {
          gBestLapRaceMs = lapMs;  // RF: fastest of current race (unchanged)
        }
        const unsigned long popupNow = millis();
        if (isNewBestLap) {
          const unsigned long blinkToggles = static_cast<unsigned long>(kBestLapBlinkCount) * 2UL;
          const unsigned long blinkWindowMs = kBestLapBlinkToggleMs * blinkToggles;
          gLapPopupUntilMs = popupNow + blinkWindowMs + kBestLapPostBlinkHoldMs;
          gBestLapBlinkStartMs = popupNow;
        } else {
          gLapPopupUntilMs = popupNow + kLapPopupDisplayMs;
          gBestLapBlinkStartMs = 0;
        }
        // Once we have a real lap, stop lock-info placeholders so OSD stats update immediately.
        gLockInfoUntilMs = 0;
        appendLapToSd(gLapCount, lapMs, isNewBestLap);

        const uint32_t sec = lapMs / 1000U;
        const uint32_t ms = lapMs % 1000U;
        Serial.printf("LAP %u: %02lu.%03lus (peak=%u, lpPeak=%u, exitStreak=%u)\n", gLapCount, static_cast<unsigned long>(sec),
                      static_cast<unsigned long>(ms), gRssiPeak, gLastLapRssiPeak, gBelowExitStreak);
        if (gBestHalfSessionAvgMs > 0) {
          const uint32_t avgSec = gBestHalfSessionAvgMs / 1000U;
          const uint32_t avgCs = (gBestHalfSessionAvgMs % 1000U) / 10U;
          const int32_t deltaMs = static_cast<int32_t>(lapMs) - static_cast<int32_t>(gBestHalfSessionAvgMs);
          const uint32_t deltaAbsMs = static_cast<uint32_t>(deltaMs < 0 ? -deltaMs : deltaMs);
          const uint32_t deltaSec = deltaAbsMs / 1000U;
          const uint32_t deltaCs = (deltaAbsMs % 1000U) / 10U;
          const char deltaSign = (deltaMs < 0) ? '-' : '+';
          Serial.printf("TOP50 AVG: %02lu.%02lu  DELTA: %c%02lu.%02lu\n",
                        static_cast<unsigned long>(avgSec), static_cast<unsigned long>(avgCs),
                        deltaSign, static_cast<unsigned long>(deltaSec), static_cast<unsigned long>(deltaCs));
        }
        if (isNewBestLap) {
          Serial.println("New BEST lap: popup blinking for 5s");
        }
        sendLockedStatusOsd();
      }
    } else {
      // Keep lap reference synced to the latest confirmed gate peak so the next
      // accepted lap does not accumulate time from race start or old peaks.
      gLastLapPeakMs = gRssiPeakTimeMs;
      if (kLogLapRejects) {
        Serial.printf("Lap rejected (cooldown): %lums < %lums\n", static_cast<unsigned long>(timeSinceLastLap),
                      static_cast<unsigned long>(gMinLapIntervalMs));
      }
    }

    // Always clear captured peak so stale peaks cannot be recorded later.
    resetPeakCaptureState();
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Boot: backpack + R-scan + lap timer");
  Serial.printf("Pins: RSSI=%d DATA=%d CLK=%d SEL=%d\n", kRssiInputPin, kRx5808DataPin, kRx5808ClkPin, kRx5808SelPin);
  Serial.printf("SD SPI pins: CS=%d SCK=%d MISO=%d MOSI=%d\n", kSdSpiCsPin, kSdSpiSckPin, kSdSpiMisoPin, kSdSpiMosiPin);
  Serial1.begin(kElrsBaud, SERIAL_8N1, kElrsRxPin, kElrsTxPin);
  Serial.printf("ELRS CRSF UART: RX=%d TX=%d baud=%lu\n", kElrsRxPin, kElrsTxPin, static_cast<unsigned long>(kElrsBaud));
  setupSdLogging();
  applyUserSettings();
  runConfigSelfTest();
  Serial.printf("Settings: OSD(%u,%u) popup(%u,%u) lock=%u enterOfs=%d exitOfs=%d cooldown=%lums exitConfirm=%u\n",
                gCfgOsdMainRow, gCfgOsdMainCol, gCfgLapPopupRow, gCfgLapPopupCol,
                gStrongSignalRssi, static_cast<int>(gEnterRssiOffset), static_cast<int>(gExitRssiOffset),
                static_cast<unsigned long>(gMinLapIntervalMs), static_cast<unsigned>(gExitConfirmSamples));
  Serial.printf("OSD elements: CH(%u,%u) RSSI(%u,%u) LPR(%u,%u) THU(%u,%u) THL(%u,%u) BEST(%u,%u) FR(%u,%u) B3(%u,%u) B3R(%u,%u) RACE_LAPS(%u,%u) POP(%u,%u)\n",
                gCfgOsdChannelCol, gCfgOsdChannelRow, gCfgOsdRssiCol, gCfgOsdRssiRow,
                gCfgOsdLapPeakRssiCol, gCfgOsdLapPeakRssiRow,
                gCfgOsdRssiThrUpperCol, gCfgOsdRssiThrUpperRow, gCfgOsdRssiThrLowerCol, gCfgOsdRssiThrLowerRow,
                gCfgOsdBestLapCol, gCfgOsdBestLapRow, gCfgOsdBestLapRaceCol, gCfgOsdBestLapRaceRow,
                gCfgOsdBest3Col, gCfgOsdBest3Row,
                gCfgOsdBest3RaceCol, gCfgOsdBest3RaceRow,
                gCfgOsdRaceLapsCol, gCfgOsdRaceLapsRow, gCfgLapPopupCol, gCfgLapPopupRow);
  Serial.printf("OSD showDuringRace: CH=%u RSSI=%u LPR=%u THU=%u THL=%u SF=%u RF=%u S3=%u R3=%u LAPS=%u POP=%u\n",
                static_cast<unsigned>(gCfgOsdChannelShowDuringRace),
                static_cast<unsigned>(gCfgOsdRssiShowDuringRace),
                static_cast<unsigned>(gCfgOsdLapPeakRssiShowDuringRace),
                static_cast<unsigned>(gCfgOsdRssiThrUpperShowDuringRace),
                static_cast<unsigned>(gCfgOsdRssiThrLowerShowDuringRace),
                static_cast<unsigned>(gCfgOsdBestLapShowDuringRace),
                static_cast<unsigned>(gCfgOsdBestLapRaceShowDuringRace),
                static_cast<unsigned>(gCfgOsdBest3ShowDuringRace),
                static_cast<unsigned>(gCfgOsdBest3RaceShowDuringRace),
                static_cast<unsigned>(gCfgOsdRaceLapsShowDuringRace),
                static_cast<unsigned>(gCfgLapPopupShowDuringRace));
  if (gChannelSelectSource == CHANNEL_SELECT_SOURCE_ADMIN) {
    Serial.println("Channel select source: ADMIN");
  } else {
    Serial.printf("Channel select source: AUX%u\n", static_cast<unsigned>(gAuxSelectNumber));
  }
  if (!gArmSourceEnabled) {
    Serial.println("ARM source: NONE");
  } else {
    Serial.printf("ARM source: AUX%u (%u-%u us)\n", static_cast<unsigned>(gArmAuxNumber),
                  static_cast<unsigned>(gArmActiveMinUs), static_cast<unsigned>(gArmActiveMaxUs));
  }

  setupEspNowWithBackpackUid();
  if (gCfgRx5808ModeSelect == 2) {
    gRx5808Enabled = false;
    Serial.println("Auto-bind active (RX5808 force off)");
  } else {
    setupRX5808();
    if (gCfgRx5808ModeSelect == 1) {
      gRx5808Enabled = true;
      Serial.println("RX5808 mode: force on");
    } else {
      gRx5808Enabled = detectRx5808Presence();
      Serial.printf("RX5808 mode: auto (%s)\n", gRx5808Enabled ? "enabled" : "disabled");
    }

    if (gRx5808Enabled) {
      // Keep RX initialized; channel selection is driven by VTX Admin/AUX source.
      setRX5808Frequency(kTimerChannels[0].freqMhz);
      Serial.println("Auto-bind active, waiting for channel source (VTX Admin/AUX)...");
    } else {
      Serial.println("Auto-bind active (RX5808 not detected, scan disabled)");
    }
  }
}

void loop() {
  static unsigned long lastBind = 0;
  static unsigned long lastProbe = 0;
  static bool wasConnected = false;
  const unsigned long now = millis();
#if defined(CONFIG_IDF_TARGET_ESP32S3) && SOC_USB_OTG_SUPPORTED && !ARDUINO_USB_MODE
  if (gUsbMscHostActive) {
    // While USB mass storage is active, avoid normal timer processing to prevent
    // SD/SPI contention and improve host filesystem stability.
    delay(5);
    return;
  }
#endif

  if (gProbeInFlight && now - gProbeStartedMs > kProbeTimeoutMs) {
    gProbeInFlight = false;
    if (gPendingTxKind == TXK_PROBE_UNICAST) {
      gPendingTxKind = TXK_NONE;
    }
  }
  if (gTxAwaiting && now - gTxStartedMs > kTxTimeoutMs) {
    gTxAwaiting = false;
    if (gPendingTxKind == TXK_PROBE_UNICAST) {
      gProbeInFlight = false;
    }
    gPendingTxKind = TXK_NONE;
  }

  if (gLinkConnected && now - gLastLinkSeenMs > kLinkTimeoutMs) {
    gLinkConnected = false;
    Serial.println("Link lost: auto-bind restarted");
  }

  if (gLinkConnected && !wasConnected) {
    wasConnected = true;
    Serial.println("Link connected");
    gOsdForceFullRefresh = true;
    if (gBootOsdClearPending) {
      clearOsdElementsOnBoot();
      gBootOsdClearPending = false;
      Serial.println("Boot OSD clear sent");
    }
    gCrsfNoDataWarningShown = false;
    if (gTimerMode == MODE_SCAN && gRx5808Enabled) {
      uint8_t idx = 0;
      if (gChannelSelectSource == CHANNEL_SELECT_SOURCE_ADMIN &&
          tryGetAdminChannelIndex(idx)) {
        Serial.printf("Link connected: resume VTX Admin channel %s\n", kTimerChannels[idx].name);
        if (!applyVtxRequestedChannel(idx)) {
          gVtxChannelPendingIdx = idx;
          gVtxChannelPending = true;
        }
      } else if (gChannelSelectSource == CHANNEL_SELECT_SOURCE_AUX) {
        if (gLastAuxRequestedIdx >= 0 && gLastAuxRequestedIdx < 8) {
          const uint8_t auxIdx = static_cast<uint8_t>(gLastAuxRequestedIdx);
          Serial.printf("Link connected: reapply AUX%u mapped channel %s\n",
                        static_cast<unsigned>(gAuxSelectNumber), kTimerChannels[auxIdx].name);
          if (!applyVtxRequestedChannel(auxIdx)) {
            gVtxChannelPendingIdx = auxIdx;
            gVtxChannelPending = true;
          }
        } else {
          Serial.printf("Link connected: waiting AUX%u range mapping\n", static_cast<unsigned>(gAuxSelectNumber));
        }
      }
    } else if (!gRx5808Enabled) {
      Serial.println("Link connected (RX5808 unavailable, scan remains disabled)");
    }
  } else if (!gLinkConnected && wasConnected) {
    wasConnected = false;
  }

  if (!gLinkConnected && !gProbeInFlight && !gTxAwaiting && now - lastBind >= kBindIntervalMs) {
    lastBind = now;
    sendBindBroadcast();
  }

  const unsigned long probeInterval = gLinkConnected ? kProbeIntervalConnectedMs : kProbeIntervalDisconnectedMs;
  if (!gProbeInFlight && !gTxAwaiting && now - lastProbe >= probeInterval) {
    lastProbe = now;
    sendLinkProbe();
  }

  static unsigned long lastVtxApplyAttemptMs = 0;
  if (gVtxChannelPending && (now - lastVtxApplyAttemptMs >= 200UL)) {
    lastVtxApplyAttemptMs = now;
    const uint8_t idx = gVtxChannelPendingIdx;
    if (applyVtxRequestedChannel(idx)) {
      gVtxChannelPending = false;
    }
  }

  if (gTimerMode == MODE_SCAN) {
    if (gRx5808Enabled) {
      processScan(now);
    } else if (gLinkConnected && !gRx5808Enabled && now - gLastOsdStatusMs >= kOsdStatusPeriodMs) {
      gLastOsdStatusMs = now;
      sendOsdMessageIfChanged("LINK OK (RX5808 OFF)");
    }
  } else {
    processTiming(now);
    const bool blinkPopupActive = (gBestLapBlinkStartMs != 0) && timerActive(now, gLapPopupUntilMs);
    if (blinkPopupActive) {
      static unsigned long sLastPopupBlinkOsdMs = 0;
      if (now - sLastPopupBlinkOsdMs >= 100UL) {
        sLastPopupBlinkOsdMs = now;
        sendLapPopupBlinkOnly(now);
      }
    } else if (now - gLastOsdStatusMs >= kOsdStatusPeriodMs) {
      gLastOsdStatusMs = now;
      gOsdForceFullRefresh = true;
      sendLockedStatusOsd();
    }
  }

  // ELRS CRSF monitor: parse frames and react to AUX/VTX events.
  while (Serial1.available() > 0) {
    const int raw = Serial1.read();
    if (raw < 0) {
      break;
    }
    feedCrsfByte(static_cast<uint8_t>(raw));
  }

  if (gLinkConnected && gChannelSelectSource == CHANNEL_SELECT_SOURCE_AUX &&
      !gCrsfChannelsInitialized && !gCrsfNoDataWarningShown &&
      now - gLastLinkSeenMs >= kCrsfRcWarnMs) {
    gCrsfNoDataWarningShown = true;
    Serial.printf("No CRSF RC frames on UART yet. Check ELRS wiring: external TX -> GPIO%d, RX -> GPIO%d, GND common, baud %lu\n",
                  kElrsRxPin, kElrsTxPin, static_cast<unsigned long>(kElrsBaud));
  }

  yield();
}

