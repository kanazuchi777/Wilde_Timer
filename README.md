# Wilde Timer Firmware

This documentation reflects the current implementation in `src/wilde_timer_firmware/main.cpp`.
If README and code ever differ, treat the code as the source of truth.

## Project Structure

- `src/wilde_timer_firmware/main.cpp` - full runtime logic (ESP-NOW, CRSF, RX5808, OSD, lap timing, SD)
- `src/wilde_timer_firmware/modules/parse_utils.h/.cpp` - config parsing and validation helpers
- `platformio.ini` - build environments (`esp32D`, `esp32-s3-devkitc-1`)

## Build and Flash

- ESP32 DevKit (`esp32D`):
  - `pio run -e esp32D -t upload`
- ESP32-S3 DevKitC:
  - `pio run -e esp32-s3-devkitc-1 -t upload`
- Serial monitor:
  - `pio device monitor -b 115200`

## Hardware Pinout (from code)

### ESP32-S3

- RX5808:
  - `RSSI -> GPIO4`
  - `DATA -> GPIO10`
  - `SEL -> GPIO11`
  - `CLK -> GPIO12`
- SD (SPI):
  - `CS -> GPIO39`
  - `SCK -> GPIO36`
  - `MISO -> GPIO37`
  - `MOSI -> GPIO35`
- ELRS/CRSF UART:
  - `RX (from external TX) -> GPIO16`
  - `TX (to external RX) -> GPIO17`
  - baud: `420000`

### ESP32D (`esp32dev`)

- RX5808:
  - `RSSI -> GPIO34`
  - `DATA -> GPIO23`
  - `CLK -> GPIO18`
  - `SEL -> GPIO5`
- SD (SPI):
  - `CS -> GPIO33`
  - `SCK -> GPIO25`
  - `MISO -> GPIO27`
  - `MOSI -> GPIO26`
- ELRS/CRSF UART:
  - `RX <- GPIO16`
  - `TX -> GPIO17`
  - baud: `420000`

## Runtime Model

### 1. Link and bind

- Uses ESP-NOW with auto-bind broadcast (`MSP_ELRS_BIND`).
- Sends periodic probe (`MSP_ELRS_SET_OSD` display opcode).
- If no successful unicast traffic is seen within `kLinkTimeoutMs`, link is considered lost and auto-bind restarts.

### 2. Channel source

Two supported sources:

- `ADMIN` (CRSF VTX ADMIN)
- `AUXx` (AUX range mapped to `R1..R8`)

Behavior:

- `ADMIN` mode does not do fallback R1..R8 sweep.
- `AUX` mode uses only configured AUX ranges.
- When channel changes, session stats are preserved, but calibration is restarted for the new channel.

### 3. Scan -> Lock -> Timing

- Scan reads RSSI on selected channel.
- Lock requires `RSSI > lock_threshold_rssi` and ARM-ready state.
- After lock:
  - `enter/exit` thresholds are derived
  - `post_lock_ignore_ms` is enforced
- First valid gate pass only arms timing reference and is not counted as a lap.

### 4. Lap detection

Gate pass logic:

- track RSSI peak above `enter`
- lap closes only after RSSI drops below `exit` for `exit_confirm_below_samples`
- enforce minimum interval (`min_lap_interval_ms`)
- hide/ignore very long laps (`> kCfgCooldownMaxMs`, default 60000 ms)

Outlier (fake lap) filter:

- uses median from up to last 7 laps (`gAllLapHistoryMs`)
- rejects overly fast anomalies using absolute and ratio floor checks
- rejected laps are not added to race/session stats

### 5. Race and session

- `gLapCount` - laps in current race
- `gSessionLapCount` - laps across current session
- Long disarm (`new_race_after_disarm_ms`) starts a new race on next ARM
- `S3` - best `R3` over the whole session
- `R3` - best rolling 3-lap sum for current race

### 6. Lap popup delta metric

Popup includes lap delta against a dynamic baseline:

- take last 100 valid session laps
- sort by lap time
- take fastest 50
- average those 50
- display `lastLap - avgTop50(last100)`

Popup format:

- `Lxx ss.cc +dd.cc` or `Lxx ss.cc -dd.cc`

Meaning:

- `+` = slower than baseline
- `-` = faster than baseline

## OSD

### Main OSD elements

- `gOsdChannel`
- `gOsdRssi`
- `gOsdRssiThrUpper`
- `gOsdRssiThrLower`
- `gOsdBestLap` (SF)
- `gOsdBestLap_race` (RF)
- `gOsdBest3` (S3)
- `gOsdBest3_race` (R3)
- `raceLaps` (column `L1`, `L2`, ...)
- `gOsdLapPopup`

`WAIT VTX ADMIN` is internal and is not configured via `[col,row,flag]`.

### OSD config format

All OSD position keys support:

- `[col,row,showDuringRace]`

Examples:

- `gOsdRssi=[12,16,1]`
- `gOsdBest3_race=[38,2,0]`

`showDuringRace`:

- `1` = show while race is active
- `0` = hide while race is active

Notes:

- Legacy `[col,row]` format is still supported.
- If 3rd parameter is missing, firmware treats that as missing config and rewrites `/config.txt` with 3-parameter format.
- Default for all `showDuringRace` flags is `1`.

### `raceLaps` rendering

- During race: renders current race laps (if enabled by `showDuringRace`)
- Outside race: if current race list is empty, may show last completed race laps
- If more rows than available OSD height, renders a tail window (latest visible rows)

## SD Behavior

- Config file: `/config.txt`
- Lap log CSV: `/LOGS/laps.csv`
- Laps are buffered in RAM, then flushed to SD
- If SD becomes unavailable during runtime:
  - firmware attempts recovery
  - on failure, SD lap logging is disabled until reboot (prevents error spam)

CSV columns:

- `boot_ms,race_no,lap_no,lap_ms,channel,is_new_best`

## USB MSC (ESP32-S3 only)

- If SD is ready and USB MSC is enabled, SD is exposed as USB mass storage.
- While host MSC is active, timer main loop is paused to avoid SD/SPI contention.

## `/config.txt` Keys

### OSD positions

- `gOsdChannel=[col,row,showDuringRace]`
- `gOsdRssi=[col,row,showDuringRace]`
- `gOsdRssiThrUpper=[col,row,showDuringRace]`
- `gOsdRssiThrLower=[col,row,showDuringRace]`
- `gOsdBestLap=[col,row,showDuringRace]`
- `gOsdBestLap_race=[col,row,showDuringRace]`
- `gOsdBest3=[col,row,showDuringRace]`
- `gOsdBest3_race=[col,row,showDuringRace]`
- `raceLaps=[col,row,showDuringRace]`
- `gOsdLapPopup=[col,row,showDuringRace]`

Also supported:

- `osd_main_row`, `osd_main_col`
- `lap_popup_row`, `lap_popup_col` (legacy popup position keys)

### Timing and RSSI

- `lock_threshold_rssi`
- `enter_offset_rssi`
- `exit_offset_rssi`
- `min_lap_interval_ms`
- `post_lock_ignore_ms`
- `exit_confirm_below_samples`

### RX5808, channel source, ARM

- `rx5808_mode_select` (`0=AUTO`, `1=FORCE ON`, `2=FORCE OFF`)
- `channel_select_source` (`ADMIN` or `AUX1..AUX12`)
- `aux_range_r1..aux_range_r8` (microsecond ranges mapped to `R1..R8`)
- `arm_source` (`NONE` or `AUX1..AUX12`)
- `arm_active_min_us`
- `arm_active_max_us`
- `new_race_after_disarm_ms`

### SD

- `sd_lap_logging_enabled` (`0/1`, `true/false`, `on/off`)

## Exact Code Defaults

Defaults in current `main.cpp`:

- `osd_main_row=17`
- `osd_main_col=13`
- `lap_popup_row=12`
- `lap_popup_col=18`
- `gOsdChannel=[10,17,1]`
- `gOsdRssi=[12,16,1]`
- `gOsdRssiThrUpper=[18,16,1]`
- `gOsdRssiThrLower=[25,16,1]`
- `gOsdBestLap=[13,17,1]`
- `gOsdBestLap_race=[22,17,1]`
- `gOsdBest3=[31,17,1]`
- `gOsdBest3_race=[38,2,1]`
- `raceLaps=[38,3,1]`
- `gOsdLapPopup=[18,12,1]`
- `lock_threshold_rssi=100`
- `enter_offset_rssi=-15`
- `exit_offset_rssi=-45`
- `min_lap_interval_ms=8000`
- `post_lock_ignore_ms=6000`
- `exit_confirm_below_samples=4`
- `rx5808_mode_select=0`
- `sd_lap_logging_enabled=1`
- `channel_select_source=ADMIN`
- `aux_range_r1=1540-1560`
- `aux_range_r2=1565-1620`
- `aux_range_r3=1625-1660`
- `aux_range_r4=1665-1720`
- `aux_range_r5=1725-1760`
- `aux_range_r6=1765-1820`
- `aux_range_r7=1825-1860`
- `aux_range_r8=1860-1920`
- `arm_source=AUX1`
- `arm_active_min_us=1700`
- `arm_active_max_us=2100`
- `new_race_after_disarm_ms=10000`

Auto-clamp ranges:

- OSD `row`: `0..17`
- OSD `col`: `0..50`
- `lock_threshold_rssi`: `60..230`
- `enter_offset_rssi`, `exit_offset_rssi`: `-60..60`
- `min_lap_interval_ms`: `1000..60000`
- `post_lock_ignore_ms`: `0..30000`
- `exit_confirm_below_samples`: `1..20`
- `new_race_after_disarm_ms`: `0..300000`

## Useful Serial Diagnostics

On boot, check:

- `Settings: ...`
- `OSD elements: ...`
- `OSD showDuringRace: ...`
- `Channel select source: ...`
- `ARM source: ...`
- `RX5808 ...`

During laps, check:

- `LAP ...`
- `TOP50 AVG ... DELTA ...`
- `Lap ignored as outlier ...` (if outlier filter triggers)

## Not used in this firmware

No active logic found for:

- buzzer
- WS2812 LED
- battery voltage input
- separate mode switch input
