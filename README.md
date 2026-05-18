# Wilde Timer Wiring (from current firmware code)

This README is based on the actual pins used in:
`src/wilde_timer_firmware/main.cpp`

If code and wiring doc ever differ, trust the code.

## Source structure

- `src/wilde_timer_firmware/main.cpp`: main runtime loop and feature integration
- `src/wilde_timer_firmware/modules/parse_utils.h/.cpp`: shared parsing/validation helpers used by config loader and runtime parsers

This split keeps behavior unchanged while reducing `main.cpp` size and preparing for further modularization.

## Build / Flash

1. ESP32-S3:
   - `pio run -e esp32-s3-devkitc-1 -t upload`
2. ESP32D:
   - `pio run -e esp32D -t upload`
3. Serial monitor:
   - `pio device monitor -b 115200`

## How timer works

1. Boot:
   - ESP32 starts ESP-NOW backpack link and ELRS/CRSF UART listener.
   - RX5808 presence is checked (unless forced by config mode).
2. Channel source:
   - Channel can come from `VTX ADMIN` or from `AUX` mapping (`R1..R8` ranges).
   - In `AUX` mode, if AUX value is outside configured ranges, OSD shows `AUXx WAIT`.
   - In `AUX` mode, when timer channel changes, firmware also sends VTX ADMIN channel update back over ELRS/CRSF UART so goggles can follow the same channel.
3. Lock/calibration:
   - For selected channel, device watches RSSI.
   - When RSSI is above lock threshold, timer locks that channel and seeds `enter/exit` thresholds.
4. Lap detection:
   - A gate pass is detected from RSSI peak logic:
     above `enter`, then drop below `exit` for required consecutive samples.
   - Cooldown blocks too-fast false laps (`min lap interval`).
5. Stats/output:
   - Laps update OSD and serial output.
   - If SD is available, valid laps are buffered and written to `/LOGS/laps.csv`.
6. Channel change behavior:
   - When channel is changed (AUX or VTX ADMIN), session lap history and best-session stats are preserved.
   - A fresh calibration is started for the new channel, and race-local timing state is re-armed.

## Soldering: ESP32-S3

### RX5808 receiver
- RX5808 `RSSI` -> ESP32-S3 `GPIO4`
- RX5808 `DATA` (CH1) -> ESP32-S3 `GPIO10`
- RX5808 `SEL/LE` (CH2) -> ESP32-S3 `GPIO11`
- RX5808 `CLK` (CH3) -> ESP32-S3 `GPIO12`
- RX5808 `VCC` -> board `3.3V` (or module-required supply)
- RX5808 `GND` -> board `GND`

### microSD (SPI)
- SD `CS` -> ESP32-S3 `GPIO39`
- SD `SCK/CLK` -> ESP32-S3 `GPIO36`
- SD `MISO` -> ESP32-S3 `GPIO37`
- SD `MOSI` -> ESP32-S3 `GPIO35`
- SD `VCC` -> board `3V3` (recommended)
- SD `GND` -> board `GND`

### External ELRS / CRSF UART
- External device `TX` -> ESP32-S3 `GPIO16` (board UART RX)
- External device `RX` -> ESP32-S3 `GPIO17` (board UART TX, optional but recommended)
- External device `GND` -> board `GND`
- External device `VCC` -> correct supply for that device
- UART speed in code: `420000`

## Soldering: ESP32D (`esp32dev`)

### RX5808 receiver
- RX5808 `RSSI` -> ESP32D `GPIO34` (input-only ADC)
- RX5808 `DATA` -> ESP32D `GPIO23`
- RX5808 `CLK` -> ESP32D `GPIO18`
- RX5808 `SEL/LE` -> ESP32D `GPIO5`
- RX5808 `VCC` -> board `3.3V` (or module-required supply)
- RX5808 `GND` -> board `GND`

### microSD (SPI)
- SD `CS` -> ESP32D `GPIO33`
- SD `SCK/CLK` -> ESP32D `GPIO25`
- SD `MISO` -> ESP32D `GPIO27`
- SD `MOSI` -> ESP32D `GPIO26`
- SD `VCC` -> board `3V3` (recommended)
- SD `GND` -> board `GND`

### External ELRS / CRSF UART
- External device `TX` -> ESP32D `GPIO16` (board UART RX)
- External device `RX` -> ESP32D `GPIO17` (board UART TX, optional but recommended)
- External device `GND` -> board `GND`
- External device `VCC` -> correct supply for that device
- UART speed in code: `420000`

## Important checks before power-on

- UART is crossed: `device TX -> board RX`, `device RX -> board TX`.
- Do not feed `5V` UART logic directly to ESP32 pins.
- If RX5808 `RSSI` can exceed `3.3V`, use a divider before ESP32 ADC pin.
- All modules must share common `GND`.

## SD card over USB (ESP32-S3)

- ESP32-S3 firmware now exposes SD card as USB MSC (mass storage) when SD is ready.
- Connect ESP32-S3 to PC with USB data cable.
- On boot, serial should print `USB MSC ready (read/write)`.
- PC should show a removable drive with SD card content.
- While USB MSC is active, timer runtime loop is paused to avoid SD/SPI contention and improve filesystem stability.

## SD config file (`/config.txt`)

- Location: SD card root, file path `/config.txt`
- On first boot with SD, firmware auto-creates this file with defaults.
- Lines starting with `#` or `;` are treated as comments.
- Unknown keys are ignored.
- If some keys are missing, firmware auto-fills missing keys on next save.

Example:

```ini
osd_main_row=17
osd_main_col=15
lap_popup_row=12
lap_popup_col=20
lock_threshold_rssi=90
enter_offset_rssi=-10
exit_offset_rssi=-50
min_lap_interval_ms=10000
post_lock_ignore_ms=6000
exit_confirm_below_samples=4
rx5808_mode_select=0
sd_lap_logging_enabled=1
channel_select_source=AUX7
aux_range_r1=1000-1124
aux_range_r2=1125-1249
aux_range_r3=1250-1374
aux_range_r4=1375-1499
aux_range_r5=1500-1624
aux_range_r6=1625-1749
aux_range_r7=1750-1874
aux_range_r8=1875-2100
arm_source=AUX1
arm_active_min_us=1700
arm_active_max_us=2100
new_race_after_disarm_ms=20000
```

Key meanings:

- `osd_main_row`, `osd_main_col`: main OSD text position
- `lap_popup_row`, `lap_popup_col`: last-lap popup position
- `lock_threshold_rssi`: RSSI threshold used for channel lock
- `enter_offset_rssi`, `exit_offset_rssi`: enter/exit offsets from lock reference RSSI
- `min_lap_interval_ms`: minimum lap time (anti-false-trigger cooldown)
- `post_lock_ignore_ms`: delay after lock before first gate can arm timing
- `exit_confirm_below_samples`: consecutive samples below exit threshold required to close lap
- `rx5808_mode_select`: `0=AUTO`, `1=FORCE ON`, `2=FORCE OFF`
- `sd_lap_logging_enabled`: `1/true/on` enable lap CSV logging, `0/false/off` disable
- `channel_select_source`: `ADMIN` or `AUX1..AUX12`
- `aux_range_r1..aux_range_r8`: AUX microsecond ranges mapped to `R1..R8`
- `arm_source`: `NONE` or `AUX1..AUX12`
- `arm_active_min_us`, `arm_active_max_us`: AUX range considered ARM ON
- `new_race_after_disarm_ms`: disarm gap needed before next ARM starts a new race

## Not used in current firmware

No pin usage found for:
- external LED
- WS2812 LED
- buzzer
- battery voltage input
- mode switch input
