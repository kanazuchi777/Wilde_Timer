# Wilde Timer

Wilde Timer is a single-gate FPV lap timer built on an ESP32 (or ESP32-S3).
It measures the RSSI of a passing drone's video transmitter with an RX5808
5.8 GHz receiver, draws lap times and stats straight onto your ELRS goggles
OSD over ESP-NOW (no extra wires to the goggles), takes channel selection and
ARM from your radio through an ELRS receiver (CRSF), and logs every valid lap
to a microSD card.

This document describes everything the firmware currently does. If the README
and the code ever disagree, the code is the source of truth.

---

## What you need

- ESP32 dev board (`esp32dev`) **or** ESP32-S3 DevKitC
- RX5808 5.8 GHz receiver module with RSSI output and SPI frequency control
- microSD card module (SPI) + a FAT32 microSD card
- An ELRS receiver bound to your radio, wired to the timer over CRSF UART
  (this is how the timer reads AUX channels and VTX info)
- ELRS goggles with a backpack (this is what shows the OSD, reached wirelessly
  over ESP-NOW — nothing to solder for this part)
- Common ground between all modules, and a 5 V supply (USB is fine)

> ⚠️ ESP32 GPIO are **3.3 V logic**. Never feed 5 V into a GPIO pin. Power the
> modules from 5 V/3.3 V as each module requires, but keep all signal lines at
> 3.3 V.

---

## Wiring / Soldering

All three peripherals connect to the ESP32. Pick the table that matches your
board. Tie every module's **GND to the ESP32 GND**.

### ESP32-S3 DevKitC

| Signal | From | ESP32-S3 pin |
|---|---|---|
| RX5808 RSSI (analog) | RX5808 RSSI pad | `GPIO4` |
| RX5808 DATA (CH1) | RX5808 | `GPIO10` |
| RX5808 SEL  (CH2) | RX5808 | `GPIO11` |
| RX5808 CLK  (CH3) | RX5808 | `GPIO12` |
| SD CS   | microSD module | `GPIO39` |
| SD SCK  | microSD module | `GPIO36` |
| SD MISO | microSD module | `GPIO37` |
| SD MOSI | microSD module | `GPIO35` |
| CRSF in | ELRS RX **TX** pad | `GPIO16` |
| CRSF out | ELRS RX **RX** pad | `GPIO17` |

### ESP32 DevKit (`esp32dev`)

| Signal | From | ESP32 pin |
|---|---|---|
| RX5808 RSSI (analog) | RX5808 RSSI pad | `GPIO34` (input only) |
| RX5808 DATA | RX5808 | `GPIO23` |
| RX5808 CLK  | RX5808 | `GPIO18` |
| RX5808 SEL  | RX5808 | `GPIO5` |
| SD CS   | microSD module | `GPIO33` |
| SD SCK  | microSD module | `GPIO25` |
| SD MISO | microSD module | `GPIO27` |
| SD MOSI | microSD module | `GPIO26` |
| CRSF in | ELRS RX **TX** pad | `GPIO16` |
| CRSF out | ELRS RX **RX** pad | `GPIO17` |

CRSF UART runs at **420000 baud**.

### Soldering notes

- **RX5808 RSSI** is an analog voltage (~0–3.3 V). Solder it to the dedicated
  RSSI pad/pin of the module and run it to the analog pin above. Keep this wire
  short and away from noisy power lines for cleaner readings.
- **RX5808 frequency control** uses three pins (DATA/SEL/CLK). Some RX5808
  modules ship in "button" mode and need the SPI mod (a resistor/jumper change)
  before the timer can set the channel. If the channel never tunes, check your
  module's SPI-enable mod.
- **CRSF** is a single half-duplex line on the ELRS side, but here it is wired
  as a normal UART: the receiver's **TX** pad goes to the ESP32 **RX (GPIO16)**.
  The ESP32 **TX (GPIO17)** back to the receiver is only needed for writing VTX
  Admin channel changes back toward the goggles; it is harmless to connect.
- **microSD** is standard SPI. A 3.3 V-capable module is recommended; many
  modules include a level shifter and 5 V regulator.
- Bring all grounds to a single common point.

---

## How it works

### Link and bind (to the goggles)

- The timer talks to the ELRS goggles backpack wirelessly over **ESP-NOW**.
- It auto-binds with a broadcast (`MSP_ELRS_BIND`) and keeps the link alive with
  periodic probes (an OSD "display" command).
- If no successful traffic is seen for ~5 s, the link is considered lost and
  auto-bind restarts. The OSD is redrawn automatically on reconnect.

### Channel source

The channel the timer listens on is chosen externally, never by a blind sweep.
Two sources are supported (set via `channel_select_source`):

- **`AUX7` (default)** — an AUX switch/knob on your radio. The AUX microsecond
  value is matched against 8 configured ranges and mapped to `R1..R8`.
- **`ADMIN`** — the channel is taken from the CRSF VTX Admin info coming from
  the goggles.

When the channel changes, lap/session stats are preserved and the timer
re-calibrates for the new channel. **Both sources now lock the same way**
(see below) — there is no separate "instant lock" path anymore.

### Scan → Lock → Timing

1. In **scan/calibration**, the timer tunes the RX5808 to the selected channel
   and reads RSSI.
2. **Arm**: when RSSI rises above `lock_threshold_rssi`, peak capture is armed
   and the highest RSSI is tracked.
3. **Lock**: when RSSI then drops below the dynamic exit level
   (`lock_threshold_rssi + exit_offset_rssi`), the channel locks and the timer
   switches to **timing** mode. This "peak then drop" pattern mimics a drone
   approaching and leaving the gate.
4. On lock, the enter/exit thresholds are seeded from the lock RSSI. On the
   first gate pass of the race they are re-calibrated once from the measured
   gate peak.
5. The gate pass that causes the lock starts the timing reference; the next
   qualifying pass completes lap 1.

Both scan and timing RSSI loops run at about a **2 ms** period.

### Lap detection

A gate pass is registered when:

- RSSI rose above the **enter** threshold (`T+`), tracking the peak, and
- RSSI then dropped below the **exit** threshold (`T-`) for **100 consecutive
  samples** (~200 ms of confirmation), and
- at least `min_lap_interval_ms` has passed since the last lap.

Laps longer than 60 s are hidden (the timing reference is kept in sync, but the
lap is not counted or shown).

### Fake-lap (outlier) filter

To reject impossibly fast readings (e.g. a double trigger or RSSI glitch):

- it takes the **median of up to the last 7 laps**,
- a lap is rejected if it is faster than `max(median − 5 s, 75% of median)`,
- the filter only activates once at least 4 laps exist, so the very first laps
  are not checked.

A rejected lap is **not counted** and shows `IGN ss.cc` briefly, but the timing
reference still advances (the gate pass was real). Rejected laps are still kept
in the internal history so the median can track a pilot who is genuinely getting
faster.

### Best laps and stats

- **`SF` (session fastest)** — the fastest lap within the **rolling last 100
  laps**. An early glitch lap that slips through drops out once 100 newer laps
  accumulate. A new SF blinks the lap popup for ~5 s.
- **`RF` (race fastest)** — the fastest lap of the current race (resets each
  new race).
- **`S3`** — best 3-consecutive-lap sum over the whole session.
- **`R3`** — best 3-consecutive-lap sum in the current race.
- **TOP50 average** — baseline used by the lap popup delta: the average of the
  fastest 50% of the last 100 valid laps.

### Race and session

- A **race** is one run; a **session** is everything since power-on.
- With an ARM source configured, timing is paused while disarmed.
- If you stay disarmed for at least `new_race_after_disarm_ms`, the next ARM
  starts a **new race** (RF/R3/race lap list reset; session stats kept). Pending
  lap logs are flushed to SD during a long disarm.
- A short disarm/re-arm (e.g. a crash + turtle) continues the same race.

### Lap popup

After each lap a popup shows the lap number, lap time, and a delta against the
TOP50 baseline:

```
Lxx ss.cc +dd.cc   (slower than baseline)
Lxx ss.cc -dd.cc   (faster than baseline)
```

---

## OSD

Each element has a configurable position `[col, row, showDuringRace]`.

| Field | Key | Meaning |
|---|---|---|
| Channel | `gOsdChannel` | Selected channel name (e.g. `R4`) |
| RSSI | `gOsdRssi` | Current RSSI `R:nnn` |
| Lap peak RSSI | `gOsdLapPeakRssi` | `LP:nnn` — rolling peak RSSI over the last ~5 s (debug) |
| Enter threshold | `gOsdRssiThrUpper` | `T+:nnn` |
| Exit threshold | `gOsdRssiThrLower` | `T-:nnn` |
| Session fastest | `gOsdBestLap` | `SF` |
| Race fastest | `gOsdBestLap_race` | `RF` |
| Session best-3 | `gOsdBest3` | `S3` |
| Race best-3 | `gOsdBest3_race` | `R3` |
| Race lap list | `gOsdRaceLaps` | Column `L1`, `L2`, ... |
| Lap popup | `gOsdLapPopup` | Last lap + delta |

`WAIT VTX ADMIN` is an internal status message (shown in ADMIN mode until a
channel is known) and is not configurable.

`showDuringRace`:

- `1` = keep showing while a race is active
- `0` = hide while a race is active

Race lap list rendering:

- During a race: shows the current race laps (if enabled).
- Outside a race: if the current list is empty, it may show the last completed
  race's laps.
- If there are more laps than OSD rows, it shows the most recent rows.

---

## microSD

- **Config file:** `/config.txt` — auto-created with defaults on first boot;
  missing keys are auto-filled.
- **Lap log:** `/LOGS/laps.csv` with columns
  `boot_ms,race_no,lap_no,lap_ms,channel,is_new_best`.
- Laps are buffered in RAM and flushed to SD (also during long disarms).
- If the card is removed/fails at runtime, the firmware tries to recover; on
  repeated failure it disables lap logging until reboot to avoid error spam.

### USB mass storage (ESP32-S3 only)

If the SD is ready, the S3 exposes the card as a USB drive when plugged into a
computer. While the computer is reading/writing the card, the timer pauses its
main loop to avoid SD/SPI contention. Unplug USB to resume timing.

---

## `/config.txt`

### OSD positions

Format `key=[col,row,showDuringRace]` (legacy `[col,row]` still accepted):

- `gOsdChannel`, `gOsdRssi`, `gOsdLapPeakRssi`
- `gOsdRssiThrUpper`, `gOsdRssiThrLower`
- `gOsdBestLap`, `gOsdBestLap_race`
- `gOsdBest3`, `gOsdBest3_race`
- `gOsdRaceLaps`, `gOsdLapPopup`
- also `osd_main_row`, `osd_main_col`

### Timing and RSSI

- `lock_threshold_rssi` — RSSI level that arms peak capture for lock
- `enter_offset_rssi` — enter threshold offset from the reference RSSI
- `exit_offset_rssi` — exit threshold offset from the reference RSSI
- `min_lap_interval_ms` — minimum time between counted laps
- `post_lock_ignore_ms` — legacy key, not used by current logic
- exit confirmation is fixed at 100 consecutive below-threshold samples

### RX5808, channel source, ARM

- `rx5808_mode_select` — `0=AUTO detect`, `1=FORCE ON`, `2=FORCE OFF`
- `channel_select_source` — `ADMIN` or `AUX1..AUX12`
- `aux_range_r1..aux_range_r8` — microsecond ranges mapped to `R1..R8`
- `arm_source` — `NONE` or `AUX1..AUX12`
- `arm_active_min_us`, `arm_active_max_us` — AUX range that counts as armed
- `new_race_after_disarm_ms` — disarm gap that triggers a new race

### SD

- `sd_lap_logging_enabled` — `0/1`, `true/false`, or `on/off`

### Default values

```
osd_main_row=17
osd_main_col=13
gOsdChannel=[9,16,1]
gOsdRssi=[12,16,1]
gOsdLapPeakRssi=[18,16,1]
gOsdRssiThrUpper=[25,16,1]
gOsdRssiThrLower=[32,16,1]
gOsdBestLap=[9,17,1]
gOsdBestLap_race=[21,17,1]
gOsdBest3=[33,17,1]
gOsdBest3_race=[42,0,1]
gOsdRaceLaps=[42,1,1]
gOsdLapPopup=[20,12,1]
lock_threshold_rssi=100
enter_offset_rssi=-25
exit_offset_rssi=-40
min_lap_interval_ms=8000
post_lock_ignore_ms=6000
rx5808_mode_select=0
sd_lap_logging_enabled=1
channel_select_source=AUX7
aux_range_r1=1540-1560
aux_range_r2=1565-1620
aux_range_r3=1625-1660
aux_range_r4=1665-1720
aux_range_r5=1725-1760
aux_range_r6=1765-1820
aux_range_r7=1825-1860
aux_range_r8=1860-1920
arm_source=AUX1
arm_active_min_us=1700
arm_active_max_us=2100
new_race_after_disarm_ms=10000
```

### Auto-clamped ranges

Out-of-range values are clamped automatically:

- OSD `row`: `0..17`, OSD `col`: `0..50`
- `lock_threshold_rssi`: `60..230`
- `enter_offset_rssi`, `exit_offset_rssi`: `-60..60`
- `min_lap_interval_ms`: `1000..60000`
- `new_race_after_disarm_ms`: `0..300000`
- after calibration: `enter` clamps to `80..100`, `exit` to `60..75`

---

## Using it

1. **Insert the microSD** and power the timer (USB or 5 V). On first boot it
   creates `/config.txt` with defaults. Edit that file to tune positions,
   thresholds, AUX ranges, and ARM, then power-cycle to apply.
2. **Power your goggles** with the ELRS backpack. The timer auto-binds; the OSD
   appears once linked.
3. **Bind your ELRS receiver** to your radio and wire it to the CRSF pins.
4. **Select the channel.** With the default `AUX7` source, move your AUX7
   switch/knob to the position whose microsecond range matches the channel you
   are flying (`R1..R8`). The timer tunes the RX5808, syncs the goggles, then
   calibrates.
5. **ARM.** With the default `AUX1` ARM source, flip ARM to the active range
   (1700–2100 µs) to allow timing. With `arm_source=NONE`, timing is always
   allowed.
6. **Fly the gate.** Each clean pass closes a lap, updates the OSD stats, and
   logs to SD. A new personal best blinks the popup.
7. **Start a fresh race** by staying disarmed for at least
   `new_race_after_disarm_ms`, then ARM again.

---

## Serial diagnostics (115200 baud)

On boot, useful lines:

- `Settings: ...`, `OSD elements: ...`, `OSD showDuringRace: ...`
- `Channel select source: ...`, `ARM source: ...`, `RX5808 ...`

During flying:

- `LAP ...` — a counted lap
- `TOP50 AVG ... DELTA ...` — baseline and lap delta
- `Lap ignored as outlier ...` — the fake-lap filter rejected a pass

---

## Not included

There is no active logic for a buzzer, WS2812 LEDs, battery-voltage input, or a
separate mode switch — selection and ARM come from your radio's AUX channels.
