# Device Monitor via Vibration

Monitors a device that when operating causes vibration, such as a sump pump, washing machine, 
AC unit etc. by clamping an ADXL345 accelerometer to the device or in the case of a sump pump, 
the discharge pipe and detecting the motor's vibration. 

This particular instance was created for a sump pump but the applications are much wider.

Runs on an ESP32-C3 and reports pump on/off to an Adafruit IO feed named `pump`, and the exact 
length of each run to a feed named `pump-run-seconds`.

## How it works

- Samples the ADXL345 at 400 Hz and high-passes each axis (~13 Hz cutoff) to
  reject gravity, footsteps and house rumble while passing motor vibration
  (~29-58 Hz).
- Computes RMS over 250 ms windows.
- The onboard LED is off when quiet, solid while any vibration is detected
  (e.g. picking the board up), and blinks while the pump is considered ON.
- Pump state is debounced: 3 s of continuous vibration above `ON_RMS_THRESHOLD`
  starts a run, 3 s of continuous quiet below `OFF_RMS_THRESHOLD` ends it.
- Publishes 1/0 to the `pump` feed on state changes, at most once per minute,
  and only if the state differs from what the feed already shows. When a run
  ends, its exact duration in seconds goes to `pump-run-seconds` (the `pump`
  feed's timestamps can lag by up to a minute because of that rate limit).
- Serial output (115200) prints `rms=... state=... wifi=... aio=...` once a
  second. `aio=up` means an active MQTT session with Adafruit IO.

## Wiring

| ADXL345 | ESP32-C3 |
|---------|----------|
| SDA     | GPIO 5   |
| SCL     | GPIO 6   |

GPIO 8/9 are the LED and boot strapping pins on this board, so I2C is moved
off them. The LED is on GPIO 8 (set `LED_ACTIVE_LOW` in the sketch if yours
lights on HIGH).

## Setup

Copy `src/secrets.h.example` to `src/secrets.h` (git-ignored) and fill in your
Wi-Fi and Adafruit IO credentials:

```
cp src/secrets.h.example src/secrets.h
```

Create the `pump` and `pump-run-seconds` feeds in Adafruit IO. Optionally set
`NTFY_TOPIC` in `secrets.h` to enable push notifications (see
[Reliability and notifications](#reliability-and-notifications)).

Tune `ON_RMS_THRESHOLD` / `OFF_RMS_THRESHOLD` in `src/pump-sensor.ino` from the
`rms=` values printed with the pump idle and running.

## Reliability and notifications

This is meant to run unattended watching a sump pump, so it borrows the
self-healing approach from the
[RF ceiling fan remote](https://github.com/bradrblack/rf-ceiling-fan-remote)
firmware: recover on its own where possible, and never fail silently.

| Mechanism | Behavior |
|-----------|----------|
| **Bounded Wi-Fi connect at boot** | If Wi-Fi isn't up within 30 s, reboot and retry instead of hanging. |
| **Wi-Fi watchdog** | If Wi-Fi drops it resets the radio and retries every 30 s; if it stays down for 2 minutes, reboot. |
| **Adafruit IO watchdog** | Wi-Fi can stay associated while Adafruit IO is unreachable (DNS/TLS/auth), which the Wi-Fi watchdog can't see. Reboot if Wi-Fi is up but Adafruit IO has been unreachable for 10 minutes. |
| **I2C sensor watchdog** | Every 30 s the ADXL345's fixed device ID (`0xE5`) is read. Three failed checks in a row (unplugged, loose wire, wedged bus) reboot the board. Without this a dead sensor reads as zeros and looks like a permanently quiet pump. |
| **Sensor check at boot** | I2C bus recovery first (clocks SCL to release a bus the sensor is holding low, which a reboot alone doesn't clear), then waits for the ADXL345. If it never appears it pushes a notification after 1 minute and reboots after 30 minutes. |
| **Nightly reboot** | Once a day at 3 AM local time (`REBOOT_HOUR`, US Eastern via `TZ_STRING`, DST-aware, time from NTP) to guard against slow heap fragmentation. It is held off while the pump is running or a report is still unsent, so it never drops a run in progress. The "already rebooted today" flag is stored in flash so it can't reboot-loop within the 3 AM hour. |
| **Unexpected reset detection** | A crash, hardware/task watchdog or brownout resets the chip before any of the above can run. On the next boot the reset reason is checked, and anything other than a power cycle, reset button, flashing, or our own deliberate restart is reported. |
| **Stuck "running" correction** | The last published state is stored in flash. If the board reboots while the feed shows "running" and the pump is really idle 15 s after boot, it publishes 0 so the feed doesn't stay stuck on "running". |

### Reboot notifications (ntfy.sh)

The reason for every self-triggered reboot is written to flash right before
restarting, then pushed to [ntfy.sh](https://ntfy.sh) once Wi-Fi is up on the
next boot, e.g. `Pump sensor rebooted: ADXL345 not responding (I2C) (reboot #2)
at 2026-09-19 14:03:11 EDT`. A manual power cycle leaves no reason behind, so
it stays quiet. `reboot #N` counts only exception reboots, not the nightly one.

- Set `NTFY_TOPIC` in `secrets.h` to a random, unguessable topic (topics are
  unauthenticated) and subscribe to it in the ntfy app. Without it, pushes are
  disabled and the would-be message is only logged to serial.
- The nightly reboot also pushes by default, doubling as a "still alive"
  heartbeat for a device guarding a sump pump. Set `NOTIFY_DAILY_REBOOT` to
  `false` to only be told about exception reboots.

### Pump ON/OFF notifications

For initial testing (and as a second channel next to Adafruit IO), pump state
changes are also pushed to ntfy: `Pump ON` and `Pump OFF` (with the run length).
Same rules as the Adafruit IO feed: at most one push per minute, and only if the
state differs from the last one pushed, so a run that starts and ends inside a
minute sends nothing. Set `NOTIFY_PUMP_STATE` to `false` in the sketch to turn
these off once you trust the detection. Requires `NTFY_TOPIC`.

### Not ported from the fan project

The SinricPro watchdog is replaced by the Adafruit IO watchdog above, and the
CC1101 radio watchdog is replaced by the I2C sensor watchdog. The clock-jump
detection and OTA/WiFiManager pieces weren't carried over.

## Notes and gotchas

- `platformio.ini` uses `lib_ignore` for WiFi101 and WiFiNINA. Adafruit IO's
  dependency list pulls them in and their `WiFi.h` shadows the ESP32 one,
  which makes `WiFi.status()` hang.
- Wi-Fi TX power is reduced (`WIFI_TX_POWER`, 8.5 dBm). Without it this
  ESP32-C3 mini board joined Wi-Fi on only ~1 in 3 boots; with it, 4 of 4. If
  you swap boards and it connects fine at full power, this can be removed.
- Log lines are timestamped (wall-clock time once NTP syncs, uptime before
  that) and the boot banner prints `FIRMWARE_VERSION`; bump it on each flash
  you want to identify later.
- `pio run -t upload` currently crashes with the default esptool 5.4.0 (progress
  bar bug with esp-pylib 1.1.5). Build with `pio run`, then flash the combined
  image with an esptool 4.x install:

  ```
  esptool.py --chip esp32c3 -p <port> -b 115200 write_flash 0x0 .pio/build/c3_mini/firmware.factory.bin
  ```

- The USB port name changes each time the board re-enumerates; close any
  serial monitor before flashing.

## Future ideas

### Battery-powered version (deep sleep + accelerometer wake)

Not planned yet; parked for later. The idea is to sleep the ESP32 and let the
ADXL345 wake it:

1. ESP32 deep sleeps. The ADXL345 watches for motion (~25 uA) using its
   activity interrupt and drives INT1.
2. On wake, sample ~3 s with the existing RMS/continuity logic. Footsteps and
   bumps fail the check and it goes back to sleep.
3. On a confirmed run: join Wi-Fi, publish `pump=1`, then re-arm the ADXL to
   interrupt on *inactivity* (up to 255 s of no motion) and sleep.
4. On inactivity wake: publish `pump=0`, re-arm for activity, sleep.
5. Keep cycle count and last published state in RTC memory
   (`RTC_DATA_ATTR`) so the one-per-minute publish limit survives sleep.

Things to sort out first:

- Wire ADXL345 INT1 to a deep-sleep-capable GPIO (0-5 on the C3), e.g. GPIO 4.
  Reading `INT_SOURCE` clears the interrupt; wake is level-triggered.
- Board sleep current matters most: the bare chip sleeps at ~5 uA, but many dev
  boards (USB chip, power LED, regulator) draw 100 uA to 1 mA+.
- False wakes from footsteps cost ~4 s at 30-40 mA each; raise the activity
  threshold if the house is busy.
- Wi-Fi join is the expensive step (~3-5 s at 100 mA+), fine at a few pump
  cycles per day.
- Keep the current always-on sketch as a USB "debug mode" for threshold tuning.
