# Pump Sensor

Monitors a sump pump by clamping an ADXL345 accelerometer to the discharge pipe
and detecting the motor's vibration. Runs on an ESP32-C3 and reports pump
on/off to an Adafruit IO feed named `pump`.

## How it works

- Samples the ADXL345 at 400 Hz and high-passes each axis (~13 Hz cutoff) to
  reject gravity, footsteps and house rumble while passing motor vibration
  (~29-58 Hz).
- Computes RMS over 250 ms windows.
- The onboard LED follows the instantaneous RMS (bench-test indicator).
- Pump state is debounced: 3 s of continuous vibration above `ON_RMS_THRESHOLD`
  starts a run, 3 s of continuous quiet below `OFF_RMS_THRESHOLD` ends it.
- Publishes 1/0 to the `pump` feed on state changes, at most once per minute,
  and only if the state differs from what the feed already shows.
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

Tune `ON_RMS_THRESHOLD` / `OFF_RMS_THRESHOLD` in `src/pump-sensor.ino` from the
`rms=` values printed with the pump idle and running.

## Notes and gotchas

- `platformio.ini` uses `lib_ignore` for WiFi101 and WiFiNINA. Adafruit IO's
  dependency list pulls them in and their `WiFi.h` shadows the ESP32 one,
  which makes `WiFi.status()` hang.
- Wi-Fi TX power is reduced (`WIFI_TX_POWER`, 8.5 dBm). Without it this
  ESP32-C3 mini board joined Wi-Fi on only ~1 in 3 boots; with it, 4 of 4. If
  you swap boards and it connects fine at full power, this can be removed.
- The sketch resets the radio and retries every 30 s if Wi-Fi drops, and
  restarts itself after 5 minutes offline.
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
