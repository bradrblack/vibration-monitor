# Changelog

Notable changes to the vibration monitor firmware, newest first. Versions refer
to `FIRMWARE_VERSION` in `src/vibration-monitor.ino`.

## 2026-09-22r1

- Cap the Adafruit IO client's network timeout at 5 s. A stalled DNS lookup,
  TCP connect, or TLS handshake could chain across the library's default 3 s
  per-call timeout and exceed the 30 s task watchdog before failing — this
  happened at least once in the field (task watchdog reboot on the pump
  device).

## 2026-09-21r12

- Make serial logging non-blocking (`Serial.setTxTimeoutMs(0)`), so a USB
  host that's attached but not reading can no longer stall the main loop.
  Excess output is dropped instead of blocking.

## 2026-09-21r10

- Add a 30 s hardware task watchdog as a backstop for a hung main loop
  (stuck I2C, network, or USB call) that the existing in-loop watchdogs
  can't see. A hang now resets the board and is reported via ntfy as an
  unexpected reset.
- Add the `[env:dryer]` PlatformIO build environment.

## 2026-09-20r9

- Publish JSON events via MQTT (Adafruit IO) and add I2C boot diagnostics.
- Fix banner formatting.
- Pin the PlatformIO platform release to work around an upload bug in the
  unpinned "stable" zip.

## 2026-09-20r3

- Generalize from a single sump-pump monitor to a multi-device appliance
  monitor with JSON events.

## 2026-09-20r1

- Add source files, update README, add Telegraf/Grafana setup for an
  InfluxDB appliance timeline (non-firmware, tooling only).

## 2026-09-19r1

- Add reliability hardening (WiFi/Adafruit IO watchdogs, scheduled nightly
  reboot, boot-state correction) and ntfy.sh reboot notifications.

## Initial version

- Add the ESP32-C3 sump pump vibration monitor.
