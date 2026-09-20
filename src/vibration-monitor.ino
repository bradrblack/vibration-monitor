#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>
#include <cstdarg>
#include <esp_system.h>
#include <esp32c3/rom/rtc.h>
#include <Adafruit_ADXL345_U.h>
#include "AdafruitIO_WiFi.h"
#include "secrets.h"

// Bump on each flash you want to identify later -- format: YYYY-MM-DDrN.
#define FIRMWARE_VERSION "2026-09-20r3"

// ---- Per-device configuration ---------------------------------------------
// One firmware, many devices: each PlatformIO environment (see platformio.ini)
// sets DEVICE_NAME plus whatever tuning that machine needs via build flags.
// DEVICE_NAME appears in every Adafruit IO event and ntfy message; keep it to
// plain characters (no quotes or backslashes) since it goes into JSON as-is.
#ifndef DEVICE_NAME
#error "DEVICE_NAME is not set. Build a device environment, e.g. `pio run -e pump` (see platformio.ini)."
#endif

// Vibration thresholds (m/s^2 RMS): above ON_RMS_THRESHOLD = vibrating, below
// OFF_RMS_THRESHOLD = quiet, with the band in between as hysteresis.
#ifndef ON_RMS_THRESHOLD
#define ON_RMS_THRESHOLD 0.40f
#endif
#ifndef OFF_RMS_THRESHOLD
#define OFF_RMS_THRESHOLD 0.20f
#endif
// A run starts after RUN_CONFIRM_MS of continuous vibration and ends after
// STOP_CONFIRM_MS of continuous quiet. Machines with pauses inside a cycle
// (a washer filling/soaking/draining) need a long STOP_CONFIRM_MS so one cycle
// isn't reported as several.
#ifndef RUN_CONFIRM_MS
#define RUN_CONFIRM_MS 3000UL
#endif
#ifndef STOP_CONFIRM_MS
#define STOP_CONFIRM_MS 3000UL
#endif

// Adafruit IO feed shared by all devices. Every event says which device it is.
#ifndef EVENT_FEED
#define EVENT_FEED "appliance-events"
#endif

// NVS (flash) namespace for persisted state.
#define NVS_NS "devmon"

const char *BANNER =
R"(__     ___ _               _   _               ____
   \ \   / (_) |__  _ __ __ _| |_(_) ___  _ __   / ___|  ___ _ __  ___  ___
    \ \ / /| | '_ \| '__/ _` | __| |/ _ \| '_ \  \___ \ / _ \ '_ \/ __|/ _ \
     \ V / | | |_) | | | (_| | |_| | (_) | | | |  ___) |  __/ | | \__ \  __/
      \_/  |_|_.__/|_|  \__,_|\__|_|\___/|_| |_| |____/ \___|_| |_|___/\___|
                                                                            )";

// ---- Pins -----------------------------------------------------------------
const int SDA_PIN = 5;
const int SCL_PIN = 6;   // GPIO 8/9 are LED/boot pins on this board

const int  LED_PIN        = 8;
const bool LED_ACTIVE_LOW = true;  // set false if the LED lights when the pin is HIGH
const uint32_t LED_BLINK_HALF_PERIOD_MS = 500;  // blink rate while the device is running

// ---- Vibration detection --------------------------------------------------
// Samples are high-passed against a slow per-axis baseline (removes gravity and
// mounting angle), then RMS'd over a short window. Tune the thresholds from the
// "rms=" values printed over serial while the device is idle and running.
// (ON_RMS_THRESHOLD / OFF_RMS_THRESHOLD are set per device, see above.)

const uint32_t SAMPLE_PERIOD_US  = 2500;   // 400 Hz, matches accel data rate
const uint16_t SAMPLES_PER_WINDOW = 100;   // 250 ms windows
// High-pass cutoff ~ alpha * fs / 2pi = ~13 Hz: rejects footsteps / house rumble
// (<10 Hz) while passing motor vibration (~29-58 Hz for 1725/3450 RPM).
const float    BASELINE_ALPHA    = 0.2;

// ---- Adafruit IO ----------------------------------------------------------
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *eventFeed = io.feed(EVENT_FEED);
// Free Adafruit IO accounts are rate limited (and shared by every device on the
// account), and events should be rare: publish at most one event per minute.
// Events queue up in the meantime, each stamped with when it really happened.
const uint32_t MIN_PUBLISH_INTERVAL_MS = 60000;
const uint32_t PUBLISH_RETRY_MS = 10000;

// ---- Reliability ----------------------------------------------------------
// Many ESP32-C3 mini boards have a marginal antenna/regulator design and fail to
// join at full TX power (~19.5 dBm); capping it is the usual fix. Drop to
// WIFI_POWER_8_5dBm if joins are flaky at this level.
const wifi_power_t WIFI_TX_POWER = WIFI_POWER_15dBm;
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000;   // at boot: reboot if no WiFi by then
const uint32_t WIFI_RETRY_MS           = 30000;   // reset the radio and retry this often
const uint32_t WIFI_DOWN_REBOOT_MS     = 120000;  // running: reboot if WiFi down this long
// WiFi can stay associated while Adafruit IO itself is unreachable (DNS/TLS/
// auth problems), which the WiFi watchdog can't see.
const uint32_t AIO_DOWN_REBOOT_MS      = 600000;  // 10 min

// The ADXL345 is polled for its fixed device ID; a wedged or unplugged I2C
// bus otherwise just produces a stream of zeros (looks like a quiet device).
const uint32_t SENSOR_CHECK_INTERVAL_MS = 30000;
const int      SENSOR_FAIL_LIMIT        = 3;       // consecutive failed checks => reboot
const uint32_t SENSOR_BOOT_RETRY_MS     = 5000;    // at boot: retry detection this often
const uint32_t SENSOR_NOTIFY_AFTER_MS   = 60000;   // at boot: push a notification after this long
const uint32_t SENSOR_BOOT_REBOOT_MS    = 1800000; // at boot: reboot if still missing after 30 min
const uint8_t  ADXL345_DEVICE_ID        = 0xE5;

// Scheduled reboot once a day at a fixed local hour, to guard against slow heap
// fragmentation from the long-running MQTT/TLS connection. POSIX TZ string (not
// a fixed offset) so DST is handled -- US Eastern.
#define TZ_STRING   "EST5EDT,M3.2.0,M11.1.0/2"
#define REBOOT_HOUR 3
const char *NTP_SERVER = "pool.ntp.org";
// The daily reboot is routine, but a push each night doubles as a heartbeat
// for an unattended monitoring device. Set false to
// only be notified of exception reboots.
#define NOTIFY_DAILY_REBOOT true
#define REASON_DAILY_REBOOT "Daily scheduled reboot"

// Also push started/stopped events to ntfy (for initial testing / as a second
// channel). At most one per minute, and only if the state differs from the last
// one pushed (so a start and stop inside a minute sends nothing). Requires
// NTFY_TOPIC.
#define NOTIFY_RUN_STATE true
#if NOTIFY_RUN_STATE && defined(NTFY_TOPIC)
#define RUN_NTFY
#endif

// If the board was restarted while the feed's last event for this device was a
// "start", correct it once the device has been observed for this long and is
// really idle.
const uint32_t BOOT_SETTLE_MS = 15000;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ---- State ----------------------------------------------------------------
float bx, by, bz;              // per-axis baseline
double sumSq = 0;
uint16_t sampleCount = 0;
uint32_t nextSampleUs = 0;

bool ledOn = false;            // instantaneous vibration indicator
bool deviceRunning = false;
uint32_t aboveSince = 0;       // 0 = not currently above ON threshold
uint32_t belowSince = 0;       // 0 = not currently below OFF threshold
uint32_t runStartMs = 0;
uint32_t cycleCount = 0;
uint32_t totalRunMs = 0;

bool aioUp = false;
uint32_t lastWifiAttempt = 0;
uint32_t wifiDownSince = 0;    // 0 = WiFi currently up
uint32_t aioDownSince = 0;     // 0 = Adafruit IO currently up (or WiFi down)
int lastPublishedState = -1;   // last event published for this device: 1 start, 0 stop, -1 unknown; persisted in NVS
// Initialised so the first publish after boot isn't held back by the rate limit.
uint32_t lastPublishMs = (uint32_t)(0 - MIN_PUBLISH_INTERVAL_MS);
uint32_t lastAttemptMs = 0;
uint32_t lastStatusMs = 0;
float lastRms = 0;

int pendingNtfy = -1;          // -1 none, else 0/1 state waiting to be pushed
int lastNtfyState = -1;
uint32_t lastNtfyMs = (uint32_t)(0 - MIN_PUBLISH_INTERVAL_MS);
uint32_t lastNtfyAttemptMs = 0;
float ntfyRunSeconds = 0;      // length of the run that ended (for the stopped message)
char ntfyEventTime[32] = "";   // when the pending state change happened

// Events waiting to be published to Adafruit IO, oldest first.
struct DeviceEvent {
  bool start;                  // true = started, false = stopped
  float seconds;               // run length, stop events only (<0 = unknown)
  char at[32];                 // when it happened (ISO 8601), empty if the clock wasn't synced
  const char *note;            // optional, e.g. "restart" for a corrected stale state
};
const int EVENT_QUEUE_SIZE = 8;
DeviceEvent eventQueue[EVENT_QUEUE_SIZE];
int eventCount = 0;

int lastRebootDay = -1;        // persisted in NVS so the daily reboot fires once, not in a loop
int sensorFails = 0;
uint32_t lastSensorCheck = 0;
bool bootCorrectionDone = false;

// ==========================================
// Timestamped logging
// ==========================================
// Prefixes each line with the synced wall-clock time once NTP has synced, or
// uptime in seconds before that.
const char *logTimestamp() {
  static char buf[32];
  time_t now = time(nullptr);
  if (now > 100000) {
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &timeinfo);
  } else {
    snprintf(buf, sizeof(buf), "+%lus", millis() / 1000);
  }
  return buf;
}

void logf(const char *fmt, ...) {
  char msg[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  Serial.printf("[%s] %s\r\n", logTimestamp(), msg);
}

// Formats when an event happened. State changes are confirmed after a debounce
// delay, so ageMs (how long ago it really started/stopped) is subtracted.
// Leaves the buffer empty if the clock hasn't synced.
void formatEventTime(char *buf, size_t len, uint32_t ageMs, const char *fmt) {
  buf[0] = 0;
  time_t now = time(nullptr);
  if (now < 100000) return;
  time_t t = now - (time_t)(ageMs / 1000);
  struct tm timeinfo;
  localtime_r(&t, &timeinfo);
  strftime(buf, len, fmt, &timeinfo);
}

// ==========================================
// Reboot reporting (ntfy.sh)
// ==========================================
// Self-triggered reboots would otherwise be silent. The reason is written to
// NVS flash right before restarting, then read back and pushed to ntfy.sh once
// WiFi is up on the next boot. A manual power cycle leaves no reason behind, so
// it stays quiet. Define NTFY_TOPIC in secrets.h to enable pushes.
Preferences prefs;

// Reads the recorded reboot reason ("" if none), without the driver logging an
// error for a key that was never written.
String readRebootReason() {
  return prefs.isKey("last_reason") ? prefs.getString("last_reason", "") : String("");
}

void recordRebootReason(const char *reason) {
  prefs.begin(NVS_NS, false);
  prefs.putString("last_reason", reason);
  // The daily reboot isn't a symptom of anything -- only count exception
  // reboots, so "reboot #N" means something went wrong N times.
  if (strcmp(reason, REASON_DAILY_REBOOT) != 0) {
    prefs.putUInt("reboot_count", prefs.getUInt("reboot_count", 0) + 1);
  }
  prefs.end();
}

void restartWithReason(const char *reason) {
  logf("Restarting: %s", reason);
  recordRebootReason(reason);
  delay(100);
  ESP.restart();
}

// Detects a reboot that bypassed our own restartWithReason() -- a crash, a
// hung network stack tripping the hardware watchdog, a brownout. Those resets
// happen before our code runs, so without this they're invisible. Must run
// before anything else in setup() can record a reason for this boot.
void checkUnexpectedReset() {
  prefs.begin(NVS_NS, false);
  String reason = readRebootReason();
  prefs.end();
  if (reason.length() > 0) return; // a deliberate reboot already recorded this

  esp_reset_reason_t r = esp_reset_reason();
  // POWERON/EXT are power cycles or the reset button; SW is our own
  // ESP.restart(), which would already have recorded a reason.
  if (r == ESP_RST_POWERON || r == ESP_RST_EXT || r == ESP_RST_SW) return;

  // A serial tool toggling RTS (flashing, opening a monitor) isn't classified
  // by esp_reset_reason(); check the raw code so that doesn't look like a crash.
  RESET_REASON raw = rtc_get_reset_reason(0);
  if (raw == USB_UART_CHIP_RESET || raw == USB_JTAG_CHIP_RESET) return;

  const char *desc;
  switch (r) {
    case ESP_RST_PANIC:     desc = "crash/panic"; break;
    case ESP_RST_INT_WDT:   desc = "interrupt watchdog"; break;
    case ESP_RST_TASK_WDT:  desc = "task watchdog"; break;
    case ESP_RST_WDT:       desc = "other watchdog"; break;
    case ESP_RST_BROWNOUT:  desc = "brownout"; break;
    case ESP_RST_DEEPSLEEP: desc = "deep sleep wake"; break;
    default:                desc = "unknown"; break;
  }
  char buf[48];
  snprintf(buf, sizeof(buf), "Unexpected reset (%s)", desc);
  recordRebootReason(buf);
}

bool sendNtfy(const String &title, const String &message) {
#ifdef NTFY_TOPIC
  if (WiFi.status() != WL_CONNECTED) return false;
  logf("[ntfy] Sending: \"%s\" - \"%s\"", title.c_str(), message.c_str());
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, String("https://ntfy.sh/") + NTFY_TOPIC)) return false;
  http.addHeader("Title", title);
  int code = http.POST(message);
  logf("[ntfy] POST status: %d", code);
  http.end();
  return code >= 200 && code < 300;
#else
  logf("[ntfy] (disabled, no NTFY_TOPIC) %s - %s", title.c_str(), message.c_str());
  return false;
#endif
}

void notifyLastReboot() {
  prefs.begin(NVS_NS, false);
  String reason = readRebootReason();
  uint32_t count = prefs.getUInt("reboot_count", 0);
  if (reason.length() > 0) {
    prefs.putString("last_reason", ""); // clear so a normal boot stays quiet
  }
  prefs.end();

  if (reason.length() == 0) return;
  if (!NOTIFY_DAILY_REBOOT && reason == REASON_DAILY_REBOOT) return;

  sendNtfy(String(DEVICE_NAME) + " rebooted",
           reason + " (reboot #" + String(count) + ") at " + logTimestamp());
}

void saveLastPublishedState(int state) {
  prefs.begin(NVS_NS, false);
  prefs.putInt("last_state", state);
  prefs.end();
}

// ==========================================
// Sensor
// ==========================================
void setLed(bool on) {
  digitalWrite(LED_PIN, (on != LED_ACTIVE_LOW) ? HIGH : LOW);
}

void readSample(float &x, float &y, float &z) {
  sensors_event_t event;
  accel.getEvent(&event);
  x = event.acceleration.x;
  y = event.acceleration.y;
  z = event.acceleration.z;
}

// A reset can leave the ADXL345 holding SDA low mid-byte, which no amount of
// rebooting the ESP32 clears. Clocking SCL up to 9 times lets it finish the
// byte and release the bus; then a STOP condition frees it.
void i2cBusRecover() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9 && digitalRead(SDA_PIN) == LOW; i++) {
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  }
  // STOP: SDA low -> high while SCL high.
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(10);
}

// Waits for the ADXL345 at boot. If it's missing: keep retrying, push a
// notification after a minute (so a dead sensor isn't silent), and reboot after
// 30 minutes as a last resort (the reason is pushed on the next boot).
void setupSensor() {
  i2cBusRecover();
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  uint32_t start = millis();
  bool notified = false;
  while (!accel.begin()) {
    logf("[I2C] ADXL345 not detected at boot -- check wiring (SDA=%d SCL=%d)", SDA_PIN, SCL_PIN);
    uint32_t waited = millis() - start;
    if (!notified && waited >= SENSOR_NOTIFY_AFTER_MS) {
      notified = true;
      sendNtfy(String(DEVICE_NAME) + ": accelerometer missing",
               String("ADXL345 not responding on I2C since boot at ") + logTimestamp());
    }
    if (waited >= SENSOR_BOOT_REBOOT_MS) {
      restartWithReason("ADXL345 not detected at boot (I2C)");
    }
    delay(SENSOR_BOOT_RETRY_MS);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_400_HZ);
  readSample(bx, by, bz);
  logf("[I2C] ADXL345 initialized");
}

// Periodically confirms the ADXL345 still answers over I2C. Without this a
// disconnected/wedged sensor reads as zeros and looks like a permanently
// quiet device.
void checkSensorWatchdog() {
  uint32_t now = millis();
  if (now - lastSensorCheck < SENSOR_CHECK_INTERVAL_MS) return;
  lastSensorCheck = now;

  if (accel.getDeviceID() == ADXL345_DEVICE_ID) {
    sensorFails = 0;
    return;
  }
  sensorFails++;
  logf("[I2C] ADXL345 not responding (%d/%d)", sensorFails, SENSOR_FAIL_LIMIT);
  if (sensorFails >= SENSOR_FAIL_LIMIT) {
    restartWithReason("ADXL345 not responding (I2C)");
  }
}

// ==========================================
// Run state and events
// ==========================================
// Appends an event to the publish queue (oldest dropped if it's ever full).
void enqueueEvent(bool start, float seconds, uint32_t ageMs, const char *note = nullptr) {
  if (eventCount == EVENT_QUEUE_SIZE) {
    logf("Event queue full, dropping oldest event");
    memmove(&eventQueue[0], &eventQueue[1], sizeof(DeviceEvent) * (EVENT_QUEUE_SIZE - 1));
    eventCount--;
  }
  DeviceEvent &e = eventQueue[eventCount++];
  e.start = start;
  e.seconds = seconds;
  e.note = note;
  formatEventTime(e.at, sizeof(e.at), ageMs, "%Y-%m-%dT%H:%M:%S%z");
}

// e.g. {"device":"washer","event":"stop","seconds":2820.4,"at":"2026-09-20T09:12:01-0400"}
void buildEventJson(const DeviceEvent &e, char *buf, size_t len) {
  int n = snprintf(buf, len, "{\"device\":\"%s\",\"event\":\"%s\"", DEVICE_NAME,
                   e.start ? "start" : "stop");
  if (e.seconds >= 0 && n < (int)len) n += snprintf(buf + n, len - n, ",\"seconds\":%.1f", e.seconds);
  if (e.at[0] && n < (int)len)        n += snprintf(buf + n, len - n, ",\"at\":\"%s\"", e.at);
  if (e.note && n < (int)len)         n += snprintf(buf + n, len - n, ",\"note\":\"%s\"", e.note);
  if (n < (int)len)                   snprintf(buf + n, len - n, "}");
}

void setRunState(bool running, uint32_t eventMs) {
  deviceRunning = running;
  uint32_t ageMs = millis() - eventMs;
  float seconds = -1;
  if (running) {
    runStartMs = eventMs;
    logf("%s STARTED", DEVICE_NAME);
  } else {
    uint32_t dur = eventMs - runStartMs;
    cycleCount++;
    totalRunMs += dur;
    seconds = dur / 1000.0f;
    logf("%s STOPPED - ran %.1f s (cycle %lu, total run %.1f s)", DEVICE_NAME,
         seconds, (unsigned long)cycleCount, totalRunMs / 1000.0);
    ntfyRunSeconds = seconds;
  }
  enqueueEvent(running, seconds, ageMs);
#ifdef RUN_NTFY
  pendingNtfy = running ? 1 : 0;
  formatEventTime(ntfyEventTime, sizeof(ntfyEventTime), ageMs, "%Y-%m-%d %H:%M:%S %Z");
#endif
}

// Called once per window with that window's RMS.
void processWindow(float rms, uint32_t now) {
  lastRms = rms;

  // Instantaneous vibration indicator (with hysteresis).
  if (rms > ON_RMS_THRESHOLD) ledOn = true;
  else if (rms < OFF_RMS_THRESHOLD) ledOn = false;

  // Debounced running/stopped state.
  if (rms > ON_RMS_THRESHOLD) {
    if (!aboveSince) aboveSince = now;
    belowSince = 0;
    if (!deviceRunning && now - aboveSince >= RUN_CONFIRM_MS) {
      setRunState(true, aboveSince);
    }
  } else if (rms < OFF_RMS_THRESHOLD) {
    if (!belowSince) belowSince = now;
    aboveSince = 0;
    if (deviceRunning && now - belowSince >= STOP_CONFIRM_MS) {
      setRunState(false, belowSince);
    }
  } else {
    // Hysteresis band: neither counts toward starting a run (footsteps are
    // impulsive, a motor is continuous) nor toward ending one.
    aboveSince = 0;
    belowSince = 0;
  }

  // LED: blinks while the device is considered running; otherwise solid while any
  // vibration is detected, and fully off when quiet.
  if (deviceRunning) setLed((now / LED_BLINK_HALF_PERIOD_MS) % 2 == 0);
  else setLed(ledOn);
}

// If we restarted (crash, watchdog, nightly reboot...) while this device's last
// published event was a "start", the feed would keep implying it's running until
// its next real run. After a short settle time, if it's really idle, publish a
// stop (marked as a restart correction, with no run length).
void checkBootStateCorrection(uint32_t now) {
  if (bootCorrectionDone || now < BOOT_SETTLE_MS) return;
  bootCorrectionDone = true;
  if (lastPublishedState == 1 && !deviceRunning && eventCount == 0) {
    logf("Last event was a start but the device is idle after a restart; correcting");
    enqueueEvent(false, -1, 0, "restart");
  }
}

// ==========================================
// Network
// ==========================================
// Bring the radio up in station mode first (io.connect() alone tears down a
// radio that was never started, which logs ESP_ERR_WIFI_NOT_INIT and can leave
// it unable to join), then let Adafruit IO start the connection.
void startNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_TX_POWER);
  WiFi.setAutoReconnect(true);
  io.connect();
}

// Bounded connect at boot: reboot and retry rather than sit there offline.
void setupWiFi() {
  startNetwork();
  lastWifiAttempt = millis();
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
    if (millis() - start > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println();
      restartWithReason("WiFi connect timeout at boot");
    }
  }
  Serial.println();
  logf("[WiFi] IP address %s", WiFi.localIP().toString().c_str());
}

void setupTime() {
  // Re-kick SNTP now that WiFi is up (the early configTzTime() in setup() ran
  // with no network, so its first sync attempt failed and backed off).
  configTzTime(TZ_STRING, NTP_SERVER);
  logf("[NTP] Syncing time");
  time_t now = time(nullptr);
  uint32_t start = millis();
  while (now < 100000 && millis() - start < 10000) {
    Serial.print(".");
    delay(250);
    now = time(nullptr);
  }
  Serial.println();
  if (now < 100000) {
    logf("[NTP] failed to sync (will keep retrying in the background)");
    return;
  }
  logf("[NTP] synced");
}

void serviceNetwork(uint32_t now) {
  if (WiFi.status() != WL_CONNECTED) {
    aioUp = false;
    if (now - lastWifiAttempt >= WIFI_RETRY_MS) {
      lastWifiAttempt = now;
      // A connect attempt can get stuck; io.connect() then fails with
      // "sta is connecting". Fully reset the radio first.
      WiFi.disconnect(true);
      delay(200);
      startNetwork();  // brief blocking only
    }
    return;
  }

  // fail_fast: returns quickly if MQTT isn't up
  aioUp = io.run(0, true) >= AIO_CONNECTED;

  bool retryOk = lastAttemptMs == 0 || now - lastAttemptMs >= PUBLISH_RETRY_MS;

  // Publish the oldest queued event, at most one per MIN_PUBLISH_INTERVAL_MS.
  if (eventCount > 0 && aioUp && retryOk && now - lastPublishMs >= MIN_PUBLISH_INTERVAL_MS) {
    lastAttemptMs = now;
    char json[160];
    buildEventJson(eventQueue[0], json, sizeof(json));
    if (eventFeed->save(json)) {
      logf("Published to Adafruit IO: %s", json);
      lastPublishedState = eventQueue[0].start ? 1 : 0;
      saveLastPublishedState(lastPublishedState);
      lastPublishMs = now;
      memmove(&eventQueue[0], &eventQueue[1], sizeof(DeviceEvent) * (EVENT_QUEUE_SIZE - 1));
      eventCount--;
    }
  }

#ifdef RUN_NTFY
  if (pendingNtfy == lastNtfyState) {
    pendingNtfy = -1;  // flipped back before it was sent: nothing to report
  }
  if (pendingNtfy >= 0 && now - lastNtfyMs >= MIN_PUBLISH_INTERVAL_MS &&
      (lastNtfyAttemptMs == 0 || now - lastNtfyAttemptMs >= PUBLISH_RETRY_MS)) {
    lastNtfyAttemptMs = now;
    // Blocks for the HTTPS request (~1 s); rare, and sampling just resumes.
    bool ok = pendingNtfy == 1
        ? sendNtfy(String(DEVICE_NAME) + " started", String("Started at ") + ntfyEventTime)
        : sendNtfy(String(DEVICE_NAME) + " stopped", String("Stopped at ") + ntfyEventTime +
                   " (ran " + String(ntfyRunSeconds, 1) + " s)");
    if (ok) {
      lastNtfyState = pendingNtfy;
      lastNtfyMs = now;
      pendingNtfy = -1;
    }
  }
#endif
}

// Reboots if WiFi has stayed down too long; auto-reconnect and the retry in
// serviceNetwork() handle brief drops, this is the backstop.
void checkWiFiWatchdog() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiDownSince == 0) {
      wifiDownSince = millis();
    } else if (millis() - wifiDownSince > WIFI_DOWN_REBOOT_MS) {
      restartWithReason("WiFi down watchdog");
    }
  } else {
    wifiDownSince = 0;
  }
}

// Reboots if WiFi is up but Adafruit IO has been unreachable too long.
void checkAioWatchdog() {
  if (WiFi.status() == WL_CONNECTED && !aioUp) {
    if (aioDownSince == 0) {
      aioDownSince = millis();
    } else if (millis() - aioDownSince > AIO_DOWN_REBOOT_MS) {
      restartWithReason("Adafruit IO down watchdog");
    }
  } else {
    aioDownSince = 0;
  }
}

// Reboots once per day at REBOOT_HOUR local time (see TZ_STRING). Held off
// while the device is running or a report is still waiting to be sent, so it
// never drops a run in progress; the whole REBOOT_HOUR hour is the window.
void checkDailyReboot() {
  time_t now = time(nullptr);
  if (now < 100000) return; // NTP hasn't synced yet

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  if (timeinfo.tm_hour != REBOOT_HOUR || timeinfo.tm_mday == lastRebootDay) return;
  if (deviceRunning || eventCount > 0 || pendingNtfy >= 0) return;

  lastRebootDay = timeinfo.tm_mday;
  // Persisted, not just RAM: otherwise the reboot this triggers would see "not
  // rebooted today" again on the next boot (still inside the same hour) and
  // loop for the rest of the hour.
  prefs.begin(NVS_NS, false);
  prefs.putInt("last_reboot_day", lastRebootDay);
  prefs.end();
  restartWithReason(REASON_DAILY_REBOOT);
}

// ==========================================
// Arduino entry points
// ==========================================
void setup(void) {
  Serial.begin(115200);
  delay(2000);               // give USB-CDC time to attach so early messages aren't lost
  Serial.println();

  // Set the timezone before any logging: the RTC survives a soft reset, so
  // time() can already be valid at boot and would otherwise be read as UTC.
  configTzTime(TZ_STRING, NTP_SERVER);

  Serial.println(BANNER);
  Serial.println();

  logf("--- %s monitor v%s ---", DEVICE_NAME, FIRMWARE_VERSION);

  // Opened read-write so the namespace is created on a brand-new device (a
  // read-only open of a missing namespace logs an error on first boot).
  prefs.begin(NVS_NS, false);
  lastRebootDay = prefs.getInt("last_reboot_day", -1);
  lastPublishedState = prefs.getInt("last_state", -1);
  prefs.end();

  checkUnexpectedReset();

  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  setupWiFi();
  setupTime();
  notifyLastReboot();
  setupSensor();

  logf("Monitoring %s vibration...", DEVICE_NAME);
  nextSampleUs = micros();
}

void loop(void) {
  // Sample at a fixed rate.
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_PERIOD_US;
  if ((int32_t)(nowUs - nextSampleUs) > 0) nextSampleUs = nowUs + SAMPLE_PERIOD_US; // fell behind

  float x, y, z;
  readSample(x, y, z);

  float dx = x - bx, dy = y - by, dz = z - bz;
  sumSq += dx * dx + dy * dy + dz * dz;
  bx += BASELINE_ALPHA * dx;
  by += BASELINE_ALPHA * dy;
  bz += BASELINE_ALPHA * dz;

  if (++sampleCount < SAMPLES_PER_WINDOW) return;

  float rms = sqrt(sumSq / sampleCount);
  sumSq = 0;
  sampleCount = 0;

  uint32_t now = millis();
  processWindow(rms, now);
  serviceNetwork(now);
  checkWiFiWatchdog();
  checkAioWatchdog();
  checkSensorWatchdog();
  checkBootStateCorrection(now);
  checkDailyReboot();

  if (now - lastStatusMs >= 1000) {
    lastStatusMs = now;
    logf("rms=%.3f state=%s wifi=%s aio=%s", lastRms,
         deviceRunning ? "RUNNING" : "idle",
         WiFi.status() == WL_CONNECTED ? "up" : "down",
         aioUp ? "up" : "down");
  }
}
