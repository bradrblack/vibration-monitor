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
#define FIRMWARE_VERSION "2026-09-19r1"

// ---- Pins -----------------------------------------------------------------
const int SDA_PIN = 5;
const int SCL_PIN = 6;   // GPIO 8/9 are LED/boot pins on this board

const int  LED_PIN        = 8;
const bool LED_ACTIVE_LOW = true;  // set false if the LED lights when the pin is HIGH
const uint32_t LED_BLINK_HALF_PERIOD_MS = 500;  // blink rate while pump is ON

// ---- Vibration detection --------------------------------------------------
// Samples are high-passed against a slow per-axis baseline (removes gravity and
// mounting angle), then RMS'd over a short window. Tune the thresholds from the
// "rms=" values printed over serial while the pump is idle and running.
const float ON_RMS_THRESHOLD  = 0.40;  // m/s^2 RMS: above this = vibrating
const float OFF_RMS_THRESHOLD = 0.20;  // m/s^2 RMS: below this = quiet (hysteresis)

const uint32_t SAMPLE_PERIOD_US  = 2500;   // 400 Hz, matches accel data rate
const uint16_t SAMPLES_PER_WINDOW = 100;   // 250 ms windows
// High-pass cutoff ~ alpha * fs / 2pi = ~13 Hz: rejects footsteps / house rumble
// (<10 Hz) while passing motor vibration (~29-58 Hz for 1725/3450 RPM).
const float    BASELINE_ALPHA    = 0.2;

// Pump state is debounced so a bump on the pipe isn't counted as a run.
const uint32_t RUN_CONFIRM_MS  = 3000;  // continuously vibrating this long => pump running
const uint32_t STOP_CONFIRM_MS = 3000;  // quiet this long => pump stopped

// ---- Adafruit IO ----------------------------------------------------------
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *pumpFeed = io.feed("pump");
AdafruitIO_Feed *runSecondsFeed = io.feed("pump-run-seconds");
// Free Adafruit IO accounts are rate limited, and state changes should be rare:
// send at most one update per minute, and only if the state really differs from
// what the feed already shows.
const uint32_t MIN_PUBLISH_INTERVAL_MS = 60000;
const uint32_t PUBLISH_RETRY_MS = 10000;

// ---- Reliability ----------------------------------------------------------
// Many ESP32-C3 mini boards have a marginal antenna design and fail to join at
// full TX power; reducing it is the usual fix.
const wifi_power_t WIFI_TX_POWER = WIFI_POWER_8_5dBm;
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000;   // at boot: reboot if no WiFi by then
const uint32_t WIFI_RETRY_MS           = 30000;   // reset the radio and retry this often
const uint32_t WIFI_DOWN_REBOOT_MS     = 120000;  // running: reboot if WiFi down this long
// WiFi can stay associated while Adafruit IO itself is unreachable (DNS/TLS/
// auth problems), which the WiFi watchdog can't see.
const uint32_t AIO_DOWN_REBOOT_MS      = 600000;  // 10 min

// The ADXL345 is polled for its fixed device ID; a wedged or unplugged I2C
// bus otherwise just produces a stream of zeros (looks like a quiet pump).
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
// for a device that's supposed to be watching over a sump pump. Set false to
// only be notified of exception reboots.
#define NOTIFY_DAILY_REBOOT true
#define REASON_DAILY_REBOOT "Daily scheduled reboot"

// Also push pump ON/OFF events to ntfy (for initial testing / as a second
// channel). Same rules as the Adafruit IO feed: at most one per minute, and only
// if the state differs from the last one sent. Requires NTFY_TOPIC.
#define NOTIFY_PUMP_STATE true
#if NOTIFY_PUMP_STATE && defined(NTFY_TOPIC)
#define PUMP_NTFY
#endif

// If the board was restarted while the feed last showed "running", correct it
// once the pump has been observed for this long and is really idle.
const uint32_t BOOT_SETTLE_MS = 15000;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ---- State ----------------------------------------------------------------
float bx, by, bz;              // per-axis baseline
double sumSq = 0;
uint16_t sampleCount = 0;
uint32_t nextSampleUs = 0;

bool ledOn = false;            // instantaneous vibration indicator
bool pumpRunning = false;
uint32_t aboveSince = 0;       // 0 = not currently above ON threshold
uint32_t belowSince = 0;       // 0 = not currently below OFF threshold
uint32_t runStartMs = 0;
uint32_t cycleCount = 0;
uint32_t totalRunMs = 0;

bool aioUp = false;
uint32_t lastWifiAttempt = 0;
uint32_t wifiDownSince = 0;    // 0 = WiFi currently up
uint32_t aioDownSince = 0;     // 0 = Adafruit IO currently up (or WiFi down)
int pendingPublish = -1;       // -1 none, else 0/1 waiting to be sent
int lastPublishedState = -1;   // what the feed currently shows (-1 = unknown); persisted in NVS
// Initialised so the first publish after boot isn't held back by the rate limit.
uint32_t lastPublishMs = (uint32_t)(0 - MIN_PUBLISH_INTERVAL_MS);
uint32_t lastAttemptMs = 0;
float pendingRunSeconds = -1;  // <0 none, else duration of a finished run waiting to be sent
uint32_t lastRunAttemptMs = 0;
uint32_t lastStatusMs = 0;
float lastRms = 0;

int pendingNtfy = -1;          // -1 none, else 0/1 pump state waiting to be pushed
int lastNtfyState = -1;
uint32_t lastNtfyMs = (uint32_t)(0 - MIN_PUBLISH_INTERVAL_MS);
uint32_t lastNtfyAttemptMs = 0;
float ntfyRunSeconds = 0;      // length of the run that ended (for the OFF message)
char ntfyEventTime[32] = "";   // when the pending state change happened

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

// ==========================================
// Reboot reporting (ntfy.sh)
// ==========================================
// Self-triggered reboots would otherwise be silent. The reason is written to
// NVS flash right before restarting, then read back and pushed to ntfy.sh once
// WiFi is up on the next boot. A manual power cycle leaves no reason behind, so
// it stays quiet. Define NTFY_TOPIC in secrets.h to enable pushes.
Preferences prefs;

void recordRebootReason(const char *reason) {
  prefs.begin("pumpctl", false);
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
  prefs.begin("pumpctl", true);
  String reason = prefs.getString("last_reason", "");
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
  prefs.begin("pumpctl", false);
  String reason = prefs.getString("last_reason", "");
  uint32_t count = prefs.getUInt("reboot_count", 0);
  if (reason.length() > 0) {
    prefs.putString("last_reason", ""); // clear so a normal boot stays quiet
  }
  prefs.end();

  if (reason.length() == 0) return;
  if (!NOTIFY_DAILY_REBOOT && reason == REASON_DAILY_REBOOT) return;

  sendNtfy("Pump sensor rebooted",
           reason + " (reboot #" + String(count) + ") at " + logTimestamp());
}

void saveLastPublishedState(int state) {
  prefs.begin("pumpctl", false);
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
      sendNtfy("Pump sensor: accelerometer missing",
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
// quiet pump.
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
// Pump state
// ==========================================
void setPumpState(bool running, uint32_t eventMs) {
  pumpRunning = running;
  if (running) {
    runStartMs = eventMs;
    logf("PUMP ON");
  } else {
    uint32_t dur = eventMs - runStartMs;
    cycleCount++;
    totalRunMs += dur;
    logf("PUMP OFF - ran %.1f s (cycle %lu, total run %.1f s)",
         dur / 1000.0, (unsigned long)cycleCount, totalRunMs / 1000.0);
    pendingRunSeconds = dur / 1000.0f;
    ntfyRunSeconds = dur / 1000.0f;
  }
  pendingPublish = running ? 1 : 0;
#ifdef PUMP_NTFY
  pendingNtfy = running ? 1 : 0;
  strlcpy(ntfyEventTime, logTimestamp(), sizeof(ntfyEventTime));
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
    if (!pumpRunning && now - aboveSince >= RUN_CONFIRM_MS) {
      setPumpState(true, aboveSince);
    }
  } else if (rms < OFF_RMS_THRESHOLD) {
    if (!belowSince) belowSince = now;
    aboveSince = 0;
    if (pumpRunning && now - belowSince >= STOP_CONFIRM_MS) {
      setPumpState(false, belowSince);
    }
  } else {
    // Hysteresis band: neither counts toward starting a run (footsteps are
    // impulsive, a motor is continuous) nor toward ending one.
    aboveSince = 0;
    belowSince = 0;
  }

  // LED: blinks while the pump is considered ON; otherwise solid while any
  // vibration is detected, and fully off when quiet.
  if (pumpRunning) setLed((now / LED_BLINK_HALF_PERIOD_MS) % 2 == 0);
  else setLed(ledOn);
}

// If we restarted (crash, watchdog, nightly reboot...) while the feed last
// showed "running", it would stay stuck on "running" until the next real cycle.
// After a short settle time, if the pump is really idle, correct it.
void checkBootStateCorrection(uint32_t now) {
  if (bootCorrectionDone || now < BOOT_SETTLE_MS) return;
  bootCorrectionDone = true;
  if (lastPublishedState == 1 && !pumpRunning && pendingPublish < 0) {
    logf("Feed showed pump running before restart but it is idle; correcting");
    pendingPublish = 0;
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

  if (pendingPublish == lastPublishedState) {
    pendingPublish = -1;  // state flipped back before it was sent: nothing to report
  }
  if (pendingPublish >= 0) {
    bool rateOk = now - lastPublishMs >= MIN_PUBLISH_INTERVAL_MS;
    if (aioUp && rateOk && retryOk) {
      lastAttemptMs = now;
      if (pumpFeed->save(pendingPublish)) {
        logf("Published pump=%d to Adafruit IO", pendingPublish);
        lastPublishedState = pendingPublish;
        saveLastPublishedState(lastPublishedState);
        lastPublishMs = now;
        pendingPublish = -1;
      }
    }
  }

#ifdef PUMP_NTFY
  if (pendingNtfy == lastNtfyState) {
    pendingNtfy = -1;  // flipped back before it was sent: nothing to report
  }
  if (pendingNtfy >= 0 && now - lastNtfyMs >= MIN_PUBLISH_INTERVAL_MS &&
      (lastNtfyAttemptMs == 0 || now - lastNtfyAttemptMs >= PUBLISH_RETRY_MS)) {
    lastNtfyAttemptMs = now;
    // Blocks for the HTTPS request (~1 s); rare, and sampling just resumes.
    bool ok = pendingNtfy == 1
        ? sendNtfy("Pump ON", String("Pump started at ") + ntfyEventTime)
        : sendNtfy("Pump OFF", String("Pump stopped at ") + ntfyEventTime +
                   " (ran " + String(ntfyRunSeconds, 1) + " s)");
    if (ok) {
      lastNtfyState = pendingNtfy;
      lastNtfyMs = now;
      pendingNtfy = -1;
    }
  }
#endif

  // Exact run length measured on the device (the state feed's timestamps can lag
  // by up to a minute because of the rate limit above).
  if (pendingRunSeconds >= 0 && aioUp &&
      (lastRunAttemptMs == 0 || now - lastRunAttemptMs >= PUBLISH_RETRY_MS)) {
    lastRunAttemptMs = now;
    if (runSecondsFeed->save(pendingRunSeconds, 0, 0, 0, 1)) {
      logf("Published pump-run-seconds=%.1f to Adafruit IO", pendingRunSeconds);
      pendingRunSeconds = -1;
    }
  }
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
// while the pump is running or a report is still waiting to be sent, so it
// never drops a run in progress; the whole REBOOT_HOUR hour is the window.
void checkDailyReboot() {
  time_t now = time(nullptr);
  if (now < 100000) return; // NTP hasn't synced yet

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  if (timeinfo.tm_hour != REBOOT_HOUR || timeinfo.tm_mday == lastRebootDay) return;
  if (pumpRunning || pendingPublish >= 0 || pendingRunSeconds >= 0 || pendingNtfy >= 0) return;

  lastRebootDay = timeinfo.tm_mday;
  // Persisted, not just RAM: otherwise the reboot this triggers would see "not
  // rebooted today" again on the next boot (still inside the same hour) and
  // loop for the rest of the hour.
  prefs.begin("pumpctl", false);
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

  logf("--- Pump sensor v%s ---", FIRMWARE_VERSION);

  prefs.begin("pumpctl", true);
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

  logf("Monitoring pump vibration...");
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
         pumpRunning ? "RUNNING" : "idle",
         WiFi.status() == WL_CONNECTED ? "up" : "down",
         aioUp ? "up" : "down");
  }
}
