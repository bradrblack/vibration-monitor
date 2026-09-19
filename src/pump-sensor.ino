#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_ADXL345_U.h>
#include "AdafruitIO_WiFi.h"
#include "secrets.h"

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
// Many ESP32-C3 mini boards have a marginal antenna design and fail to join at
// full TX power; reducing it is the usual fix.
const wifi_power_t WIFI_TX_POWER = WIFI_POWER_8_5dBm;
const uint32_t WIFI_RETRY_MS = 30000;
const uint32_t WIFI_REBOOT_MS = 300000;  // restart if WiFi has been down this long
// Free Adafruit IO accounts are rate limited, and state changes should be rare:
// send at most one update per minute, and only if the state really differs from
// what the feed already shows.
const uint32_t MIN_PUBLISH_INTERVAL_MS = 60000;
const uint32_t PUBLISH_RETRY_MS = 10000;

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
uint32_t lastWifiUpMs = 0;     // last time WiFi was seen connected (boot counts as up)
int pendingPublish = -1;       // -1 none, else 0/1 waiting to be sent
int lastPublishedState = -1;   // what the feed currently shows (-1 = unknown)
uint32_t lastPublishMs = 0;
uint32_t lastAttemptMs = 0;
float pendingRunSeconds = -1;  // <0 none, else duration of a finished run waiting to be sent
uint32_t lastRunAttemptMs = 0;
uint32_t lastStatusMs = 0;
float lastRms = 0;

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

void setPumpState(bool running, uint32_t eventMs) {
  pumpRunning = running;
  if (running) {
    runStartMs = eventMs;
    Serial.println("PUMP ON");
  } else {
    uint32_t dur = eventMs - runStartMs;
    cycleCount++;
    totalRunMs += dur;
    Serial.printf("PUMP OFF - ran %.1f s (cycle %lu, total run %.1f s)\n",
                  dur / 1000.0, (unsigned long)cycleCount, totalRunMs / 1000.0);
    pendingRunSeconds = dur / 1000.0f;
  }
  pendingPublish = running ? 1 : 0;
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

// Bring the radio up in station mode first (io.connect() alone tears down a
// radio that was never started, which logs ESP_ERR_WIFI_NOT_INIT and can leave
// it unable to join), then let Adafruit IO start the connection.
void startNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_TX_POWER);
  WiFi.setAutoReconnect(true);
  io.connect();
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
    // Last resort for an unattended device: reboot if offline too long.
    if (now - lastWifiUpMs >= WIFI_REBOOT_MS) {
      Serial.println("WiFi down too long, restarting");
      delay(100);
      ESP.restart();
    }
    return;
  }

  lastWifiUpMs = now;

  // fail_fast: returns quickly if MQTT isn't up
  aioUp = io.run(0, true) >= AIO_CONNECTED;

  bool retryOk = lastAttemptMs == 0 || now - lastAttemptMs >= PUBLISH_RETRY_MS;

  if (pendingPublish == lastPublishedState) {
    pendingPublish = -1;  // state flipped back before it was sent: nothing to report
  }
  if (pendingPublish >= 0) {
    bool rateOk = lastPublishedState < 0 || now - lastPublishMs >= MIN_PUBLISH_INTERVAL_MS;
    if (aioUp && rateOk && retryOk) {
      lastAttemptMs = now;
      if (pumpFeed->save(pendingPublish)) {
        Serial.printf("Published pump=%d to Adafruit IO\n", pendingPublish);
        lastPublishedState = pendingPublish;
        lastPublishMs = now;
        pendingPublish = -1;
      }
    }
  }

  // Exact run length measured on the device (the state feed's timestamps can lag
  // by up to a minute because of the rate limit above).
  if (pendingRunSeconds >= 0 && aioUp &&
      (lastRunAttemptMs == 0 || now - lastRunAttemptMs >= PUBLISH_RETRY_MS)) {
    lastRunAttemptMs = now;
    if (runSecondsFeed->save(pendingRunSeconds, 0, 0, 0, 1)) {
      Serial.printf("Published pump-run-seconds=%.1f to Adafruit IO\n", pendingRunSeconds);
      pendingRunSeconds = -1;
    }
  }
}

void setup(void) {
  Serial.begin(115200);
  delay(2000);               // give USB-CDC time to attach so early messages aren't lost

  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  while (!accel.begin()) {
    Serial.println("No ADXL345 detected. Check wiring!");
    delay(1000);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_400_HZ);

  readSample(bx, by, bz);

  startNetwork();
  lastWifiAttempt = millis();

  Serial.println("ADXL345 initialized. Monitoring pump vibration...");
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


  if (now - lastStatusMs >= 1000) {
    lastStatusMs = now;
    Serial.printf("rms=%.3f state=%s wifi=%s aio=%s\n", lastRms,
                  pumpRunning ? "RUNNING" : "idle",
                  WiFi.status() == WL_CONNECTED ? "up" : "down",
                  aioUp ? "up" : "down");
  }
}
