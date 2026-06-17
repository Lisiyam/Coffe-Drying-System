#include <Arduino.h>
#include <DHT.h>
#include <Preferences.h>
#include <ThingSpeak.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <math.h>
#include <stdlib.h>

// ================= PIN CONFIG =================
const byte DHT_PIN = 26;
const byte SCK_PIN = 23;
const byte NUM_LOADCELLS = 4;
const byte DOUT_PINS[NUM_LOADCELLS] = {5, 4, 19, 18};
const byte HEATER_PIN = 27;
const byte LED_RED_PIN = 13;
const byte LED_YELLOW_PIN = 14;
const byte LED_GREEN_PIN = 22;
const byte BUTTON_PIN = 33;

// ================= WIFI MANAGER + THINGSPEAK CONFIG =================
// Nama AP dan password saat mode konfigurasi
const char *AP_NAME     = "CORDY-Acces Point";
const char *AP_PASSWORD = "Cordy Cihuy";

// Timeout portal konfigurasi (detik) — setelah ini ESP restart
const int WIFI_PORTAL_TIMEOUT_S = 180;

// Interval coba reconnect saat WiFi putus (10 detik)
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 10000UL;

// Jika reconnect terus gagal selama X menit, baru buka portal
const unsigned long WIFI_FAIL_PORTAL_MS = 5UL * 60UL * 1000UL; // 5 menit

// State WiFi internal
volatile bool wifiConnected       = false; // diupdate oleh event handler
unsigned long wifiDisconnectedSinceMs = 0; // kapan pertama kali putus
unsigned long lastReconnectAttemptMs  = 0; // kapan terakhir coba reconnect
bool wifiWasConnected = false;

// Namespace NVS untuk menyimpan konfigurasi
const char *NVS_NAMESPACE = "cordy_cfg";

// Variabel runtime konfigurasi ThingSpeak (dibaca dari NVS saat boot)
char cfg_ts_channel_id[16]  = "0";
char cfg_ts_api_key[33]     = "";

Preferences preferences;
WiFiManager wifiManager;
WiFiClient  wifiClient;

// Custom parameter WiFiManager untuk ThingSpeak
WiFiManagerParameter param_ts_channel_id("ts_ch_id",  "ThingSpeak Channel ID",  cfg_ts_channel_id, 15);
WiFiManagerParameter param_ts_api_key   ("ts_api_key","ThingSpeak Write API Key",cfg_ts_api_key,    32);

const unsigned long THINGSPEAK_INTERVAL_MS = 20000;
unsigned long lastThingSpeakSendMs = 0;

// ================= DHT =================
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);
const unsigned long DHT_READ_INTERVAL_MS = 2000;
unsigned long lastDhtReadMs = 0;
float currentTemperatureC = NAN;
float currentHumidity     = NAN;

// ================= PID =================
const double TEMPERATURE_SETPOINT_C = 38.0;
double kp = 35.0;
double ki = 0.50;
double kd = 0.0;

double pidIntegral    = 0.0;
double previousError  = 0.0;
unsigned long previousPidMs = 0;
int heaterDuty = 0;

const int PWM_FREQ         = 1000;
const int PWM_RESOLUTION   = 8;
const int PWM_MAX_DUTY     = 255;
const int HEATER_PWM_CHANNEL = 0;

// ================= LOAD CELL =================
float calibrationFactor[NUM_LOADCELLS] = {
    1200.9980426022,
    1200.9261370178,
    1200.9648819804,
    1200.9690846287};

long tareOffset[NUM_LOADCELLS] = {0, 0, 0, 0};

const byte  RAW_SAMPLES   = 5;
const byte  TARE_SAMPLES  = 30;
const byte  MEDIAN_SIZE   = 5;
const float EMA_ALPHA     = 0.4;
const float ZERO_DEADBAND_GRAM = 2.0;

const long THRESHOLD_GILA_RAW   = 800000;
const long HX711_SATURATION_RAW = 8300000;

const bool  AUTO_ZERO_TRACKING         = true;
const float AUTO_ZERO_WINDOW_GRAM      = 8.0;
const float AUTO_ZERO_STABLE_DELTA_GRAM = 2.0;
const float AUTO_ZERO_ALPHA            = 0.02;
const byte  AUTO_ZERO_CONFIRM_READS    = 8;

const float RELEASE_TO_ZERO_WINDOW_GRAM    = 12.0;
const float RELEASE_FILTER_MEMORY_MIN_GRAM = 20.0;
const byte  RELEASE_TO_ZERO_CONFIRM_READS  = 3;

float medianBuffer[MEDIAN_SIZE];
byte  medianIndex  = 0;
bool  medianFilled = false;

float filteredWeightGram       = 0.0;
float currentWeightGram        = NAN;
bool  filterStarted            = false;
float lastUnfilteredWeightGram = 0.0;
byte  autoZeroCounter          = 0;
byte  releaseToZeroCounter     = 0;
bool  loadCellReady            = false;

const unsigned long WEIGHT_READ_INTERVAL_MS = 250;
unsigned long lastWeightReadMs = 0;

// ================= STATE =================
enum SystemState {
  STATE_IDLE,
  STATE_DRYING,
  STATE_FINISHED
};

enum DrynessStatus {
  DRYNESS_UNKNOWN,
  DRYNESS_WET,
  DRYNESS_FAIRLY_DRY,
  DRYNESS_READY_TO_STORE,
  DRYNESS_TOO_DRY,
  DRYNESS_BELOW_TARGET
};

SystemState   systemState  = STATE_IDLE;
DrynessStatus drynessStatus = DRYNESS_UNKNOWN;

// ================= BUTTON =================
const unsigned long BUTTON_DEBOUNCE_MS   = 50;
const unsigned long BUTTON_HOLD_RESET_MS = 3000;
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
bool longPressHandled  = false;
unsigned long lastButtonChangeMs = 0;
unsigned long buttonPressedAtMs  = 0;

// ================= SERIAL LOG =================
const unsigned long SERIAL_PRINT_INTERVAL_MS = 1000;
unsigned long lastSerialPrintMs = 0;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void updateButton();
void tareScale();
void stopHeater();
void updateIndicators();
void openConfigPortal();
void onWiFiEvent(WiFiEvent_t event);

// ============================================================
// NVS — SIMPAN & BACA KONFIGURASI
// ============================================================

/**
 * Muat konfigurasi dari NVS (flash).
 * Jika belum pernah disimpan, nilai default tetap dipakai.
 */
void loadConfigFromNVS() {
  preferences.begin(NVS_NAMESPACE, true); // read-only
  String ch  = preferences.getString("ts_ch_id",  "0");
  String key = preferences.getString("ts_api_key", "");
  preferences.end();

  ch.toCharArray(cfg_ts_channel_id, sizeof(cfg_ts_channel_id));
  key.toCharArray(cfg_ts_api_key,   sizeof(cfg_ts_api_key));

  Serial.print("Konfigurasi dimuat — Channel ID: ");
  Serial.print(cfg_ts_channel_id);
  Serial.print(" | API Key: ");
  Serial.println(cfg_ts_api_key);
}

/**
 * Simpan konfigurasi ke NVS.
 * Dipanggil setelah portal WiFiManager berhasil menyimpan.
 */
void saveConfigToNVS() {
  // Ambil nilai terbaru dari parameter WiFiManager
  strncpy(cfg_ts_channel_id, param_ts_channel_id.getValue(), sizeof(cfg_ts_channel_id) - 1);
  strncpy(cfg_ts_api_key,    param_ts_api_key.getValue(),    sizeof(cfg_ts_api_key)    - 1);

  preferences.begin(NVS_NAMESPACE, false); // read-write
  preferences.putString("ts_ch_id",  cfg_ts_channel_id);
  preferences.putString("ts_api_key", cfg_ts_api_key);
  preferences.end();

  Serial.println("Konfigurasi ThingSpeak disimpan ke NVS.");
  Serial.print("  Channel ID : "); Serial.println(cfg_ts_channel_id);
  Serial.print("  API Key    : "); Serial.println(cfg_ts_api_key);
}

// ============================================================
// WIFI MANAGER
// ============================================================

/**
 * Callback — dipanggil WiFiManager setelah user klik Save di portal.
 * Kita simpan parameter ThingSpeak ke NVS di sini.
 */
void onWiFiManagerSave() {
  saveConfigToNVS();
  // Re-init ThingSpeak supaya pakai config terbaru dari portal
  ThingSpeak.begin(wifiClient);
  Serial.println("ThingSpeak di-reinit dengan konfigurasi baru.");
}

/**
 * Buka portal konfigurasi (blocking sampai user selesai atau timeout).
 * Setelah keluar dari fungsi ini, ESP sudah terhubung ke WiFi BARU
 * atau di-restart otomatis bila timeout.
 */
void openConfigPortal() {
  Serial.println("Membuka portal konfigurasi WiFi...");
  Serial.print("Hubungkan HP/laptop ke AP: ");
  Serial.println(AP_NAME);
  Serial.println("Lalu buka browser ke 192.168.4.1");

  // Semua LED nyala sebagai tanda mode konfigurasi
  digitalWrite(LED_RED_PIN,    HIGH);
  digitalWrite(LED_YELLOW_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN,  HIGH);

  stopHeater(); // Pastikan heater mati selama konfigurasi

  wifiManager.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT_S);
  wifiManager.setSaveConfigCallback(onWiFiManagerSave);

  // Perbarui nilai default parameter dari nilai yang sudah tersimpan
  param_ts_channel_id.setValue(cfg_ts_channel_id, 15);
  param_ts_api_key.setValue(cfg_ts_api_key, 23);

  bool connected = wifiManager.startConfigPortal(AP_NAME, AP_PASSWORD);

  if (connected) {
    Serial.print("WiFi terhubung via portal. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Portal timeout. Merestart ESP...");
    delay(1000);
    ESP.restart();
  }

  // Kembalikan indikator LED ke kondisi sistem saat ini
  updateIndicators();
}

/**
 * Inisialisasi WiFiManager saat boot.
 * - Tambahkan custom parameter ThingSpeak.
 * - Coba sambung ke WiFi tersimpan; jika gagal, buka portal.
 */
void setupWiFiManager() {
  // Daftarkan event handler sebelum apapun
  WiFi.onEvent(onWiFiEvent);

  wifiManager.addParameter(&param_ts_channel_id);
  wifiManager.addParameter(&param_ts_api_key);
  wifiManager.setSaveConfigCallback(onWiFiManagerSave);
  wifiManager.setConnectTimeout(30);
  wifiManager.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT_S);

  // Aktifkan auto-reconnect bawaan ESP32 sebagai lapis pertama
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true); // simpan kredensial ke flash ESP32

  Serial.println("Menghubungkan WiFi (WiFiManager)...");

  bool connected = wifiManager.autoConnect(AP_NAME, AP_PASSWORD);

  if (connected) {
    Serial.print("WiFi terhubung. IP: ");
    Serial.println(WiFi.localIP());
    wifiConnected    = true;
    wifiWasConnected = true;
  } else {
    Serial.println("WiFi tidak terhubung setelah setup.");
  }
}

/**
 * WiFi Event Handler — dipanggil oleh ESP32 secara otomatis saat
 * status koneksi berubah (tanpa polling).
 */
void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifiConnected           = true;
      wifiWasConnected        = true;
      wifiDisconnectedSinceMs = 0;
      lastReconnectAttemptMs  = 0;
      Serial.print("[WiFi] Terhubung! IP: ");
      Serial.println(WiFi.localIP());
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      if (wifiConnected) {
        // Baru saja putus — catat waktu
        wifiDisconnectedSinceMs = millis();
        Serial.println("[WiFi] Koneksi terputus. Akan mencoba reconnect...");
      }
      wifiConnected = false;
      // Langsung minta ESP32 coba reconnect segera
      WiFi.reconnect();
      break;

    default:
      break;
  }
}

/**
 * Monitor koneksi WiFi di loop().
 * Strategi berlapis:
 *   1. Event handler sudah trigger WiFi.reconnect() saat putus.
 *   2. Polling setiap WIFI_RECONNECT_INTERVAL_MS sebagai fallback
 *      jika event tidak ter-trigger (mis. router restart).
 *   3. Jika gagal lebih dari WIFI_FAIL_PORTAL_MS → buka portal ulang.
 */
void checkWiFiConnection() {
  // Sudah terhubung — tidak ada yang perlu dilakukan
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiConnected) {
      // Tangkap kondisi terhubung yang terlewat event
      wifiConnected           = true;
      wifiWasConnected        = true;
      wifiDisconnectedSinceMs = 0;
      lastReconnectAttemptMs  = 0;
    }
    return;
  }

  unsigned long now = millis();

  // Catat waktu pertama kali putus
  if (wifiDisconnectedSinceMs == 0) {
    wifiDisconnectedSinceMs = now;
    Serial.println("[WiFi] Tidak terhubung. Memulai auto-reconnect...");
  }

  unsigned long disconnectedMs = now - wifiDisconnectedSinceMs;

  // --- Fallback polling reconnect ---
  // Setiap WIFI_RECONNECT_INTERVAL_MS, paksa reconnect ke SSID tersimpan
  if (now - lastReconnectAttemptMs >= WIFI_RECONNECT_INTERVAL_MS) {
    lastReconnectAttemptMs = now;
    Serial.print("[WiFi] Mencoba reconnect ke \"");
    Serial.print(WiFi.SSID());
    Serial.print("\" ... (putus ");
    Serial.print(disconnectedMs / 1000);
    Serial.println(" detik)");

    WiFi.disconnect(false);   // putus tanpa hapus kredensial
    delay(100);
    WiFi.reconnect();         // sambung ulang ke SSID tersimpan
  }

  // --- Buka portal jika terlalu lama gagal ---
  if (disconnectedMs >= WIFI_FAIL_PORTAL_MS) {
    Serial.println("[WiFi] Gagal reconnect selama 5 menit. Membuka portal konfigurasi...");
    wifiDisconnectedSinceMs = 0;
    lastReconnectAttemptMs  = 0;

    // Jangan hapus resetSettings() agar SSID tersimpan tetap ada
    // (portal hanya dipakai untuk ubah jika memang mau ganti)
    openConfigPortal();
    ThingSpeak.begin(wifiClient);
  }
}

// ============================================================
// HELPER
// ============================================================

bool isThingSpeakConfigured() {
  unsigned long chId = strtoul(cfg_ts_channel_id, nullptr, 10);
  return chId > 0 && strlen(cfg_ts_api_key) > 0 &&
         strcmp(cfg_ts_api_key, "ISI_WRITE_API_KEY") != 0;
}

const char *systemStateToText(SystemState state) {
  switch (state) {
  case STATE_IDLE:     return "idle";
  case STATE_DRYING:   return "dalam pengeringan";
  case STATE_FINISHED: return "pengeringan selesai";
  default:             return "unknown";
  }
}

const char *drynessStatusToText(DrynessStatus status) {
  switch (status) {
  case DRYNESS_WET:            return "basah";
  case DRYNESS_FAIRLY_DRY:     return "cukup kering";
  case DRYNESS_READY_TO_STORE: return "kering siap disimpan";
  case DRYNESS_TOO_DRY:        return "terlalu kering";
  case DRYNESS_BELOW_TARGET:   return "di bawah target";
  default:                     return "unknown";
  }
}

DrynessStatus classifyDryness(float weightGram) {
  if (isnan(weightGram))      return DRYNESS_UNKNOWN;
  if (weightGram > 400.0)     return DRYNESS_WET;
  if (weightGram >= 320.0)    return DRYNESS_FAIRLY_DRY;
  if (weightGram >= 284.0)    return DRYNESS_READY_TO_STORE;
  if (weightGram < 260.0)     return DRYNESS_TOO_DRY;
  return DRYNESS_BELOW_TARGET;
}

// ============================================================
// HEATER / PWM
// ============================================================

void setupHeaterPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(HEATER_PIN, PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(HEATER_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, HEATER_PWM_CHANNEL);
#endif
}

void writeHeaterPwm(int duty) {
  heaterDuty = constrain(duty, 0, PWM_MAX_DUTY);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(HEATER_PIN, heaterDuty);
#else
  ledcWrite(HEATER_PWM_CHANNEL, heaterDuty);
#endif
}

void stopHeater() { writeHeaterPwm(0); }

// ============================================================
// PID
// ============================================================

void resetPid() {
  pidIntegral   = 0.0;
  previousError = 0.0;
  previousPidMs = millis();
}

// ============================================================
// LED INDIKATOR
// ============================================================

void updateIndicators() {
  digitalWrite(LED_RED_PIN,    systemState == STATE_IDLE     ? HIGH : LOW);
  digitalWrite(LED_YELLOW_PIN, systemState == STATE_DRYING   ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN,  systemState == STATE_FINISHED ? HIGH : LOW);
}

void setSystemState(SystemState nextState) {
  if (systemState == nextState) return;

  systemState = nextState;
  updateIndicators();

  if (systemState == STATE_DRYING) {
    resetPid();
    Serial.println("Sistem masuk state: dalam pengeringan");
    return;
  }

  stopHeater();
  resetPid();
  Serial.print("Sistem masuk state: ");
  Serial.println(systemStateToText(systemState));
}

// ============================================================
// LOAD CELL
// ============================================================

bool isAllLoadCellsReady() {
  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    if (digitalRead(DOUT_PINS[i]) == HIGH) return false;
  }
  return true;
}

bool waitUntilLoadCellsReady(unsigned long timeoutMs) {
  unsigned long startMs = millis();
  while (!isAllLoadCellsReady()) {
    updateButton();
    if (millis() - startMs >= timeoutMs) return false;
    delay(2);
  }
  return true;
}

void printNotReadySensors() {
  Serial.print("HX711 belum ready: ");
  bool first = true;
  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    if (digitalRead(DOUT_PINS[i]) == HIGH) {
      if (!first) Serial.print(", ");
      Serial.print("LC"); Serial.print(i + 1);
      Serial.print("(DT GPIO "); Serial.print(DOUT_PINS[i]); Serial.print(")");
      first = false;
    }
  }
  if (first) Serial.print("tidak ada");
  Serial.println();
}

bool readRawParallel(long rawValues[NUM_LOADCELLS]) {
  unsigned long values[NUM_LOADCELLS] = {0, 0, 0, 0};
  if (!waitUntilLoadCellsReady(1000)) return false;

  noInterrupts();
  for (byte bitIndex = 0; bitIndex < 24; bitIndex++) {
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(1);
    for (byte i = 0; i < NUM_LOADCELLS; i++) {
      values[i] <<= 1;
      if (digitalRead(DOUT_PINS[i]) == HIGH) values[i]++;
    }
    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(SCK_PIN, HIGH); delayMicroseconds(1);
  digitalWrite(SCK_PIN, LOW);  delayMicroseconds(1);
  interrupts();

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    if (values[i] & 0x800000) values[i] |= 0xFF000000;
    rawValues[i] = (long)values[i];
  }
  return true;
}

bool readAverageRaw(long averagedRaw[NUM_LOADCELLS], byte samples) {
  long   raw[NUM_LOADCELLS];
  double sum[NUM_LOADCELLS] = {0, 0, 0, 0};

  for (byte sample = 0; sample < samples; sample++) {
    if (!readRawParallel(raw)) return false;
    for (byte i = 0; i < NUM_LOADCELLS; i++) sum[i] += raw[i];
  }
  for (byte i = 0; i < NUM_LOADCELLS; i++) averagedRaw[i] = (long)(sum[i] / samples);
  return true;
}

void resetWeightFilterToZero() {
  for (byte i = 0; i < MEDIAN_SIZE; i++) medianBuffer[i] = 0.0;
  medianIndex        = 0;
  medianFilled       = false;
  filteredWeightGram = 0.0;
  currentWeightGram  = 0.0;
  filterStarted      = true;
}

void tareScale() {
  long raw[NUM_LOADCELLS];
  Serial.println("Tare load cell dimulai. Pastikan wadah kosong.");
  if (!readAverageRaw(raw, TARE_SAMPLES)) {
    Serial.println("Tare gagal: HX711 tidak ready.");
    printNotReadySensors();
    loadCellReady = false;
    return;
  }
  for (byte i = 0; i < NUM_LOADCELLS; i++) tareOffset[i] = raw[i];
  medianIndex            = 0;
  medianFilled           = false;
  filteredWeightGram     = 0.0;
  currentWeightGram      = 0.0;
  filterStarted          = false;
  lastUnfilteredWeightGram = 0.0;
  autoZeroCounter        = 0;
  releaseToZeroCounter   = 0;
  loadCellReady          = true;
  Serial.println("Tare selesai.");
}

float getMedian(float newValue) {
  medianBuffer[medianIndex++] = newValue;
  if (medianIndex >= MEDIAN_SIZE) { medianIndex = 0; medianFilled = true; }

  byte  count = medianFilled ? MEDIAN_SIZE : medianIndex;
  float sorted[MEDIAN_SIZE];
  for (byte i = 0; i < count; i++) sorted[i] = medianBuffer[i];
  for (byte i = 0; i < count - 1; i++)
    for (byte j = i + 1; j < count; j++)
      if (sorted[j] < sorted[i]) { float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }
  return sorted[count / 2];
}

float updateWeightFilter(float weightGram) {
  float medianWeight = getMedian(weightGram);
  if (!filterStarted) {
    filteredWeightGram = medianWeight;
    filterStarted = true;
  } else {
    filteredWeightGram = (EMA_ALPHA * medianWeight) + ((1.0 - EMA_ALPHA) * filteredWeightGram);
  }
  if (fabs(filteredWeightGram) < ZERO_DEADBAND_GRAM) filteredWeightGram = 0.0;
  return filteredWeightGram;
}

bool hasSensorAnomaly(const long raw[NUM_LOADCELLS]) {
  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    long netRaw = raw[i] - tareOffset[i];
    if (labs(netRaw) > THRESHOLD_GILA_RAW || labs(raw[i]) > HX711_SATURATION_RAW) return true;
  }
  return false;
}

float calculateTotalWeightGram(const long raw[NUM_LOADCELLS]) {
  float total = 0.0;
  for (byte i = 0; i < NUM_LOADCELLS; i++)
    total += (raw[i] - tareOffset[i]) / calibrationFactor[i];
  return total;
}

bool updateAutoZeroTracking(const long raw[NUM_LOADCELLS], float totalWeightGram) {
  if (!AUTO_ZERO_TRACKING) return false;
  float movementGram = fabs(totalWeightGram - lastUnfilteredWeightGram);
  bool  nearZero     = fabs(totalWeightGram) <= AUTO_ZERO_WINDOW_GRAM;
  bool  stable       = movementGram <= AUTO_ZERO_STABLE_DELTA_GRAM;
  lastUnfilteredWeightGram = totalWeightGram;
  if (!nearZero || !stable) { autoZeroCounter = 0; return false; }
  if (autoZeroCounter < AUTO_ZERO_CONFIRM_READS) { autoZeroCounter++; return false; }
  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    float offsetCorrection = (raw[i] - tareOffset[i]) * AUTO_ZERO_ALPHA;
    tareOffset[i] += lround(offsetCorrection);
  }
  return true;
}

bool shouldReleaseToZero(float totalWeightGram) {
  bool rawBackToZero       = fabs(totalWeightGram) <= RELEASE_TO_ZERO_WINDOW_GRAM;
  bool filterStillHasLoad  = fabs(filteredWeightGram) >= RELEASE_FILTER_MEMORY_MIN_GRAM;
  if (!rawBackToZero || !filterStillHasLoad) { releaseToZeroCounter = 0; return false; }
  if (releaseToZeroCounter < RELEASE_TO_ZERO_CONFIRM_READS) { releaseToZeroCounter++; return false; }
  return true;
}

void updateWeightReading() {
  long raw[NUM_LOADCELLS];
  if (!readAverageRaw(raw, RAW_SAMPLES)) {
    loadCellReady = false;
    Serial.println("HX711 tidak ready. Cek koneksi DT/SCK.");
    printNotReadySensors();
    return;
  }
  loadCellReady = true;
  if (hasSensorAnomaly(raw)) { Serial.println("Peringatan: anomali sensor load cell terdeteksi."); return; }
  float totalWeightGram = calculateTotalWeightGram(raw);
  updateAutoZeroTracking(raw, totalWeightGram);
  totalWeightGram = calculateTotalWeightGram(raw);
  if (shouldReleaseToZero(totalWeightGram)) resetWeightFilterToZero();
  else currentWeightGram = updateWeightFilter(totalWeightGram);
  drynessStatus = classifyDryness(currentWeightGram);
}

// ============================================================
// DHT
// ============================================================

void updateDhtReading() {
  float temperature = dht.readTemperature();
  float humidity    = dht.readHumidity();
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT gagal dibaca. Heater dimatikan sementara.");
    currentTemperatureC = NAN;
    currentHumidity     = NAN;
    stopHeater();
    return;
  }
  currentTemperatureC = temperature;
  currentHumidity     = humidity;
}

// ============================================================
// PID HEATER
// ============================================================

void updatePidHeater() {
  if (systemState != STATE_DRYING || isnan(currentTemperatureC)) { stopHeater(); return; }
  unsigned long now = millis();
  double dt = (now - previousPidMs) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  previousPidMs = now;
  double error  = TEMPERATURE_SETPOINT_C - currentTemperatureC;
  pidIntegral  += error * dt;
  pidIntegral   = constrain(pidIntegral, -100.0, 100.0);
  double derivative = (error - previousError) / dt;
  previousError     = error;
  double output = (kp * error) + (ki * pidIntegral) + (kd * derivative);
  writeHeaterPwm((int)constrain(output, 0.0, (double)PWM_MAX_DUTY));
}

// ============================================================
// BUTTON
// ============================================================

void handleShortButtonPress() {
  if (systemState == STATE_IDLE) {
    Serial.println("Tombol ditekan: mulai pengeringan.");
    setSystemState(STATE_DRYING);
  } else if (systemState == STATE_FINISHED) {
    Serial.println("Tombol ditekan: kembali ke idle.");
    setSystemState(STATE_IDLE);
  }
}

void handleLongButtonPress() {
  if (systemState == STATE_DRYING || systemState == STATE_FINISHED) {
    Serial.println("Tombol ditahan 3 detik: reset ke idle.");
    setSystemState(STATE_IDLE);
  }
}

void updateButton() {
  bool          reading = digitalRead(BUTTON_PIN);
  unsigned long now     = millis();

  if (reading != lastButtonReading) {
    lastButtonChangeMs = now;
    lastButtonReading  = reading;
    Serial.print("Raw tombol: ");
    Serial.println(reading == LOW ? "LOW/ditekan" : "HIGH/dilepas");
  }

  if ((now - lastButtonChangeMs) < BUTTON_DEBOUNCE_MS || reading == stableButtonState) {
    if (stableButtonState == LOW && !longPressHandled &&
        (now - buttonPressedAtMs) >= BUTTON_HOLD_RESET_MS) {
      longPressHandled = true;
      handleLongButtonPress();
    }
    return;
  }

  stableButtonState = reading;
  if (stableButtonState == LOW) {
    buttonPressedAtMs = now;
    longPressHandled  = false;
    handleShortButtonPress();
  } else {
    Serial.println("Tombol dilepas.");
  }

  if (stableButtonState == LOW && !longPressHandled &&
      (now - buttonPressedAtMs) >= BUTTON_HOLD_RESET_MS) {
    longPressHandled = true;
    handleLongButtonPress();
  }
}

// ============================================================
// THINGSPEAK
// ============================================================

void sendThingSpeakIfDue() {
  if (!isThingSpeakConfigured()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - lastThingSpeakSendMs < THINGSPEAK_INTERVAL_MS) return;
  lastThingSpeakSendMs = now;

  unsigned long channelId = strtoul(cfg_ts_channel_id, nullptr, 10);

  ThingSpeak.setField(1, currentWeightGram);
  ThingSpeak.setField(2, currentTemperatureC);
  ThingSpeak.setField(3, currentHumidity);
  ThingSpeak.setField(4, String(drynessStatusToText(drynessStatus)));
  ThingSpeak.setField(5, String(systemStateToText(systemState)));

  String statusText = String("kopi=") + drynessStatusToText(drynessStatus) +
                      ", sistem=" + systemStateToText(systemState);
  ThingSpeak.setStatus(statusText);

  int responseCode = ThingSpeak.writeFields(channelId, cfg_ts_api_key);
  if (responseCode == 200) {
    Serial.println("Data berhasil dikirim ke ThingSpeak.");
  } else {
    Serial.print("Gagal kirim ThingSpeak. Kode: ");
    Serial.println(responseCode);
  }
}

// ============================================================
// STATE OTOMATIS
// ============================================================

void updateAutomaticState() {
  if (systemState == STATE_DRYING && drynessStatus == DRYNESS_READY_TO_STORE) {
    setSystemState(STATE_FINISHED);
  }
}

// ============================================================
// TELEMETRY SERIAL
// ============================================================

void printTelemetry() {
  Serial.print("State=");      Serial.print(systemStateToText(systemState));
  Serial.print(" | Berat=");
  isnan(currentWeightGram)   ? Serial.print("nan") : Serial.print(currentWeightGram, 1);
  Serial.print(" g | Status kopi="); Serial.print(drynessStatusToText(drynessStatus));
  Serial.print(" | Suhu=");
  isnan(currentTemperatureC) ? Serial.print("nan") : Serial.print(currentTemperatureC, 1);
  Serial.print(" C | Kelembapan=");
  isnan(currentHumidity)     ? Serial.print("nan") : Serial.print(currentHumidity, 1);
  Serial.print(" % | PWM=");   Serial.println(heaterDuty);
}

// ============================================================
// SERIAL COMMANDS
// ============================================================

void handleSerialCommands() {
  if (!Serial.available()) return;
  char command = Serial.read();

  if (command == 't' || command == 'T') {
    tareScale();
  } else if (command == 's' || command == 'S') {
    if (systemState == STATE_IDLE) setSystemState(STATE_DRYING);
  } else if (command == 'i' || command == 'I') {
    setSystemState(STATE_IDLE);
  } else if (command == 'r' || command == 'R') {
    // Perintah manual reset konfigurasi WiFi via Serial
    Serial.println("Reset konfigurasi WiFi via serial. Membuka portal...");
    wifiManager.resetSettings();
    openConfigPortal();
    ThingSpeak.begin(wifiClient);
  } else if (command == 'c' || command == 'C') {
    // Tampilkan konfigurasi tersimpan
    Serial.println("=== Konfigurasi Tersimpan ===");
    Serial.print("WiFi SSID  : "); Serial.println(WiFi.SSID());
    Serial.print("Channel ID : "); Serial.println(cfg_ts_channel_id);
    Serial.print("API Key    : "); Serial.println(cfg_ts_api_key);
    Serial.println("============================");
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // GPIO
  pinMode(SCK_PIN, OUTPUT);
  digitalWrite(SCK_PIN, LOW);
  for (byte i = 0; i < NUM_LOADCELLS; i++) pinMode(DOUT_PINS[i], INPUT);
  pinMode(LED_RED_PIN,    OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN,  OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  lastButtonReading  = digitalRead(BUTTON_PIN);
  stableButtonState  = lastButtonReading;
  lastButtonChangeMs = millis();

  dht.begin();
  setupHeaterPwm();
  stopHeater();
  updateIndicators();
  resetPid();

  Serial.println();
  Serial.println("=========================================");
  Serial.println(" Sistem Pengering Kopi — CORDY");
  Serial.println(" PID + Load Cell + ThingSpeak + WiFiManager");
  Serial.println("=========================================");
  Serial.println("Field ThingSpeak: 1=berat, 2=suhu, 3=kelembapan, 4=status kopi, 5=status sistem.");
  Serial.println("Serial: t=tare | s=start | i=idle | r=reset WiFi | c=lihat konfigurasi");
  Serial.println();

  // 1. Muat konfigurasi ThingSpeak dari NVS
  loadConfigFromNVS();

  // 2. Inisialisasi WiFiManager (sambung WiFi atau buka portal)
  setupWiFiManager();

  // 3. Inisialisasi ThingSpeak setelah WiFi tersambung
  ThingSpeak.begin(wifiClient);

  // 4. Siapkan sensor
  tareScale();
  updateDhtReading();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  unsigned long now = millis();

  updateButton();
  handleSerialCommands();

  // Monitor koneksi WiFi — buka portal ulang jika putus terlalu lama
  checkWiFiConnection();

  if (now - lastWeightReadMs >= WEIGHT_READ_INTERVAL_MS) {
    lastWeightReadMs = now;
    updateWeightReading();
  }

  if (now - lastDhtReadMs >= DHT_READ_INTERVAL_MS) {
    lastDhtReadMs = now;
    updateDhtReading();
  }

  updateAutomaticState();
  updatePidHeater();
  sendThingSpeakIfDue();

  if (now - lastSerialPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
    lastSerialPrintMs = now;
    printTelemetry();
  }
}
