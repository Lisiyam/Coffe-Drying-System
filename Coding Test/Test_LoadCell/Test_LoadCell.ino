#include <math.h>
#include <stdlib.h>
#include "KekeringanKopi.h"

/*
  Timbangan 4 load cell + 4 HX711 untuk ESP32

  Wiring:
  LC1 HX711 DT / DOUT -> GPIO 18
  LC2 HX711 DT / DOUT -> GPIO 19
  LC3 HX711 DT / DOUT -> GPIO 5
  LC4 HX711 DT / DOUT -> GPIO 4
  Semua HX711 SCK / CLK -> GPIO 23

  Catatan penting:
  Karena semua HX711 memakai SCK yang sama, data harus dibaca paralel.
  Jika memakai 4 object HX711 biasa dengan SCK yang sama, clock dari satu
  sensor akan ikut menggeser data sensor lain.
*/

const byte NUM_LOADCELLS = 4;
const byte DOUT_PINS[NUM_LOADCELLS] = {5, 4, 19, 18};
const byte SCK_PIN = 23;

// Ganti nilai ini setelah kalibrasi.
// Rumus: calibration_factor = raw_setelah_tare / berat_diketahui_dalam_gram
// Jika hasil berat negatif, balik tanda faktor kalibrasinya.
float calibrationFactor[NUM_LOADCELLS] = {
  1200.9980426022,  // LC1
  1200.9261370178,  // LC2
  1200.9648819804,  // LC3
  1200.9690846287   // LC4
};

long tareOffset[NUM_LOADCELLS] = {0, 0, 0, 0};

const byte RAW_SAMPLES = 5;       // Rata-rata raw per pembacaan
const byte TARE_SAMPLES = 30;      // Sampel saat tare
const byte MEDIAN_SIZE = 5;        // Median filter untuk menahan spike
const float EMA_ALPHA = 0.4;      // Makin kecil makin halus, tapi makin lambat
const float ZERO_DEADBAND_GRAM = 2.0;

// Ambang debug/anomali. Nilai ini perlu disesuaikan dari hasil kalibrasi nyata.
// Jika terlalu kecil, beban normal akan dianggap error. Jika terlalu besar,
// sensor bermasalah baru terdeteksi setelah loncat sangat jauh.
const long THRESHOLD_GILA_RAW = 800000;
const long HX711_SATURATION_RAW = 8300000;
const long IMBALANCE_MIN_TOTAL_RAW = 20000;
const float IMBALANCE_RATIO = 0.75;

// Kompensasi drift dan percepatan balik ke nol.
// Auto zero hanya aktif saat pembacaan mentah sudah dekat nol dan stabil.
const bool AUTO_ZERO_TRACKING = true;
const float AUTO_ZERO_WINDOW_GRAM = 8.0;
const float AUTO_ZERO_STABLE_DELTA_GRAM = 2.0;
const float AUTO_ZERO_ALPHA = 0.02;
const byte AUTO_ZERO_CONFIRM_READS = 8;

// Jika beban diambil, raw biasanya sudah dekat 0 tetapi EMA/median masih
// menyimpan nilai berat sebelumnya. Konfirmasi beberapa pembacaan lalu reset filter.
const float RELEASE_TO_ZERO_WINDOW_GRAM = 12.0;
const float RELEASE_FILTER_MEMORY_MIN_GRAM = 20.0;
const byte RELEASE_TO_ZERO_CONFIRM_READS = 3;

// Stable load hold: untuk menahan drift saat beban diam.
// Cocok untuk menimbang benda yang diletakkan lalu dibaca, bukan untuk proses
// pengisian pelan-pelan gram demi gram.
const bool STABLE_LOAD_HOLD = true;
const float HOLD_MIN_LOAD_GRAM = 25.0;
const float HOLD_STABLE_DELTA_GRAM = 1.5;
const byte HOLD_CONFIRM_READS = 8;
const float HOLD_DRIFT_WINDOW_GRAM = 8.0;
const float HOLD_UNLOCK_DELTA_GRAM = 12.0;

float medianBuffer[MEDIAN_SIZE];
byte medianIndex = 0;
bool medianFilled = false;

float filteredWeightGram = 0.0;
bool filterStarted = false;
float lastUnfilteredWeightGram = 0.0;
byte autoZeroCounter = 0;
byte releaseToZeroCounter = 0;
float lastHoldInputGram = 0.0;
float lockedWeightGram = 0.0;
byte holdStableCounter = 0;
bool holdLocked = false;

bool isAllReady() {
  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    if (digitalRead(DOUT_PINS[i]) == HIGH) {
      return false;
    }
  }

  return true;
}

bool waitUntilReady(unsigned long timeoutMs) {
  unsigned long startTime = millis();

  while (!isAllReady()) {
    if (millis() - startTime >= timeoutMs) {
      return false;
    }
    delay(1);
  }

  return true;
}

void printNotReadySensors() {
  Serial.print("Sensor belum ready: ");

  bool firstSensor = true;
  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    if (digitalRead(DOUT_PINS[i]) == HIGH) {
      if (!firstSensor) {
        Serial.print(", ");
      }
      Serial.print("LC");
      Serial.print(i + 1);
      Serial.print("(DT GPIO ");
      Serial.print(DOUT_PINS[i]);
      Serial.print(")");
      firstSensor = false;
    }
  }

  if (firstSensor) {
    Serial.print("tidak ada, cek SCK atau timing pembacaan");
  }

  Serial.println();
}

bool readRawParallel(long rawValues[NUM_LOADCELLS]) {
  unsigned long values[NUM_LOADCELLS] = {0, 0, 0, 0};

  if (!waitUntilReady(1000)) {
    return false;
  }

  noInterrupts();

  for (byte bitIndex = 0; bitIndex < 24; bitIndex++) {
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(1);

    for (byte i = 0; i < NUM_LOADCELLS; i++) {
      values[i] <<= 1;
      if (digitalRead(DOUT_PINS[i]) == HIGH) {
        values[i]++;
      }
    }

    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(1);
  }

  // 1 pulsa tambahan = gain 128 channel A, sama seperti default library HX711.
  digitalWrite(SCK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(SCK_PIN, LOW);
  delayMicroseconds(1);

  interrupts();

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    if (values[i] & 0x800000) {
      values[i] |= 0xFF000000;
    }
    rawValues[i] = (long)values[i];
  }

  return true;
}

bool readAverageRaw(long averagedRaw[NUM_LOADCELLS], byte samples) {
  long raw[NUM_LOADCELLS];
  double sum[NUM_LOADCELLS] = {0, 0, 0, 0};

  for (byte sample = 0; sample < samples; sample++) {
    if (!readRawParallel(raw)) {
      return false;
    }

    for (byte i = 0; i < NUM_LOADCELLS; i++) {
      sum[i] += raw[i];
    }
  }

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    averagedRaw[i] = (long)(sum[i] / samples);
  }

  return true;
}

void tareScale() {
  long raw[NUM_LOADCELLS];

  Serial.println("Tare dimulai. Pastikan loyang kosong...");

  if (!readAverageRaw(raw, TARE_SAMPLES)) {
    Serial.println("Tare gagal: HX711 tidak ready. Cek wiring.");
    return;
  }

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    tareOffset[i] = raw[i];
  }

  medianIndex = 0;
  medianFilled = false;
  filterStarted = false;
  filteredWeightGram = 0.0;
  lastUnfilteredWeightGram = 0.0;
  autoZeroCounter = 0;
  releaseToZeroCounter = 0;
  lastHoldInputGram = 0.0;
  lockedWeightGram = 0.0;
  holdStableCounter = 0;
  holdLocked = false;

  Serial.println("Tare selesai.");
}

void resetWeightFilterToZero() {
  for (byte i = 0; i < MEDIAN_SIZE; i++) {
    medianBuffer[i] = 0.0;
  }

  medianIndex = 0;
  medianFilled = false;
  filteredWeightGram = 0.0;
  filterStarted = true;
  lastHoldInputGram = 0.0;
  lockedWeightGram = 0.0;
  holdStableCounter = 0;
  holdLocked = false;
}

float getMedian(float newValue) {
  medianBuffer[medianIndex] = newValue;
  medianIndex++;

  if (medianIndex >= MEDIAN_SIZE) {
    medianIndex = 0;
    medianFilled = true;
  }

  byte count = medianFilled ? MEDIAN_SIZE : medianIndex;
  float sorted[MEDIAN_SIZE];

  for (byte i = 0; i < count; i++) {
    sorted[i] = medianBuffer[i];
  }

  for (byte i = 0; i < count - 1; i++) {
    for (byte j = i + 1; j < count; j++) {
      if (sorted[j] < sorted[i]) {
        float temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }

  return sorted[count / 2];
}

float updateFilter(float weightGram) {
  float medianWeight = getMedian(weightGram);

  if (!filterStarted) {
    filteredWeightGram = medianWeight;
    filterStarted = true;
  } else {
    filteredWeightGram = (EMA_ALPHA * medianWeight) + ((1.0 - EMA_ALPHA) * filteredWeightGram);
  }

  if (fabs(filteredWeightGram) < ZERO_DEADBAND_GRAM) {
    filteredWeightGram = 0.0;
  }

  return filteredWeightGram;
}

void printRawCalibrationHelp(const long raw[NUM_LOADCELLS]) {
  Serial.print("Raw setelah tare: ");

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    long netRaw = raw[i] - tareOffset[i];

    Serial.print("LC");
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(netRaw);

    if (i < NUM_LOADCELLS - 1) {
      Serial.print(", ");
    }
  }

  Serial.println();
}

bool isCrazyRaw(long netRaw) {
  return labs(netRaw) > THRESHOLD_GILA_RAW;
}

bool isSaturatedRaw(long rawValue) {
  return labs(rawValue) > HX711_SATURATION_RAW;
}

bool isImbalancedSensor(long netRaw, long totalAbsRaw) {
  if (totalAbsRaw < IMBALANCE_MIN_TOTAL_RAW) {
    return false;
  }

  return ((float)labs(netRaw) / (float)totalAbsRaw) > IMBALANCE_RATIO;
}

bool hasSensorAnomaly(const long raw[NUM_LOADCELLS]) {
  long totalAbsRaw = 0;

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    totalAbsRaw += labs(raw[i] - tareOffset[i]);
  }

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    long netRaw = raw[i] - tareOffset[i];

    if (isCrazyRaw(netRaw) || isSaturatedRaw(raw[i])) {
      return true;
    }
  }

  return false;
}

float calculateTotalWeightGram(const long raw[NUM_LOADCELLS]) {
  float totalWeightGram = 0.0;

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    float cellWeightGram = (raw[i] - tareOffset[i]) / calibrationFactor[i];
    totalWeightGram += cellWeightGram;
  }

  return totalWeightGram;
}

bool updateAutoZeroTracking(const long raw[NUM_LOADCELLS], float totalWeightGram) {
  if (!AUTO_ZERO_TRACKING) {
    return false;
  }

  float movementGram = fabs(totalWeightGram - lastUnfilteredWeightGram);
  bool nearZero = fabs(totalWeightGram) <= AUTO_ZERO_WINDOW_GRAM;
  bool stable = movementGram <= AUTO_ZERO_STABLE_DELTA_GRAM;

  lastUnfilteredWeightGram = totalWeightGram;

  if (!nearZero || !stable) {
    autoZeroCounter = 0;
    return false;
  }

  if (autoZeroCounter < AUTO_ZERO_CONFIRM_READS) {
    autoZeroCounter++;
    return false;
  }

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    float offsetCorrection = (raw[i] - tareOffset[i]) * AUTO_ZERO_ALPHA;
    tareOffset[i] += lround(offsetCorrection);
  }

  return true;
}

bool shouldReleaseToZero(float totalWeightGram) {
  bool rawIsBackToZero = fabs(totalWeightGram) <= RELEASE_TO_ZERO_WINDOW_GRAM;
  bool filterStillHasOldLoad = fabs(filteredWeightGram) >= RELEASE_FILTER_MEMORY_MIN_GRAM;

  if (!rawIsBackToZero || !filterStillHasOldLoad) {
    releaseToZeroCounter = 0;
    return false;
  }

  if (releaseToZeroCounter < RELEASE_TO_ZERO_CONFIRM_READS) {
    releaseToZeroCounter++;
    return false;
  }

  return true;
}

float applyStableLoadHold(float measuredWeightGram) {
  if (!STABLE_LOAD_HOLD) {
    return measuredWeightGram;
  }

  if (fabs(measuredWeightGram) < HOLD_MIN_LOAD_GRAM) {
    holdStableCounter = 0;
    holdLocked = false;
    lockedWeightGram = 0.0;
    lastHoldInputGram = measuredWeightGram;
    return measuredWeightGram;
  }

  if (holdLocked) {
    float driftFromLocked = measuredWeightGram - lockedWeightGram;

    if (fabs(driftFromLocked) <= HOLD_DRIFT_WINDOW_GRAM) {
      lastHoldInputGram = measuredWeightGram;
      return lockedWeightGram;
    }

    if (fabs(driftFromLocked) >= HOLD_UNLOCK_DELTA_GRAM) {
      holdLocked = false;
      holdStableCounter = 0;
    }
  }

  float stepChange = fabs(measuredWeightGram - lastHoldInputGram);
  lastHoldInputGram = measuredWeightGram;

  if (stepChange <= HOLD_STABLE_DELTA_GRAM) {
    if (holdStableCounter < HOLD_CONFIRM_READS) {
      holdStableCounter++;
    }
  } else {
    holdStableCounter = 0;
  }

  if (holdStableCounter >= HOLD_CONFIRM_READS) {
    lockedWeightGram = measuredWeightGram;
    holdLocked = true;
    return lockedWeightGram;
  }

  return measuredWeightGram;
}

void printSensorDiagnostics(const long raw[NUM_LOADCELLS]) {
  long totalAbsRaw = 0;

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    totalAbsRaw += labs(raw[i] - tareOffset[i]);
  }

  Serial.print("DEBUG SENSOR: ");

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    long netRaw = raw[i] - tareOffset[i];
    float cellWeightGram = netRaw / calibrationFactor[i];

    Serial.print("LC");
    Serial.print(i + 1);
    Serial.print("{raw=");
    Serial.print(raw[i]);
    Serial.print(", net=");
    Serial.print(netRaw);
    Serial.print(", g=");
    Serial.print(cellWeightGram, 1);
    Serial.print(", status=");

    if (isSaturatedRaw(raw[i])) {
      Serial.print("SATURATED");
    } else if (isCrazyRaw(netRaw)) {
      Serial.print("THRESHOLD_GILA");
    } else if (isImbalancedSensor(netRaw, totalAbsRaw)) {
      Serial.print("TIDAK_SEIMBANG");
    } else {
      Serial.print("OK");
    }

    Serial.print("}");

    if (i < NUM_LOADCELLS - 1) {
      Serial.print(" | ");
    }
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(SCK_PIN, OUTPUT);
  digitalWrite(SCK_PIN, LOW);

  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    pinMode(DOUT_PINS[i], INPUT);
  }

  Serial.println();
  Serial.println("Timbangan 4 Load Cell - ESP32 + HX711");
  Serial.println("Ketik 't' di Serial Monitor untuk tare ulang.");
  Serial.println("Ketik 'r' untuk melihat raw setelah tare untuk kalibrasi.");
  Serial.println("Ketik 'd' untuk debug detail masing-masing sensor.");
  Serial.println("Auto-zero aktif saat pembacaan dekat 0 g dan stabil.");
  Serial.println("Stable-hold aktif untuk menahan drift saat beban diam.");

  tareScale();
}

void loop() {
  bool printRaw = false;
  bool printDebug = false;

  if (Serial.available()) {
    char command = Serial.read();

    if (command == 't' || command == 'T') {
      tareScale();
    } else if (command == 'r' || command == 'R') {
      printRaw = true;
    } else if (command == 'd' || command == 'D') {
      printDebug = true;
    }
  }

  long raw[NUM_LOADCELLS];

  if (!readAverageRaw(raw, RAW_SAMPLES)) {
    Serial.println("HX711 tidak ready. Cek koneksi DT/SCK.");
    printNotReadySensors();
    delay(500);
    return;
  }

  bool sensorAnomaly = hasSensorAnomaly(raw);

  if (sensorAnomaly) {
    Serial.print("PERINGATAN: anomali sensor terdeteksi. Berat ditahan di nilai terakhir: ");
    Serial.print(filteredWeightGram, 1);
    Serial.println(" g");
    printSensorDiagnostics(raw);
    delay(500);
    return;
  }

  float totalWeightGram = calculateTotalWeightGram(raw);
  bool zeroAdjusted = updateAutoZeroTracking(raw, totalWeightGram);
  totalWeightGram = calculateTotalWeightGram(raw);

  if (shouldReleaseToZero(totalWeightGram)) {
    resetWeightFilterToZero();
  }

  float heldWeightGram = applyStableLoadHold(totalWeightGram);
  float stableWeightGram = updateFilter(heldWeightGram);

  Serial.print("Berat: ");
  Serial.print(stableWeightGram, 1);
  Serial.print(" g");

  Serial.print(" | Status kopi: ");
  Serial.print(tentukanStatusKekeringanKopi(stableWeightGram));

  Serial.print(" | Susut: ");
  Serial.print(hitungPersentaseSusutKopi(stableWeightGram), 1);
  Serial.print("%");

  Serial.print(" | raw total: ");
  long totalRaw = 0;
  for (byte i = 0; i < NUM_LOADCELLS; i++) {
    totalRaw += raw[i] - tareOffset[i];
  }
  Serial.print(totalRaw);

  if (zeroAdjusted) {
    Serial.print(" | auto-zero");
  }

  if (holdLocked) {
    Serial.print(" | hold");
  }

  if (printRaw) {
    Serial.print(" | ");
    printRawCalibrationHelp(raw);
  } else if (printDebug) {
    Serial.println();
    printSensorDiagnostics(raw);
  } else {
    Serial.println();
  }

  delay(200);
}
