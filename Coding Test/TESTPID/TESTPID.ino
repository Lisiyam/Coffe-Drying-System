#include <DHT.h>

#define DHTPIN 26
#define DHTTYPE DHT22

#define HEATER_PIN 27

DHT dht(DHTPIN, DHTTYPE);

// ================= PID =================
float setpoint = 38.0;

// TUNING PID
float Kp = 35.0;
float Ki = 0.50;
float Kd = 0.0;

float error;
float previousError = 0;

float integral = 0;
float derivative;

float output;

// ================= PWM =================
const int pwmFreq = 1000;
const int pwmResolution = 8;

// ================= TIME =================
unsigned long previousTime = 0;
unsigned long startTime = 0;

void setup() {

  Serial.begin(115200);

  dht.begin();

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);

  // PWM setup ESP32
  ledcAttach(HEATER_PIN, pwmFreq, pwmResolution);

  previousTime = millis();
  startTime = millis();

  // Header CSV
  Serial.println("Waktu,Suhu,Kelembapan,Error,Duty");
}

void loop() {

  // ================= BACA SENSOR =================
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Cek gagal baca
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("ERROR SENSOR");
    delay(1000);
    return;
  }

  // ================= HITUNG dt =================
  unsigned long currentTime = millis();

  float dt = (currentTime - previousTime) / 1000.0;

  if (dt <= 0) dt = 0.001;

  previousTime = currentTime;

  // ================= PID =================
  error = setpoint - temperature;

  // Integral
  integral += error * dt;

  // Anti-windup
  integral = constrain(integral, -100, 100);

  // Derivative
  derivative = (error - previousError) / dt;

  // PID Output
  output =
      (Kp * error) +
      (Ki * integral) +
      (Kd * derivative);

  previousError = error;

  // ================= BATASI OUTPUT =================
  output = constrain(output, 0, 255);

  // ================= PWM OUTPUT =================
  ledcWrite(HEATER_PIN, (int)output);

  // ================= DUTY PERCENT =================
  float dutyPercent = (output / 255.0) * 100.0;

  // ================= WAKTU =================
  unsigned long elapsedSeconds =
      (millis() - startTime) / 1000;

  // ================= SERIAL CSV =================
  Serial.print(elapsedSeconds);
  Serial.print(",");

  Serial.print(temperature, 2);
  Serial.print(",");

  Serial.print(humidity, 2);
  Serial.print(",");

  Serial.print(error, 2);
  Serial.print(",");

  Serial.println(dutyPercent, 2);

  delay(1000);
}