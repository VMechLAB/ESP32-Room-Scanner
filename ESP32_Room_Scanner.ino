/*
 * ESP32 Room Scanner
 * HC-SR04 + MG996R + MPU6050 + GPS + DHT11
 * Sends sensor data as JSON over USB.
 * Made by : VMechLAB ; Vasilije
 */

#include <Wire.h>
#include <ESP32Servo.h>
#include <TinyGPSPlus.h>
#include <DHT.h>
#include <MPU6050_light.h>

// Pins
#define I2C_SDA       21
#define I2C_SCL       22
#define DHT_PIN       4
#define SERVO_PIN     13
#define GPS_RX_PIN    16
#define TRIG_PIN      12
#define ECHO_PIN      14

// Sensors
TinyGPSPlus gps;
DHT dht(DHT_PIN, DHT11);
Servo myservo;
MPU6050 mpu(Wire);

bool imu_ok = false;

// Scan settings
int angle = 0;
int step = 2;
const int SERVO_DELAY = 30;

// Read distance from HC-SR04
uint16_t readHC_SR04() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return 0;

  return duration / 5.8;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== ROOM SCANNER START ===");

  Wire.begin(I2C_SDA, I2C_SCL);

  // HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo
  myservo.attach(SERVO_PIN);
  myservo.write(0);
  delay(100);

  // DHT11
  dht.begin();

  // MPU6050
  if (mpu.begin() != 0) {
    Serial.println("MPU6050 NOT found!");
  } else {
    Serial.println("MPU6050 found. Calibrating...");
    mpu.calcOffsets(true, true);
    imu_ok = true;
    Serial.println("IMU ready.");
  }

  // GPS
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, -1);
  Serial.println("GPS serial started.");

  Serial.println("=== SETUP DONE ===");
}

void loop() {
  // Move scanner
  myservo.write(angle);
  delay(SERVO_DELAY);

  // Distance
  uint16_t distance = readHC_SR04();

  // IMU
  float pitch = 0, roll = 0, yaw = 0;

  if (imu_ok) {
    mpu.update();
    pitch = mpu.getAngleX();
    roll  = mpu.getAngleY();
    yaw   = mpu.getAngleZ();
  }

  // GPS
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  float lat = 0, lon = 0;

  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  // Temperature
  float temperature = dht.readTemperature();

  if (isnan(temperature)) {
    temperature = 0;
  }

  // Send data
  Serial.print("{\"angle\":");
  Serial.print(angle);
  Serial.print(",\"dist\":");
  Serial.print(distance);
  Serial.print(",\"pitch\":");
  Serial.print(pitch, 2);
  Serial.print(",\"roll\":");
  Serial.print(roll, 2);
  Serial.print(",\"yaw\":");
  Serial.print(yaw, 2);
  Serial.print(",\"temp\":");
  Serial.print(temperature, 1);
  Serial.print(",\"lat\":");
  Serial.print(lat, 6);
  Serial.print(",\"lon\":");
  Serial.print(lon, 6);
  Serial.println("}");

  // Change scan direction
  angle += step;

  if (angle >= 180 || angle <= 0) {
    step = -step;
  }

  delay(20);
}
