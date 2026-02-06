#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include "LittleFS.h"
#include <MadgwickAHRS.h>
#include <ESP32Servo.h>  // ✅ ESP32 servo library

// ================= WIFI =================
const char* ssid = "Sanmugavel";
const char* password = "7667189228";

// ================= MPU =================
#define MPU_ADDR 0x68
bool stabilizeMode = true;  // true = STABILIZE, false = FREE

AsyncWebServer server(80);
AsyncEventSource events("/events");
JSONVar readings;

// ================= TIMING =================
unsigned long lastTime = 0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long lastIMUTime = 0;

unsigned long gyroDelay = 10;
unsigned long accelerometerDelay = 200;
unsigned long temperatureDelay = 1000;

// ================= FILTER =================
Madgwick filter;

// raw orientation
float roll, pitch, yaw;

// smoothed orientation
float smoothRoll = 0;
float smoothPitch = 0;
float smoothYaw = 0;

// accel low-pass
float ax_f=0, ay_f=0, az_f=0;

// gyro bias (drift removal)
float gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;

float temperature;

// ================= SERVO =================
Servo actuator;
int servoAngle = 90;   // neutral position

// ================= MPU FUNCTIONS =================
void initMPU() {
  Wire.begin(21, 22);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08); // ±500 dps
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x08); // ±4g
  Wire.endTransmission();

  Serial.println("MPU initialized");
}

// ================= CALIBRATION =================
void calibrateGyro() {
  Serial.println("Calibrating gyro...");
  float sx=0, sy=0, sz=0;

  for(int i=0;i<500;i++){
    float ax,ay,az,gx,gy,gz,t;
    readMPU(ax,ay,az,gx,gy,gz,t);
    sx+=gx; sy+=gy; sz+=gz;
    delay(5);
  }

  gyroBiasX = sx/500.0;
  gyroBiasY = sy/500.0;
  gyroBiasZ = sz/500.0;

  Serial.println("Calibration done");
}

void readMPU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz,
             float &tempC) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / 8192.0;
  ay = rawAy / 8192.0;
  az = rawAz / 8192.0;

  gx = rawGx / 65.5 - gyroBiasX;
  gy = rawGy / 65.5 - gyroBiasY;
  gz = rawGz / 65.5 - gyroBiasZ;

  tempC = (rawTemp / 340.0) + 36.53;
}

// ================= FILESYSTEM =================
void initLittleFS() { LittleFS.begin(); }

// ================= WIFI =================
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(1000);
  Serial.println(WiFi.localIP());
}

// ================= SENSOR JSON =================
String getGyroReadings(){
  float ax, ay, az, gx, gy, gz, t;
  readMPU(ax, ay, az, gx, gy, gz, t);

  // accel low-pass filter
  ax_f = 0.8*ax_f + 0.2*ax;
  ay_f = 0.8*ay_f + 0.2*ay;
  az_f = 0.8*az_f + 0.2*az;

  // real dt
  unsigned long now = micros();
  float dt = (now - lastIMUTime)*1e-6;
  lastIMUTime = now;
  if(dt<=0 || dt>0.2) dt=0.01;

  filter.updateIMU(
    gx * DEG_TO_RAD,
    gy * DEG_TO_RAD,
    gz * DEG_TO_RAD,
    ax_f, ay_f, az_f
  );

  roll  = filter.getRoll();
  pitch = filter.getPitch();
  yaw   = filter.getYaw();

  // ultra-smooth visual filter
  smoothRoll  = 0.97*smoothRoll  + 0.03*roll;
  smoothPitch = 0.97*smoothPitch + 0.03*pitch;
  smoothYaw   = 0.995*smoothYaw  + 0.005*yaw;

  // ================= CONTROL LOGIC =================
  float targetRoll = 0;
  float error = targetRoll - smoothRoll;

  if(abs(error) < 1.0) error = 0;

  float Kp = 1.5;

  if(stabilizeMode){
   servoAngle = 90 + Kp * error;  // STABILIZE
  }
  else{
    servoAngle = 90 + smoothRoll;           // FREE
  }

  if(servoAngle > 180) servoAngle = 180;
  if(servoAngle < 0)   servoAngle = 0;

  actuator.write(servoAngle);

  // ================= SERIAL OUTPUT =================
  Serial.print(stabilizeMode ? "MODE: STABILIZE | " : "MODE: FREE | ");
  Serial.print("Roll: ");
  Serial.print(smoothRoll, 2);
  Serial.print(" deg | Error: ");
  Serial.print(error, 2);
  Serial.print(" deg | Servo: ");
  Serial.print(servoAngle);
  Serial.println(" deg");

  readings["gyroX"] = String(smoothRoll, 2);
  readings["gyroY"] = String(smoothPitch, 2);
  readings["gyroZ"] = String(smoothYaw, 2);
  readings["servo"] = servoAngle;

  return JSON.stringify(readings);
}

String getAccReadings() {
  float ax, ay, az, gx, gy, gz, t;
  readMPU(ax, ay, az, gx, gy, gz, t);

  readings["accX"] = String(ax, 2);
  readings["accY"] = String(ay, 2);
  readings["accZ"] = String(az, 2);

  return JSON.stringify(readings);
}

String getTemperature(){
  float ax, ay, az, gx, gy, gz, t;
  readMPU(ax, ay, az, gx, gy, gz, t);
  temperature = t;
  return String(temperature, 2);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  initWiFi();
  initLittleFS();
  initMPU();

  calibrateGyro();

  filter.begin(100);

  actuator.attach(25);
  actuator.write(90);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    smoothRoll = smoothPitch = smoothYaw = 0;
    filter.begin(100);
    request->send(200, "text/plain", "OK");
  });

  server.on("/free", HTTP_GET, [](AsyncWebServerRequest *request){
    stabilizeMode = false;
    request->send(200, "text/plain", "FREE MODE");
  });

  server.on("/stabilize", HTTP_GET, [](AsyncWebServerRequest *request){
    stabilizeMode = true;
    request->send(200, "text/plain", "STABILIZE MODE");
  });

  events.onConnect([](AsyncEventSourceClient *client){
    client->send("hello!", NULL, millis(), 10000);
  });

  server.addHandler(&events);
  server.begin();
}

// ================= LOOP =================
void loop() {
  if ((millis() - lastTime) > gyroDelay) {
    events.send(getGyroReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }

  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    events.send(getAccReadings().c_str(),"accelerometer_readings",millis());
    lastTimeAcc = millis();
  }

  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    events.send(getTemperature().c_str(),"temperature_reading",millis());
    lastTimeTemperature = millis();
  }
}
