#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <DHT.h>
#include <Adafruit_ADXL345_U.h>
#include <ArduinoJson.h>

// Sensör pinleri
const int trigPin = 12;
const int echoPin = 13;
#define DHTPIN 2
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
LSM9DS1 imu;
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

// Kalibrasyon değişkenleri (sadece gyroscope için offset)
float gyro_offset_x = 0;
float gyro_offset_y = 0;
float gyro_offset_z = 0;
bool calibrated = false;

// Serial komut buffer - Basit char array (String yerine)
char inputBuffer[20];
byte inputIndex = 0;
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  dht.begin();
  Wire.begin();
  
  // LSM9DS1 manuel adres ayarı
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = 0x1E; // Magnetometer
  imu.settings.device.agAddress = 0x6B; // Gyro/Accel
  
  if (imu.begin() == false) {
    scanI2CDevices();
  } else {
    imu.settings.gyro.scale = 2000;
    imu.settings.accel.scale = 16;
    imu.settings.mag.scale = 4;
  }
  
  if(!adxl.begin()) {
    scanI2CDevices();
  } else {
    adxl.setRange(ADXL345_RANGE_16_G);
  }

  delay(500);
  calibrateGyroscope();
}

// I2C scan kaldırıldı - RAM tasarrufu
void scanI2CDevices() {}

void loop() {
  if (stringComplete) {
    inputBuffer[inputIndex] = '\0';
    processSerialCommand(inputBuffer);
    inputIndex = 0;
    stringComplete = false;
  }
  
  readSensors();
  delay(50);
}

void processSerialCommand(char* command) {
  if (strcmp(command, "CALIBRATE") == 0) {
    calibrateGyroscope();
  }
}

void calibrateGyroscope() {
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int samples = 50;
  
  for(int i = 0; i < samples; i++) {
    if (imu.gyroAvailable()) {
      imu.readGyro();
      sum_gx += imu.calcGyro(imu.gx);
      sum_gy += imu.calcGyro(imu.gy);
      sum_gz += imu.calcGyro(imu.gz);
    }
    delay(10);
  }
  
  gyro_offset_x = sum_gx / samples;
  gyro_offset_y = sum_gy / samples;
  gyro_offset_z = sum_gz / samples;
  calibrated = true;
}

// Test fonksiyonu kaldırıldı - RAM tasarrufu

void readSensors() {
  // JSON buffer - 512 byte yeterli
  StaticJsonDocument<512> doc;
  
  // HC-SR04 - Timeout korumalı
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  float distance = 0;
  if (duration > 0) {
    distance = (duration * 0.0343) / 2;
    if (distance > 400) distance = 400; // Max mesafe sınırı
  }
  doc["distance_cm"] = distance;
  
  // DHT22 - Hata kontrolü ile
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  if (!isnan(h) && h >= 0 && h <= 100) {
    doc["humidity"] = h;
  } else {
    doc["humidity"] = nullptr;
  }
  
  if (!isnan(t) && t >= -40 && t <= 80) {
    doc["temperature_c"] = t;
  } else {
    doc["temperature_c"] = nullptr;
  }
  
  // HAM SENSÖR VERİLERİ - PYTHON MADGWİCK İÇİN
  JsonObject raw_sensors = doc.createNestedObject("raw_sensors");
  bool imu_success = false;
  
  // Gyroscope ham verileri (kalibrasyon uygulanmış)
  if (imu.gyroAvailable()) {
    imu.readGyro();
    JsonObject gyro = raw_sensors.createNestedObject("gyro");
    gyro["x"] = imu.calcGyro(imu.gx) - gyro_offset_x;  // derece/saniye
    gyro["y"] = imu.calcGyro(imu.gy) - gyro_offset_y;
    gyro["z"] = imu.calcGyro(imu.gz) - gyro_offset_z;
    imu_success = true;
  }
  
  // Accelerometer ham verileri
  if (imu.accelAvailable()) {
    imu.readAccel();
    JsonObject accel = raw_sensors.createNestedObject("accel");
    accel["x"] = imu.calcAccel(imu.ax);  // g cinsinden
    accel["y"] = imu.calcAccel(imu.ay);
    accel["z"] = imu.calcAccel(imu.az);
    imu_success = true;
  }
  
  // Magnetometer ham verileri
  if (imu.magAvailable()) {
    imu.readMag();
    JsonObject mag = raw_sensors.createNestedObject("mag");
    mag["x"] = imu.calcMag(imu.mx);     // Gauss cinsinden
    mag["y"] = imu.calcMag(imu.my);
    mag["z"] = imu.calcMag(imu.mz);
  }
  
  // PLATFORM için ADXL345 verisi (m/s² cinsinden)
  sensors_event_t event; 
  adxl.getEvent(&event);
  
  float adxlX = event.acceleration.x;
  float adxlY = event.acceleration.y;
  float adxlZ = event.acceleration.z;
  
  // ADXL345 geçerlilik kontrolü (m/s² cinsinden)
  if (abs(adxlX) < 50 && abs(adxlY) < 50 && abs(adxlZ) < 50) {
    // Gürültü filtresi (0.5 m/s² altı sıfırla)
    if (abs(adxlX) < 0.5) adxlX = 0;
    if (abs(adxlY) < 0.5) adxlY = 0;
    
    JsonObject adxl345 = doc.createNestedObject("adxl345");
    adxl345["x"] = adxlX;
    adxl345["y"] = adxlY;
    adxl345["z"] = adxlZ;
  } else {
    // Geçersiz ADXL345 verisi
    JsonObject adxl345 = doc.createNestedObject("adxl345");
    adxl345["x"] = 0;
    adxl345["y"] = 0;
    adxl345["z"] = 9.8;
  }
  
  // JSON çıktısı - SADECE JSON GÖNDER
  serializeJson(doc, Serial);
  Serial.println();
  Serial.flush();
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else if (inputIndex < 19) {
      inputBuffer[inputIndex++] = inChar;
    }
  }
}