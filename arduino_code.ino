#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <DHT.h>
#include <Adafruit_ADXL345_U.h>
#include <ArduinoJson.h>

// Motor kontrol pinleri
const int xPulPin = 6;
const int xDirPin = 7;
const int xEnaPin = 8;
const int yPulPin = 9;
const int yDirPin = 10;
const int yEnaPin = 11;

// Sensör pinleri
const int trigPin = 12;
const int echoPin = 13;
#define DHTPIN 2
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
LSM9DS1 imu;
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

// Motor kontrol değişkenleri
volatile int moveXSpeed = 0;
volatile int moveYSpeed = 0;

// Kalibrasyon değişkenleri (sadece gyroscope için offset)
float gyro_offset_x = 0;
float gyro_offset_y = 0;
float gyro_offset_z = 0;
bool calibrated = false;

void setup() {
  Serial.begin(9600);
  
  // Başlangıç mesajı
  Serial.println("Arduino başlatılıyor... (LED TEST MODU)");
  
  // Motor pinleri
  pinMode(xPulPin, OUTPUT);
  pinMode(xDirPin, OUTPUT);
  pinMode(xEnaPin, OUTPUT);
  pinMode(yPulPin, OUTPUT);
  pinMode(yDirPin, OUTPUT);
  pinMode(yEnaPin, OUTPUT);
  
  // Sensör pinleri
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Sensörleri başlat
  dht.begin();
  Wire.begin();
  
  if (imu.begin() == false) {
    Serial.println("LSM9DS1 başlatılamadı.");
  } else {
    Serial.println("LSM9DS1 başarıyla başlatıldı!");
    // Gyroscope ve accelerometer ayarları
    imu.settings.gyro.scale = 2000; // 2000 dps
    imu.settings.accel.scale = 16;  // 16g
    imu.settings.mag.scale = 4;     // 4 Gauss
  }
  
  if(!adxl.begin()) {
    Serial.println("ADXL345 başlatılamadı!");
  } else {
    Serial.println("ADXL345 başarıyla başlatıldı!");
  }
  
  digitalWrite(xEnaPin, HIGH);
  digitalWrite(yEnaPin, HIGH);
  
  // Gyroscope kalibrasyonu (sadece offset için)
  delay(2000);
  calibrateGyroscope();
}

void loop() {
  // Motor komutlarını oku
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // Kalibrasyon komutu
    if (cmd == "CALIBRATE") {
      calibrateGyroscope();
      return;
    }
    
    // Motor komutları
    if (cmd == "X+") { moveXSpeed = 255; }
    else if (cmd == "X-") { moveXSpeed = -255; }
    else if (cmd == "Y+") { moveYSpeed = 255; }
    else if (cmd == "Y-") { moveYSpeed = -255; }
    else if (cmd == "STOPX") { moveXSpeed = 0; }
    else if (cmd == "STOPY") { moveYSpeed = 0; }
    else if (cmd == "STOP") { 
      moveXSpeed = 0; 
      moveYSpeed = 0; 
    }
  }
  
  // LED TEST MODU - Motor kontrol testleri
  if (moveXSpeed > 0) {
    digitalWrite(xDirPin, HIGH);       // Yön LED'i
    digitalWrite(xPulPin, HIGH);       // X+ LED'i yak
  } else if (moveXSpeed < 0) {
    digitalWrite(xDirPin, LOW);        // Yön LED'i 
    digitalWrite(xPulPin, HIGH);       // X- LED'i yak
  } else {
    digitalWrite(xPulPin, LOW);        // X LED'i söndür
  }
  
  if (moveYSpeed > 0) {
    digitalWrite(yDirPin, HIGH);       // Yön LED'i
    digitalWrite(yPulPin, HIGH);       // Y+ LED'i yak
  } else if (moveYSpeed < 0) {
    digitalWrite(yDirPin, LOW);        // Yön LED'i
    digitalWrite(yPulPin, HIGH);       // Y- LED'i yak
  } else {
    digitalWrite(yPulPin, LOW);        // Y LED'i söndür
  }
  
  // Sensör verilerini oku ve gönder (ham veri)
  readSensors();
  
  delay(50); // 20 Hz veri gönderimi - daha hızlı
}

void calibrateGyroscope() {
  Serial.println("Gyroscope kalibrasyonu başlıyor...");
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int samples = 100;
  
  for(int i = 0; i < samples; i++) {
    // Sadece gyroscope kalibrasyon
    if (imu.gyroAvailable()) {
      imu.readGyro();
      sum_gx += imu.calcGyro(imu.gx);
      sum_gy += imu.calcGyro(imu.gy);
      sum_gz += imu.calcGyro(imu.gz);
    }
    delay(10);
  }
  
  // Offset değerlerini hesapla
  gyro_offset_x = sum_gx / samples;
  gyro_offset_y = sum_gy / samples;
  gyro_offset_z = sum_gz / samples;
  
  calibrated = true;
  Serial.println("Gyroscope kalibrasyonu tamamlandı!");
  Serial.print("Gyro offsets: X=");
  Serial.print(gyro_offset_x);
  Serial.print(", Y=");
  Serial.print(gyro_offset_y);
  Serial.print(", Z=");
  Serial.println(gyro_offset_z);
}

void readSensors() {
  // HC-SR04
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0343) / 2;
  
  // DHT22
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  // ADXL345 - Platform için
  sensors_event_t event; 
  adxl.getEvent(&event);
  
  float adxlX = event.acceleration.x;
  float adxlY = event.acceleration.y;
  float adxlZ = event.acceleration.z;
  
  // ADXL345 gürültü filtresi
  if (abs(adxlX) < 0.05) adxlX = 0;
  if (abs(adxlY) < 0.05) adxlY = 0;
  if (abs(adxlZ - 9.8) < 0.5) adxlZ = 9.8;
  
  // JSON çıkışı - Ham veriler
  StaticJsonDocument<1536> doc;
  
  // Distance - her zaman ekle
  doc["distance_cm"] = distance;

  // DHT22 verileri - NaN kontrolü ile
  if (!isnan(h)) {
    doc["humidity"] = h;
  }
  
  if (!isnan(t)) {
    doc["temperature_c"] = t;
  }
  
  // HAM SENSÖR VERİLERİ - PYTHON MADGWİCK İÇİN
  JsonObject raw_sensors = doc.createNestedObject("raw_sensors");
  
  // Gyroscope ham verileri (kalibrasyon uygulanmış)
  if (imu.gyroAvailable()) {
    imu.readGyro();
    JsonObject gyro = raw_sensors.createNestedObject("gyro");
    gyro["x"] = imu.calcGyro(imu.gx) - gyro_offset_x;  // derece/saniye
    gyro["y"] = imu.calcGyro(imu.gy) - gyro_offset_y;
    gyro["z"] = imu.calcGyro(imu.gz) - gyro_offset_z;
  }
  
  // Accelerometer ham verileri
  if (imu.accelAvailable()) {
    imu.readAccel();
    JsonObject accel = raw_sensors.createNestedObject("accel");
    accel["x"] = imu.calcAccel(imu.ax);  // g cinsinden
    accel["y"] = imu.calcAccel(imu.ay);
    accel["z"] = imu.calcAccel(imu.az);
  }
  
  // Magnetometer ham verileri
  if (imu.magAvailable()) {
    imu.readMag();
    JsonObject mag = raw_sensors.createNestedObject("mag");
    mag["x"] = imu.calcMag(imu.mx);     // Gauss cinsinden
    mag["y"] = imu.calcMag(imu.my);
    mag["z"] = imu.calcMag(imu.mz);
  }
  
  // PLATFORM için ADXL345 verisi
  JsonObject adxl345 = doc.createNestedObject("adxl345");
  adxl345["x"] = adxlX;
  adxl345["y"] = adxlY;
  adxl345["z"] = adxlZ;
  
  serializeJson(doc, Serial);
  Serial.println();
  Serial.flush();
  
  // Her 50 okumada bir durum mesajı
  static int readCount = 0;
  readCount++;
  if (readCount >= 50) {
    Serial.println("STATUS: Arduino - Ham veri modu aktif");
    Serial.flush();
    readCount = 0;
  }
}