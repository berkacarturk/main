// Arduino kodunda readSensors() fonksiyonunun son kısmını bu şekilde değiştirin:

void readSensors() {
  // ... önceki kod aynı kalıyor ...
  
  // JSON çıkışı - Buffer boyutu artırıldı ve güvenlik kontrolleri eklendi
  StaticJsonDocument<2048> doc;  // 1024'ten 2048'e artırıldı
  
  // Sensör değerlerini kontrol ederek ekle
  if (!isnan(distance) && distance >= 0) {
    doc["distance_cm"] = distance;
  } else {
    doc["distance_cm"] = 0;
  }
  
  if (!isnan(h)) {
    doc["humidity"] = h;
  } else {
    doc["humidity"] = nullptr;
  }
  
  if (!isnan(t)) {
    doc["temperature_c"] = t;
  } else {
    doc["temperature_c"] = nullptr;
  }
  
  // MADGWICK SONUÇLARI - SU TERAZİSİ İÇİN
  JsonObject madgwick = doc.createNestedObject("madgwick");
  madgwick["roll"] = roll;
  madgwick["pitch"] = pitch;  
  madgwick["yaw"] = yaw;
  
  // QUATERNION verisi
  JsonObject quat = doc.createNestedObject("quaternion");
  quat["w"] = q[0];
  quat["x"] = q[1];
  quat["y"] = q[2];
  quat["z"] = q[3];
  
  // HAM SENSÖR VERİLERİ - DETECTOR İÇİN
  JsonObject raw_sensors = doc.createNestedObject("raw_sensors");
  
  // Gyroscope ham verileri
  if (imu.gyroAvailable()) {
    JsonObject gyro = raw_sensors.createNestedObject("gyro");
    gyro["x"] = imu.calcGyro(imu.gx) - gyro_offset_x;
    gyro["y"] = imu.calcGyro(imu.gy) - gyro_offset_y;
    gyro["z"] = imu.calcGyro(imu.gz) - gyro_offset_z;
  }
  
  // Accelerometer ham verileri
  if (imu.accelAvailable()) {
    JsonObject accel = raw_sensors.createNestedObject("accel");
    accel["x"] = imu.calcAccel(imu.ax);
    accel["y"] = imu.calcAccel(imu.ay);
    accel["z"] = imu.calcAccel(imu.az);
  }
  
  // PLATFORM için ADXL345 verisi
  JsonObject adxl345 = doc.createNestedObject("adxl345");
  adxl345["x"] = adxlX;
  adxl345["y"] = adxlY;
  adxl345["z"] = adxlZ;
  
  // JSON'ı kontrollü olarak gönder
  String jsonString;
  serializeJson(doc, jsonString);
  
  // JSON'ın geçerli olup olmadığını kontrol et
  if (jsonString.length() > 10 && jsonString.startsWith("{") && jsonString.endsWith("}")) {
    Serial.println(jsonString);  // println kullan - \n otomatik eklenir
    Serial.flush();              // Buffer'ı boşalt - ÖNEMLİ!
  } else {
    Serial.println("ERROR: Geçersiz JSON oluşturuldu");
  }
  
  delay(100); // 10 FPS - daha stabil veri gönderimi (50ms'den 100ms'ye çıkarıldı)
}