#include "SparkFunLSM9DS1.h"
#include "DHT.h"
#include "Adafruit_ADXL345_U.h"
#include <ArduinoJson.h>

LSM9DS1 imu;
DHT dht22(22, DHT22);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#define LSM9DS1_M 0x1E
#define LSM9DS1_AG 0x6B
#define TRIG_PIN 23
#define ECHO_PIN 24

// Madgwick filter değişkenleri
float beta = 0.1f;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float sampleFreq = 20.0f; // 20 Hz

void setup() {
    Serial.begin(9600);
    
    // DHT22 başlat
    dht22.begin();
    
    // LSM9DS1 başlat
    imu.settings.device.commInterface = IMU_MODE_I2C;
    imu.settings.device.mAddress = LSM9DS1_M;
    imu.settings.device.agAddress = LSM9DS1_AG;
    
    if (!imu.begin()) {
        while (1) {
            Serial.println("{\"error\":\"LSM9DS1 başlatılamadı\"}");
            delay(1000);
        }
    }
    
    // ADXL345 başlat
    if(!accel.begin()) {
        while(1) {
            Serial.println("{\"error\":\"ADXL345 başlatılamadı\"}");
            delay(1000);
        }
    }
    
    // Ultrasonik sensor
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    delay(1000);
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

float getRoll() {
    return atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 180.0f / PI;
}

float getPitch() {
    return asinf(-2.0f * (q1*q3 - q0*q2)) * 180.0f / PI;
}

float getYaw() {
    return atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 180.0f / PI;
}

int readUltrasonicDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) return 0;
    int distance = duration * 0.034 / 2;
    return distance;
}

void loop() {
    // Basit JSON format - her sensör için ayrı mesaj
    static unsigned long lastSensorTime = 0;
    static int sensorIndex = 0;
    
    if (millis() - lastSensorTime > 100) {  // Her 100ms'de farklı sensör
        lastSensorTime = millis();
        
        switch(sensorIndex) {
            case 0: {
                // DHT22 verisi
                float h = dht22.readHumidity();
                float t = dht22.readTemperature();
                if (!isnan(h) && !isnan(t)) {
                    Serial.print("{\"sensor\":\"dht22\",\"humidity\":");
                    Serial.print(h, 1);
                    Serial.print(",\"temperature\":");
                    Serial.print(t, 1);
                    Serial.println("}");
                }
                break;
            }
            
            case 1: {
                // Ultrasonik sensör
                int distance = readUltrasonicDistance();
                Serial.print("{\"sensor\":\"ultrasonic\",\"distance\":");
                Serial.print(distance);
                Serial.println("}");
                break;
            }
            
            case 2: {
                // LSM9DS1 Gyro
                if (imu.gyroAvailable()) {
                    imu.readGyro();
                    Serial.print("{\"sensor\":\"gyro\",\"gx\":");
                    Serial.print(imu.calcGyro(imu.gx), 3);
                    Serial.print(",\"gy\":");
                    Serial.print(imu.calcGyro(imu.gy), 3);
                    Serial.print(",\"gz\":");
                    Serial.print(imu.calcGyro(imu.gz), 3);
                    Serial.println("}");
                }
                break;
            }
            
            case 3: {
                // LSM9DS1 Accel
                if (imu.accelAvailable()) {
                    imu.readAccel();
                    float ax = imu.calcAccel(imu.ax);
                    float ay = imu.calcAccel(imu.ay);
                    float az = imu.calcAccel(imu.az);
                    
                    Serial.print("{\"sensor\":\"accel\",\"ax\":");
                    Serial.print(ax, 3);
                    Serial.print(",\"ay\":");
                    Serial.print(ay, 3);
                    Serial.print(",\"az\":");
                    Serial.print(az, 3);
                    Serial.println("}");
                }
                break;
            }
            
            case 4: {
                // ADXL345
                sensors_event_t event;
                accel.getEvent(&event);
                Serial.print("{\"sensor\":\"adxl345\",\"x\":");
                Serial.print(event.acceleration.x, 2);
                Serial.print(",\"y\":");
                Serial.print(event.acceleration.y, 2);
                Serial.print(",\"z\":");
                Serial.print(event.acceleration.z, 2);
                Serial.println("}");
                break;
            }
            
            case 5: {
                // Madgwick filtreli açılar (mevcut sensör verilerini kullan)
                if (imu.gyroAvailable() && imu.accelAvailable()) {
                    imu.readGyro();
                    imu.readAccel();
                    
                    float gx = imu.calcGyro(imu.gx) * PI / 180.0;
                    float gy = imu.calcGyro(imu.gy) * PI / 180.0;  
                    float gz = imu.calcGyro(imu.gz) * PI / 180.0;
                    float ax = imu.calcAccel(imu.ax);
                    float ay = imu.calcAccel(imu.ay);
                    float az = imu.calcAccel(imu.az);
                    
                    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
                    
                    Serial.print("{\"sensor\":\"madgwick\",\"roll\":");
                    Serial.print(getRoll(), 2);
                    Serial.print(",\"pitch\":");
                    Serial.print(getPitch(), 2);
                    Serial.print(",\"yaw\":");
                    Serial.print(getYaw(), 2);
                    Serial.println("}");
                }
                break;
            }
        }
        
        sensorIndex = (sensorIndex + 1) % 6;  // 6 farklı sensör tipi
        
        // Her 50 döngüde bir durum mesajı
        static int statusCounter = 0;
        statusCounter++;
        if (statusCounter >= 300) {  // 50*6 = 300 mesaj sonrası
            Serial.println("{\"sensor\":\"status\",\"message\":\"Arduino OK\"}");
            statusCounter = 0;
        }
    }
    
    // Serial komutları dinle
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "STOP") {
            Serial.println("{\"sensor\":\"response\",\"message\":\"Motors stopped\"}");
        }
        else if (command.startsWith("MOVE")) {
            Serial.println("{\"sensor\":\"response\",\"message\":\"Movement command received\"}");
        }
    }
    
    delay(20);  // 20ms delay - yaklaşık 50 Hz
}