// MOTOR CONTROL ARDUINO - Touchscreen Kontrolü
// Python touchscreen uygulaması ile entegre
// Dual Arduino sisteminde motor kontrol birimi
// Sadece touchscreen kontrolü
// MOTOR PİNLERİ LED OLARAK KULLANILIYOR - Ayrı LED yok

// X eksen pinleri - EM 806 Motor Sürücü
const int xPulPin = 6;    // Pulse pin (her zaman yanar)
const int xDirPin = 5;    // Direction pin (+ yönde yanar, - yönde kapalı)
const int xEnaPin = 8;    // Enable pin (LOW = aktif, HIGH = devre dışı)

// Y eksen pinleri
const int yPulPin = 3;    // Pulse pin (her zaman yanar)
const int yDirPin = 11;   // Direction pin (+ yönde yanar, - yönde kapalı)
const int yEnaPin = 13;   // Enable pin (LOW = aktif, HIGH = devre dışı)

// Z eksen pinleri
const int zPulPin = 9;    // Pulse pin (her zaman yanar)
const int zDirPin = 10;   // Direction pin (+ yönde yanar, - yönde kapalı)
const int zEnaPin = A2;   // Enable pin (LOW = aktif, HIGH = devre dışı)

// Motor kontrol değişkenleri
int stepDelay = 500;        // Mikrosaniye cinsinden step delay (hız kontrolü)
int motorSpeed = 128;       // PWM hız değeri (0-255)

// Sürekli hareket kontrol değişkenleri
bool xMotorRunning = false;
bool yMotorRunning = false;
bool zMotorRunning = false;
bool xDirection = true;     // true = pozitif, false = negatif
bool yDirection = true;
bool zDirection = true;

// EM 806 Motor sürücü enable durumları
bool xAxisEnabled = true;   // Eksen unlock durumu (true = unlock, false = lock)
bool yAxisEnabled = true;
bool zAxisEnabled = true;

// Serial komut buffer
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  
  // Motor pinlerini output olarak ayarla
  pinMode(xPulPin, OUTPUT);
  pinMode(xDirPin, OUTPUT);
  pinMode(xEnaPin, OUTPUT);

  pinMode(yPulPin, OUTPUT);
  pinMode(yDirPin, OUTPUT);
  pinMode(yEnaPin, OUTPUT);

  pinMode(zPulPin, OUTPUT);
  pinMode(zDirPin, OUTPUT);
  pinMode(zEnaPin, OUTPUT);

  // Başlangıçta tüm motor pinlerini kapat
  digitalWrite(xPulPin, LOW);
  digitalWrite(xDirPin, LOW);
  digitalWrite(yPulPin, LOW);
  digitalWrite(yDirPin, LOW);
  digitalWrite(zPulPin, LOW);
  digitalWrite(zDirPin, LOW);
  
  // EM 806 Enable pinleri - Başlangıçta motorları aktif et (LOW = aktif)
  digitalWrite(xEnaPin, LOW);
  digitalWrite(yEnaPin, LOW);
  digitalWrite(zEnaPin, LOW);

  // Reserve space for the input string
  inputString.reserve(200);
  
  Serial.println("MOTOR_ARDUINO_READY");
}

void loop() {
  // Serial komutları işle (Touchscreen kontrolü)
  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Sürekli hareket kontrolü
  continuousMotorControl();
}

void continuousMotorControl() {
  // EM 806 Enable pin kontrolü
  digitalWrite(xEnaPin, xAxisEnabled ? LOW : HIGH);  // LOW = aktif, HIGH = devre dışı
  digitalWrite(yEnaPin, yAxisEnabled ? LOW : HIGH);
  digitalWrite(zEnaPin, zAxisEnabled ? LOW : HIGH);
  
  // X ekseni sürekli hareket
  if (xMotorRunning && xAxisEnabled) {
    analogWrite(xPulPin, motorSpeed);  // Pulse pin her zaman yanar
    if (xDirection) {
      analogWrite(xDirPin, motorSpeed);  // Direction pin sadece + yönde yanar
    } else {
      digitalWrite(xDirPin, LOW);        // Direction pin - yönde kapalı
    }
  } else {
    digitalWrite(xPulPin, LOW);
    digitalWrite(xDirPin, LOW);
  }
  
  // Y ekseni sürekli hareket (eski Z)
  if (yMotorRunning && yAxisEnabled) {
    analogWrite(yPulPin, motorSpeed);  // Pulse pin her zaman yanar
    if (yDirection) {
      analogWrite(yDirPin, motorSpeed);  // Direction pin sadece + yönde yanar
    } else {
      digitalWrite(yDirPin, LOW);        // Direction pin - yönde kapalı
    }
  } else {
    digitalWrite(yPulPin, LOW);
    digitalWrite(yDirPin, LOW);
  }
  
  // Z ekseni sürekli hareket (eski Y)
  if (zMotorRunning && zAxisEnabled) {
    analogWrite(zPulPin, motorSpeed);  // Pulse pin her zaman yanar
    if (zDirection) {
      analogWrite(zDirPin, motorSpeed);  // Direction pin sadece + yönde yanar
    } else {
      digitalWrite(zDirPin, LOW);        // Direction pin - yönde kapalı
    }
  } else {
    digitalWrite(zPulPin, LOW);
    digitalWrite(zDirPin, LOW);
  }
  
  // Kısa delay
  delay(1);
}

void updateMotorSpeeds() {
  // Çalışan motorların hızını anında güncelle
  if (xMotorRunning) {
    analogWrite(xPulPin, motorSpeed);  // Pulse pin her zaman
    if (xDirection) {
      analogWrite(xDirPin, motorSpeed);  // Direction pin sadece + yönde
    } else {
      digitalWrite(xDirPin, LOW);        // Direction pin - yönde kapalı
    }
  }
  
  if (yMotorRunning) {
    analogWrite(yPulPin, motorSpeed);  // Pulse pin her zaman (eski Z ekseni)
    if (yDirection) {
      analogWrite(yDirPin, motorSpeed);  // Direction pin sadece + yönde
    } else {
      digitalWrite(yDirPin, LOW);        // Direction pin - yönde kapalı
    }
  }
  
  if (zMotorRunning) {
    analogWrite(zPulPin, motorSpeed);  // Pulse pin her zaman (eski Y ekseni)
    if (zDirection) {
      analogWrite(zDirPin, motorSpeed);  // Direction pin sadece + yönde
    } else {
      digitalWrite(zDirPin, LOW);        // Direction pin - yönde kapalı
    }
  }
}

void processSerialCommand(String command) {
  command.trim();
  
  // Debug: Gelen komutu yazdır
  Serial.print("COMMAND_RECEIVED: ");
  Serial.println(command);
  
  // IDENTIFY komutu - Arduino tipini belirt
  if (command == "IDENTIFY") {
    Serial.println("MOTOR_ARDUINO");
    Serial.println("TOUCHSCREEN_CONTINUOUS_CONTROL");
    return;
  }
  
  // Motor hareket komutları (Touchscreen kontrolü - Sürekli hareket)
  if (command == "X+") {
    if (xAxisEnabled) {
      xMotorRunning = true;
      xDirection = true;
      Serial.println("X_MOTOR_STARTED_POSITIVE");
    } else {
      Serial.println("X_AXIS_LOCKED");
    }
  }
  else if (command == "X-") {
    if (xAxisEnabled) {
      xMotorRunning = true;
      xDirection = false;
      Serial.println("X_MOTOR_STARTED_NEGATIVE");
    } else {
      Serial.println("X_AXIS_LOCKED");
    }
  }
  else if (command == "Y+") {
    if (yAxisEnabled) {
      yMotorRunning = true;
      yDirection = true;
      Serial.println("Y_MOTOR_STARTED_POSITIVE");
    } else {
      Serial.println("Y_AXIS_LOCKED");
    }
  }
  else if (command == "Y-") {
    if (yAxisEnabled) {
      yMotorRunning = true;
      yDirection = false;
      Serial.println("Y_MOTOR_STARTED_NEGATIVE");
    } else {
      Serial.println("Y_AXIS_LOCKED");
    }
  }
  else if (command == "Z+") {
    if (zAxisEnabled) {
      zMotorRunning = true;
      zDirection = true;
      Serial.println("Z_MOTOR_STARTED_POSITIVE");
    } else {
      Serial.println("Z_AXIS_LOCKED");
    }
  }
  else if (command == "Z-") {
    if (zAxisEnabled) {
      zMotorRunning = true;
      zDirection = false;
      Serial.println("Z_MOTOR_STARTED_NEGATIVE");
    } else {
      Serial.println("Z_AXIS_LOCKED");
    }
  }
  
  // Motor durdurma komutları
  else if (command == "STOPX") {
    xMotorRunning = false;
    digitalWrite(xPulPin, LOW);
    digitalWrite(xDirPin, LOW);
    Serial.println("X_MOTOR_STOPPED");
  }
  else if (command == "STOPY") {
    yMotorRunning = false;
    digitalWrite(yPulPin, LOW);
    digitalWrite(yDirPin, LOW);
    Serial.println("Y_MOTOR_STOPPED");
  }
  else if (command == "STOPZ") {
    zMotorRunning = false;
    digitalWrite(zPulPin, LOW);
    digitalWrite(zDirPin, LOW);
    Serial.println("Z_MOTOR_STOPPED");
  }
  
  // Hız kontrol komutları - PWM değerleri (daha geniş aralık)
  else if (command == "SMİN") {
    motorSpeed = 20;   // PWM 20 - Minimum hız (çok düşük)
    updateMotorSpeeds();  // Anlık güncelleme
    Serial.print("SPEED_SET_MIN:");
    Serial.println(motorSpeed);
  }
  else if (command == "S%25") {
    motorSpeed = 64;   // PWM 64 - %25 hız (25% of 255)
    updateMotorSpeeds();  // Anlık güncelleme
    Serial.print("SPEED_SET_25:");
    Serial.println(motorSpeed);
  }
  else if (command == "S%50") {
    motorSpeed = 128;  // PWM 128 - %50 hız (50% of 255)
    updateMotorSpeeds();  // Anlık güncelleme
    Serial.print("SPEED_SET_50:");
    Serial.println(motorSpeed);
  }
  else if (command == "S%75") {
    motorSpeed = 192;  // PWM 192 - %75 hız (75% of 255)
    updateMotorSpeeds();  // Anlık güncelleme
    Serial.print("SPEED_SET_75:");
    Serial.println(motorSpeed);
  }
  else if (command == "SMAX") {
    motorSpeed = 255;  // PWM 255 - Maximum hız
    updateMotorSpeeds();  // Anlık güncelleme
    Serial.print("SPEED_SET_MAX:");
    Serial.println(motorSpeed);
  }
  
  // EM 806 Axis lock/unlock komutları - Sadece Enable pin kontrolü
  else if (command == "lX") {
    xAxisEnabled = false;        // Ekseni kilitle
    digitalWrite(xEnaPin, HIGH); // Motor sürücüyü devre dışı bırak
    Serial.println("AXIS_LOCKED_X");
  }
  else if (command == "ulX") {
    xAxisEnabled = true;         // Ekseni aç
    digitalWrite(xEnaPin, LOW);  // Motor sürücüyü aktif et
    Serial.println("AXIS_UNLOCKED_X");
  }
  else if (command == "lY") {
    yAxisEnabled = false;        // Ekseni kilitle
    digitalWrite(yEnaPin, HIGH); // Motor sürücüyü devre dışı bırak
    Serial.println("AXIS_LOCKED_Y");
  }
  else if (command == "ulY") {
    yAxisEnabled = true;         // Ekseni aç
    digitalWrite(yEnaPin, LOW);  // Motor sürücüyü aktif et
    Serial.println("AXIS_UNLOCKED_Y");
  }
  else if (command == "lZ") {
    zAxisEnabled = false;        // Ekseni kilitle
    digitalWrite(zEnaPin, HIGH); // Motor sürücüyü devre dışı bırak
    Serial.println("AXIS_LOCKED_Z");
  }
  else if (command == "ulZ") {
    zAxisEnabled = true;         // Ekseni aç
    digitalWrite(zEnaPin, LOW);  // Motor sürücüyü aktif et
    Serial.println("AXIS_UNLOCKED_Z");
  }
  
  // (Test komutları kaldırıldı)
  
  else {
    Serial.print("UNKNOWN_COMMAND: ");
    Serial.println(command);
  }
}

void moveMotor(char axis, bool positive, int steps) {
  int pulPin, dirPin;
  
  // Hangi motor olduğunu belirle
  if (axis == 'X') {
    pulPin = xPulPin;
    dirPin = xDirPin;
  }
  else if (axis == 'Y') {  // Y artık eski Z pinlerini kullanıyor
    pulPin = yPulPin;
    dirPin = yDirPin;
  }
  else if (axis == 'Z') {  // Z artık eski Y pinlerini kullanıyor
    pulPin = zPulPin;
    dirPin = zDirPin;
  }
  else {
    return;
  }
  
  // Motor hareket et ve LED göster
  
  // Motor pinlerini PWM ile LED olarak kullan (touchscreen komutları için)
  if (positive) {
    analogWrite(pulPin, motorSpeed);  // Pulse pin PWM ile yanar
    analogWrite(dirPin, motorSpeed);  // Direction pin + yönde yanar
  } else {
    analogWrite(pulPin, motorSpeed);  // Pulse pin PWM ile yanar
    digitalWrite(dirPin, LOW);        // Direction pin - yönde kapalı
  }
  
  // LED'i 500ms göster 
  delay(500);
  
  // LED'i kapat
  digitalWrite(pulPin, LOW);
  digitalWrite(dirPin, LOW);
  
  Serial.print("MOTOR_MOVED_");
  Serial.print(axis);
  Serial.print(positive ? "+" : "-");
  Serial.print("_STEPS:");
  Serial.println(steps);
}

void sendFeedback() {
  // Motor durumu
  Serial.print("MOTOR_STATUS:");
  Serial.print("xRun=");
  Serial.print(xMotorRunning);
  Serial.print(",xDir=");
  Serial.print(xDirection);
  Serial.print(",yRun=");
  Serial.print(yMotorRunning);
  Serial.print(",yDir=");
  Serial.print(yDirection);
  Serial.print(",zRun=");
  Serial.print(zMotorRunning);
  Serial.print(",zDir=");
  Serial.println(zDirection);
  
  // Step delay (hız) ve PWM değeri
  Serial.print("STEP_DELAY:");
  Serial.println(stepDelay);
  
  Serial.print("PWM_SPEED:");
  Serial.println(motorSpeed);
  
  // Motor pin durumları (LED durumları)
  Serial.print("MOTOR_PINS:");
  Serial.print("xPul=");
  Serial.print(digitalRead(xPulPin));
  Serial.print(",xDir=");
  Serial.print(digitalRead(xDirPin));
  Serial.print(",yPul=");
  Serial.print(digitalRead(yPulPin));
  Serial.print(",yDir=");
  Serial.print(digitalRead(yDirPin));
  Serial.print(",zPul=");
  Serial.print(digitalRead(zPulPin));
  Serial.print(",zDir=");
  Serial.println(digitalRead(zDirPin));
}

// Serial event handler
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
