// MOTOR CONTROL ARDUINO - Joystick ve Touchscreen Kontrolü
// Python touchscreen uygulaması ile entegre
// Dual Arduino sisteminde motor kontrol birimi
// Admin switch kaldırıldı - Sadece fren switch'leri ile kontrol
// YÖN LED'LERİ EKLENDİ - Her yön için ayrı LED

// Analog joystick pinleri
const int xJoyPin = A0;
const int yJoyPin = A1;

// Fren switch pinleri (admin switch kaldırıldı)
const int xFrenSwitchPin = 3;
const int yFrenSwitchPin = 4;
const int zFrenSwitchPin = 5;

// X eksen pinleri
const int xPulPin = 6;
const int xDirPin = 7;
const int xEnaPin = 8;

// Y eksen pinleri
const int yPulPin = 9;
const int yDirPin = 10;
const int yEnaPin = 11;

// Z eksen pinleri
const int zPulPin = 12;
const int zDirPin = 13;
const int zEnaPin = A2;

// YÖN LED PİNLERİ - Her yön için ayrı LED
const int xPlusLedPin = 2;    // X+ LED
const int xMinusLedPin = A3;  // X- LED
const int yPlusLedPin = A4;   // Y+ LED
const int yMinusLedPin = A5;  // Y- LED
// Z için henüz pin yok, gerekirse ek pin kullanın

// Motor kontrol değişkenleri
const int threshold = 100;  // Joystick deadzone
int stepDelay = 500;        // Mikrosaniye cinsinden step delay (hız kontrolü)
bool motorXEnabled = false;
bool motorYEnabled = false;
bool motorZEnabled = false;

// Serial komut buffer
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  
  // Switch pinlerini input olarak ayarla (admin switch kaldırıldı)
  pinMode(xFrenSwitchPin, INPUT);
  pinMode(yFrenSwitchPin, INPUT);
  pinMode(zFrenSwitchPin, INPUT);

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

  // YÖN LED pinlerini output olarak ayarla
  pinMode(xPlusLedPin, OUTPUT);
  pinMode(xMinusLedPin, OUTPUT);
  pinMode(yPlusLedPin, OUTPUT);
  pinMode(yMinusLedPin, OUTPUT);

  // Başlangıçta motorları devre dışı bırak
  digitalWrite(xEnaPin, HIGH);
  digitalWrite(yEnaPin, HIGH);
  digitalWrite(zEnaPin, HIGH);

  // Başlangıçta tüm LED'leri kapat
  digitalWrite(xPlusLedPin, LOW);
  digitalWrite(xMinusLedPin, LOW);
  digitalWrite(yPlusLedPin, LOW);
  digitalWrite(yMinusLedPin, LOW);

  // Reserve space for the input string
  inputString.reserve(200);
  
  Serial.println("MOTOR_ARDUINO_READY");
  Serial.println("Joystick ve Touchscreen Kontrolü Aktif - No Admin Switch - Direction LEDs Added");
}

void loop() {
  // Serial komutları işle (Touchscreen kontrolü)
  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Fren switch durumlarını oku
  bool xFren = digitalRead(xFrenSwitchPin);
  bool yFren = digitalRead(yFrenSwitchPin);
  bool zFren = digitalRead(zFrenSwitchPin);

  // Fren switch'lerine göre motor enable durumlarını ayarla
  if (!xFren && motorXEnabled) {
    digitalWrite(xEnaPin, LOW);  // Enable motor
  } else {
    digitalWrite(xEnaPin, HIGH); // Disable motor
  }
  
  if (!yFren && motorYEnabled) {
    digitalWrite(yEnaPin, LOW);
  } else {
    digitalWrite(yEnaPin, HIGH);
  }
  
  if (!zFren && motorZEnabled) {
    digitalWrite(zEnaPin, LOW);
  } else {
    digitalWrite(zEnaPin, HIGH);
  }

  // Joystick kontrolü (fren kapalıysa)
  controlMotorsWithJoystick(xFren, yFren);
}

void controlMotorsWithJoystick(bool xFren, bool yFren) {
  // Joystick verilerini oku
  int xVal = analogRead(xJoyPin) - 512;
  int yVal = analogRead(yJoyPin) - 512;

  // X eksen kontrolü (joystick ile)
  if (abs(xVal) > threshold && !xFren && motorXEnabled) {
    bool xPositive = xVal > 0;
    digitalWrite(xDirPin, xPositive ? HIGH : LOW);
    
    // Yön LED'lerini kontrol et
    digitalWrite(xPlusLedPin, xPositive ? HIGH : LOW);
    digitalWrite(xMinusLedPin, xPositive ? LOW : HIGH);
    
    digitalWrite(xPulPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(xPulPin, LOW);
    delayMicroseconds(stepDelay);
  } else {
    // Hareket yoksa X LED'lerini kapat
    digitalWrite(xPlusLedPin, LOW);
    digitalWrite(xMinusLedPin, LOW);
  }

  // Y eksen kontrolü (joystick ile)
  if (abs(yVal) > threshold && !yFren && motorYEnabled) {
    bool yPositive = yVal > 0;
    digitalWrite(yDirPin, yPositive ? HIGH : LOW);
    
    // Yön LED'lerini kontrol et
    digitalWrite(yPlusLedPin, yPositive ? HIGH : LOW);
    digitalWrite(yMinusLedPin, yPositive ? LOW : HIGH);
    
    digitalWrite(yPulPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(yPulPin, LOW);
    delayMicroseconds(stepDelay);
  } else {
    // Hareket yoksa Y LED'lerini kapat
    digitalWrite(yPlusLedPin, LOW);
    digitalWrite(yMinusLedPin, LOW);
  }
}

void processSerialCommand(String command) {
  command.trim();
  
  // IDENTIFY komutu - Arduino tipini belirt
  if (command == "IDENTIFY") {
    Serial.println("MOTOR_ARDUINO");
    Serial.println("JOYSTICK_CONTROL_NO_ADMIN_DIRECTION_LEDS");
    return;
  }
  
  // Motor hareket komutları (Touchscreen kontrolü)
  if (command == "X+") {
    moveMotor('X', true, 100);
  }
  else if (command == "X-") {
    moveMotor('X', false, 100);
  }
  else if (command == "Y+") {
    moveMotor('Y', true, 100);
  }
  else if (command == "Y-") {
    moveMotor('Y', false, 100);
  }
  else if (command == "Z+") {
    moveMotor('Z', true, 100);
  }
  else if (command == "Z-") {
    moveMotor('Z', false, 100);
  }
  
  // Motor durdurma komutları
  else if (command == "STOPX") {
    motorXEnabled = false;
    digitalWrite(xEnaPin, HIGH);
    // X LED'lerini kapat
    digitalWrite(xPlusLedPin, LOW);
    digitalWrite(xMinusLedPin, LOW);
    Serial.println("X_MOTOR_STOPPED");
  }
  else if (command == "STOPY") {
    motorYEnabled = false;
    digitalWrite(yEnaPin, HIGH);
    // Y LED'lerini kapat
    digitalWrite(yPlusLedPin, LOW);
    digitalWrite(yMinusLedPin, LOW);
    Serial.println("Y_MOTOR_STOPPED");
  }
  else if (command == "STOPZ") {
    motorZEnabled = false;
    digitalWrite(zEnaPin, HIGH);
    Serial.println("Z_MOTOR_STOPPED");
  }
  
  // Hız kontrol komutları
  else if (command == "SMIN") {
    stepDelay = 2000;
    Serial.println("SPEED_SET_MIN");
  }
  else if (command == "S%25") {
    stepDelay = 1500;
    Serial.println("SPEED_SET_25");
  }
  else if (command == "S%50") {
    stepDelay = 1000;
    Serial.println("SPEED_SET_50");
  }
  else if (command == "S%75") {
    stepDelay = 600;
    Serial.println("SPEED_SET_75");
  }
  else if (command == "SMAX") {
    stepDelay = 300;
    Serial.println("SPEED_SET_MAX");
  }
  
  // Axis lock/unlock komutları
  else if (command == "lX") {
    motorXEnabled = false;
    digitalWrite(xEnaPin, HIGH);
    digitalWrite(xPlusLedPin, LOW);
    digitalWrite(xMinusLedPin, LOW);
    Serial.println("AXIS_LOCKED_X");
  }
  else if (command == "ulX") {
    motorXEnabled = true;
    Serial.println("AXIS_UNLOCKED_X");
  }
  else if (command == "lY") {
    motorYEnabled = false;
    digitalWrite(yEnaPin, HIGH);
    digitalWrite(yPlusLedPin, LOW);
    digitalWrite(yMinusLedPin, LOW);
    Serial.println("AXIS_LOCKED_Y");
  }
  else if (command == "ulY") {
    motorYEnabled = true;
    Serial.println("AXIS_UNLOCKED_Y");
  }
  else if (command == "lZ") {
    motorZEnabled = false;
    digitalWrite(zEnaPin, HIGH);
    Serial.println("AXIS_LOCKED_Z");
  }
  else if (command == "ulZ") {
    motorZEnabled = true;
    Serial.println("AXIS_UNLOCKED_Z");
  }
  
  // Test komutu
  else if (command == "TEST") {
    Serial.println("MOTOR_ARDUINO_TEST_OK");
    sendFeedback();
  }
  
  // LED Test komutu - GÜNCELLENMIŞ
  else if (command == "LEDTEST") {
    Serial.println("LED_TEST_STARTING");
    
    // Yön LED'lerini test et
    for (int i = 0; i < 3; i++) {
      // X+ LED
      digitalWrite(xPlusLedPin, HIGH);
      delay(300);
      digitalWrite(xPlusLedPin, LOW);
      delay(200);
      
      // X- LED
      digitalWrite(xMinusLedPin, HIGH);
      delay(300);
      digitalWrite(xMinusLedPin, LOW);
      delay(200);
      
      // Y+ LED
      digitalWrite(yPlusLedPin, HIGH);
      delay(300);
      digitalWrite(yPlusLedPin, LOW);
      delay(200);
      
      // Y- LED
      digitalWrite(yMinusLedPin, HIGH);
      delay(300);
      digitalWrite(yMinusLedPin, LOW);
      delay(200);
      
      // Z LED (pulse pin)
      digitalWrite(zPulPin, HIGH);
      delay(300);
      digitalWrite(zPulPin, LOW);
      delay(200);
    }
    
    Serial.println("LED_TEST_COMPLETED");
  }
  
  else {
    Serial.print("UNKNOWN_COMMAND: ");
    Serial.println(command);
  }
}

void moveMotor(char axis, bool positive, int steps) {
  int pulPin, dirPin, enaPin;
  int plusLedPin = -1, minusLedPin = -1;
  bool *motorEnabled;
  
  // Hangi motor olduğunu belirle
  if (axis == 'X') {
    pulPin = xPulPin;
    dirPin = xDirPin;
    enaPin = xEnaPin;
    plusLedPin = xPlusLedPin;
    minusLedPin = xMinusLedPin;
    motorEnabled = &motorXEnabled;
  }
  else if (axis == 'Y') {
    pulPin = yPulPin;
    dirPin = yDirPin;
    enaPin = yEnaPin;
    plusLedPin = yPlusLedPin;
    minusLedPin = yMinusLedPin;
    motorEnabled = &motorYEnabled;
  }
  else if (axis == 'Z') {
    pulPin = zPulPin;
    dirPin = zDirPin;
    enaPin = zEnaPin;
    // Z için henüz yön LED'i yok
    motorEnabled = &motorZEnabled;
  }
  else {
    return;
  }
  
  // Motor etkinleştir
  *motorEnabled = true;
  
  // Fren switch kontrolü
  bool frenActive = false;
  if (axis == 'X') frenActive = digitalRead(xFrenSwitchPin);
  else if (axis == 'Y') frenActive = digitalRead(yFrenSwitchPin);
  else if (axis == 'Z') frenActive = digitalRead(zFrenSwitchPin);
  
  if (frenActive) {
    Serial.print("BRAKE_ACTIVE_");
    Serial.println(axis);
    return;
  }
  
  // Motor hareket et
  digitalWrite(enaPin, LOW);  // Motor'u etkinleştir
  digitalWrite(dirPin, positive ? HIGH : LOW);  // Yön ayarla
  
  // Yön LED'lerini ayarla (X ve Y için)
  if (plusLedPin != -1 && minusLedPin != -1) {
    digitalWrite(plusLedPin, positive ? HIGH : LOW);
    digitalWrite(minusLedPin, positive ? LOW : HIGH);
  }
  
  // Motor pulse 
  for (int i = 0; i < steps; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(stepDelay);
  }
  
  // Hareket bitince yön LED'lerini kapat
  if (plusLedPin != -1 && minusLedPin != -1) {
    digitalWrite(plusLedPin, LOW);
    digitalWrite(minusLedPin, LOW);
  }
  
  Serial.print("MOTOR_MOVED_");
  Serial.print(axis);
  Serial.print(positive ? "+" : "-");
  Serial.print("_STEPS:");
  Serial.println(steps);
}

void sendFeedback() {
  // Switch durumları (admin switch kaldırıldı)
  Serial.print("SWITCHES:");
  Serial.print("xFren=");
  Serial.print(digitalRead(xFrenSwitchPin));
  Serial.print(",yFren=");
  Serial.print(digitalRead(yFrenSwitchPin));
  Serial.print(",zFren=");
  Serial.println(digitalRead(zFrenSwitchPin));
  
  // Motor enable durumları
  Serial.print("MOTORS:");
  Serial.print("xEna=");
  Serial.print(motorXEnabled);
  Serial.print(",yEna=");
  Serial.print(motorYEnabled);
  Serial.print(",zEna=");
  Serial.println(motorZEnabled);
  
  // Joystick değerleri
  int xJoy = analogRead(xJoyPin);
  int yJoy = analogRead(yJoyPin);
  Serial.print("JOYSTICK:");
  Serial.print("x=");
  Serial.print(xJoy);
  Serial.print(",y=");
  Serial.println(yJoy);
  
  // Step delay (hız)
  Serial.print("STEP_DELAY:");
  Serial.println(stepDelay);
  
  // LED durumları
  Serial.print("LEDS:");
  Serial.print("X+=");
  Serial.print(digitalRead(xPlusLedPin));
  Serial.print(",X-=");
  Serial.print(digitalRead(xMinusLedPin));
  Serial.print(",Y+=");
  Serial.print(digitalRead(yPlusLedPin));
  Serial.print(",Y-=");
  Serial.println(digitalRead(yMinusLedPin));
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