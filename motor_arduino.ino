// MOTOR CONTROL ARDUINO - Joystick ve Step Motor Kontrolü
// Python touchscreen uygulaması ile entegre
// Dual Arduino sisteminde motor kontrol birimi

// Analog joystick pinleri
const int xJoyPin = A0;
const int yJoyPin = A1;

// Fren switch pinleri
const int adminSwitchPin = 2;
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

// PWM LED pinleri - Motor test için
const int LED_X_PLUS = A3;    // X+ motor LED
const int LED_X_MINUS = A4;   // X- motor LED  
const int LED_Y_PLUS = A5;    // Y+ motor LED
const int LED_Y_MINUS = 14;   // Y- motor LED (A0 dijital olarak)
const int LED_Z_PLUS = 15;    // Z+ motor LED (A1 dijital olarak)
const int LED_Z_MINUS = 16;   // Z- motor LED (A2 dijital olarak - farklı pin)

// Motor kontrol değişkenleri
const int threshold = 100;  // Joystick deadzone
int stepDelay = 1000;       // Mikrosaniye cinsinden step delay (hız kontrolü)
bool motorXEnabled = false;
bool motorYEnabled = false;
bool motorZEnabled = false;

// LED test değişkenleri
bool ledTestMode = false;
int currentLEDBrightness = 255;

// Serial komut buffer
String inputString = "";
bool stringComplete = false;

// Feedback timer
unsigned long lastFeedbackTime = 0;
const unsigned long feedbackInterval = 1000; // 1 saniye

void setup() {
  Serial.begin(9600);
  
  // Switch pinlerini input olarak ayarla
  pinMode(adminSwitchPin, INPUT);
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

  // LED pinlerini output olarak ayarla
  pinMode(LED_X_PLUS, OUTPUT);
  pinMode(LED_X_MINUS, OUTPUT);
  pinMode(LED_Y_PLUS, OUTPUT);
  pinMode(LED_Y_MINUS, OUTPUT);
  pinMode(LED_Z_PLUS, OUTPUT);
  pinMode(LED_Z_MINUS, OUTPUT);

  // Başlangıçta motorları devre dışı bırak
  digitalWrite(xEnaPin, HIGH);
  digitalWrite(yEnaPin, HIGH);
  digitalWrite(zEnaPin, HIGH);
  
  // LED'leri kapat
  analogWrite(LED_X_PLUS, 0);
  analogWrite(LED_X_MINUS, 0);
  analogWrite(LED_Y_PLUS, 0);
  analogWrite(LED_Y_MINUS, 0);
  analogWrite(LED_Z_PLUS, 0);
  analogWrite(LED_Z_MINUS, 0);

  // Reserve space for the input string
  inputString.reserve(200);
  
  Serial.println("MOTOR_ARDUINO_READY");
  Serial.println("Joystick ve Step Motor Kontrolü Aktif");
}

void loop() {
  // Serial komutları işle
  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Switch durumlarını oku
  bool admin = digitalRead(adminSwitchPin);
  bool xFren = digitalRead(xFrenSwitchPin);
  bool yFren = digitalRead(yFrenSwitchPin);
  bool zFren = digitalRead(zFrenSwitchPin);

  // Admin switch kapalıysa tüm motorları durdur
  if (!admin) {
    digitalWrite(xEnaPin, HIGH);
    digitalWrite(yEnaPin, HIGH);
    digitalWrite(zEnaPin, HIGH);
    motorXEnabled = false;
    motorYEnabled = false;
    motorZEnabled = false;
    
    // Feedback gönder
    sendFeedback();
    return;
  }

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

  // Joystick kontrolü (sadece admin switch açık ve fren kapalıysa)
  if (admin) {
    controlMotorsWithJoystick(xFren, yFren);
  }
  
  // Periyodik feedback gönder
  sendPeriodicFeedback();
}

void controlMotorsWithJoystick(bool xFren, bool yFren) {
  // Joystick verilerini oku
  int xVal = analogRead(xJoyPin) - 512;
  int yVal = analogRead(yJoyPin) - 512;

  // X eksen kontrolü
  if (abs(xVal) > threshold && !xFren && motorXEnabled) {
    digitalWrite(xDirPin, xVal > 0 ? HIGH : LOW);
    digitalWrite(xPulPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(xPulPin, LOW);
    delayMicroseconds(stepDelay);
  }

  // Y eksen kontrolü
  if (abs(yVal) > threshold && !yFren && motorYEnabled) {
    digitalWrite(yDirPin, yVal > 0 ? HIGH : LOW);
    digitalWrite(yPulPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(yPulPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

void processSerialCommand(String command) {
  command.trim();
  
  // IDENTIFY komutu - Arduino tipini belirt
  if (command == "IDENTIFY") {
    Serial.println("MOTOR_ARDUINO");
    Serial.println("JOYSTICK_CONTROL");
    return;
  }
  
  // Motor hareket komutları
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
    Serial.println("X_MOTOR_STOPPED");
  }
  else if (command == "STOPY") {
    motorYEnabled = false;
    digitalWrite(yEnaPin, HIGH);
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
    Serial.println("AXIS_LOCKED_X");
  }
  else if (command == "ulX") {
    motorXEnabled = true;
    Serial.println("AXIS_UNLOCKED_X");
  }
  else if (command == "lY") {
    motorYEnabled = false;
    digitalWrite(yEnaPin, HIGH);
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
  
  // LED test komutları
  else if (command == "LED_TEST_ON") {
    ledTestMode = true;
    Serial.println("LED_TEST_MODE_ON");
  }
  else if (command == "LED_TEST_OFF") {
    ledTestMode = false;
    turnOffAllLEDs();
    Serial.println("LED_TEST_MODE_OFF");
  }
  else if (command == "LED_TEST_SEQUENCE") {
    testLEDSequence();
  }
  else if (command == "LED_ALL_OFF") {
    turnOffAllLEDs();
    Serial.println("ALL_LEDS_OFF");
  }
  
  // Motor LED test komutları
  else if (command.startsWith("LED_X+_")) {
    int brightness = command.substring(7).toInt();
    brightness = constrain(brightness, 0, 255);
    analogWrite(LED_X_PLUS, brightness);
    Serial.print("LED_X_PLUS_SET:");
    Serial.println(brightness);
  }
  else if (command.startsWith("LED_X-_")) {
    int brightness = command.substring(7).toInt();
    brightness = constrain(brightness, 0, 255);
    analogWrite(LED_X_MINUS, brightness);
    Serial.print("LED_X_MINUS_SET:");
    Serial.println(brightness);
  }
  else if (command.startsWith("LED_Y+_")) {
    int brightness = command.substring(7).toInt();
    brightness = constrain(brightness, 0, 255);
    analogWrite(LED_Y_PLUS, brightness);
    Serial.print("LED_Y_PLUS_SET:");
    Serial.println(brightness);
  }
  else if (command.startsWith("LED_Y-_")) {
    int brightness = command.substring(7).toInt();
    brightness = constrain(brightness, 0, 255);
    analogWrite(LED_Y_MINUS, brightness);
    Serial.print("LED_Y_MINUS_SET:");
    Serial.println(brightness);
  }
  else if (command.startsWith("LED_Z+_")) {
    int brightness = command.substring(7).toInt();
    brightness = constrain(brightness, 0, 255);
    analogWrite(LED_Z_PLUS, brightness);
    Serial.print("LED_Z_PLUS_SET:");
    Serial.println(brightness);
  }
  else if (command.startsWith("LED_Z-_")) {
    int brightness = command.substring(7).toInt();
    brightness = constrain(brightness, 0, 255);
    analogWrite(LED_Z_MINUS, brightness);
    Serial.print("LED_Z_MINUS_SET:");
    Serial.println(brightness);
  }
  
  else {
    Serial.print("UNKNOWN_COMMAND: ");
    Serial.println(command);
  }
}

void moveMotor(char axis, bool positive, int steps) {
  int pulPin, dirPin, enaPin;
  bool *motorEnabled;
  
  // Hangi motor olduğunu belirle
  if (axis == 'X') {
    pulPin = xPulPin;
    dirPin = xDirPin;
    enaPin = xEnaPin;
    motorEnabled = &motorXEnabled;
  }
  else if (axis == 'Y') {
    pulPin = yPulPin;
    dirPin = yDirPin;
    enaPin = yEnaPin;
    motorEnabled = &motorYEnabled;
  }
  else if (axis == 'Z') {
    pulPin = zPulPin;
    dirPin = zDirPin;
    enaPin = zEnaPin;
    motorEnabled = &motorZEnabled;
  }
  else {
    return;
  }
  
  // Motor etkinleştir
  *motorEnabled = true;
  
  // Admin switch kontrolü
  if (!digitalRead(adminSwitchPin)) {
    Serial.println("ADMIN_SWITCH_OFF");
    return;
  }
  
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
  
  // LED feedback - motor hareketini LED ile göster
  if (ledTestMode) {
    showMotorMovementOnLED(axis, positive);
  }
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(stepDelay);
  }
  
  // LED'i kapat (hareket bitince)
  if (ledTestMode) {
    delay(200); // LED'i 200ms göster
    turnOffMotorLED(axis, positive);
  }
  
  Serial.print("MOTOR_MOVED_");
  Serial.print(axis);
  Serial.print(positive ? "+" : "-");
  Serial.print("_STEPS:");
  Serial.println(steps);
}

void sendFeedback() {
  // Switch durumları
  Serial.print("SWITCHES:");
  Serial.print("admin=");
  Serial.print(digitalRead(adminSwitchPin));
  Serial.print(",xFren=");
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
}

void sendPeriodicFeedback() {
  unsigned long currentTime = millis();
  if (currentTime - lastFeedbackTime >= feedbackInterval) {
    sendFeedback();
    lastFeedbackTime = currentTime;
  }
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

// LED kontrol fonksiyonları
void showMotorMovementOnLED(char axis, bool positive) {
  // Hıza göre LED parlaklığını ayarla (yavaş = az parlak, hızlı = çok parlak)
  int brightness = map(stepDelay, 300, 2000, 255, 50); // Ters mapping: az delay = parlak
  brightness = constrain(brightness, 50, 255);
  
  int ledPin = -1;
  
  if (axis == 'X') {
    ledPin = positive ? LED_X_PLUS : LED_X_MINUS;
  }
  else if (axis == 'Y') {
    ledPin = positive ? LED_Y_PLUS : LED_Y_MINUS;
  }
  else if (axis == 'Z') {
    ledPin = positive ? LED_Z_PLUS : LED_Z_MINUS;
  }
  
  if (ledPin != -1) {
    analogWrite(ledPin, brightness);
  }
}

void turnOffMotorLED(char axis, bool positive) {
  int ledPin = -1;
  
  if (axis == 'X') {
    ledPin = positive ? LED_X_PLUS : LED_X_MINUS;
  }
  else if (axis == 'Y') {
    ledPin = positive ? LED_Y_PLUS : LED_Y_MINUS;
  }
  else if (axis == 'Z') {
    ledPin = positive ? LED_Z_PLUS : LED_Z_MINUS;
  }
  
  if (ledPin != -1) {
    analogWrite(ledPin, 0);
  }
}

void turnOffAllLEDs() {
  analogWrite(LED_X_PLUS, 0);
  analogWrite(LED_X_MINUS, 0);
  analogWrite(LED_Y_PLUS, 0);
  analogWrite(LED_Y_MINUS, 0);
  analogWrite(LED_Z_PLUS, 0);
  analogWrite(LED_Z_MINUS, 0);
}

void testLEDSequence() {
  Serial.println("LED_TEST_SEQUENCE_START");
  
  // Her LED'i sırayla test et - farklı parlaklık seviyeleri
  int brightness[] = {50, 100, 150, 200, 255};
  
  for (int i = 0; i < 5; i++) {
    Serial.print("Testing brightness level: ");
    Serial.println(brightness[i]);
    
    // X+ LED
    analogWrite(LED_X_PLUS, brightness[i]);
    delay(300);
    analogWrite(LED_X_PLUS, 0);
    delay(100);
    
    // X- LED
    analogWrite(LED_X_MINUS, brightness[i]);
    delay(300);
    analogWrite(LED_X_MINUS, 0);
    delay(100);
    
    // Y+ LED
    analogWrite(LED_Y_PLUS, brightness[i]);
    delay(300);
    analogWrite(LED_Y_PLUS, 0);
    delay(100);
    
    // Y- LED
    analogWrite(LED_Y_MINUS, brightness[i]);
    delay(300);
    analogWrite(LED_Y_MINUS, 0);
    delay(100);
    
    // Z+ LED
    analogWrite(LED_Z_PLUS, brightness[i]);
    delay(300);
    analogWrite(LED_Z_PLUS, 0);
    delay(100);
    
    // Z- LED
    analogWrite(LED_Z_MINUS, brightness[i]);
    delay(300);
    analogWrite(LED_Z_MINUS, 0);
    delay(100);
  }
  
  // Final test - tüm LED'leri aynı anda
  Serial.println("All LEDs together test");
  for (int b = 0; b <= 255; b += 51) {
    analogWrite(LED_X_PLUS, b);
    analogWrite(LED_X_MINUS, b);
    analogWrite(LED_Y_PLUS, b);
    analogWrite(LED_Y_MINUS, b);
    analogWrite(LED_Z_PLUS, b);
    analogWrite(LED_Z_MINUS, b);
    delay(200);
  }
  
  turnOffAllLEDs();
  Serial.println("LED_TEST_SEQUENCE_COMPLETE");
}