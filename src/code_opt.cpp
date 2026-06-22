// -------- LINE SETTINGS --------
bool isBlackLine = 1;

// -------- TB6612 PINS (ESP32) --------
#define AIN1 21
#define BIN1 37
#define AIN2 39
#define BIN2 36
#define PWMA 47
#define PWMB 35
#define STBY 2

// -------- BUTTONS / LED --------
#define CAL_BUTTON   41
#define START_BUTTON 42
#define LED_PIN      48

// -------- SENSOR PINS --------
int sensorPins[5] = {4, 5, 6, 7, 15};

// -------- SPEED (Adjusted Lower) --------
int baseSpeed = 90;      // Cruise speed
int maxSpeed = 150;      // Speed limit for PID
int pivotSpeed = 130;    // Stronger burst for sharp turns

// -------- PID (Adjusted Lower) --------
float Kp = 0.32;         
float Kd = 1.0;          
float Ki = 0.001;        

float P, I, D;
int previousError = 0;
int minValues[5], maxValues[5], sensorValue[5];
bool onLine = false;

// ---------------- MOTOR FUNCTIONS ----------------

void motor1(int speed) { // Left Motor
  speed = constrain(speed, -255, 255);
  digitalWrite(AIN1, speed >= 0 ? LOW : HIGH);
  digitalWrite(AIN2, speed >= 0 ? HIGH : LOW);
  analogWrite(PWMA, abs(speed));
}

void motor2(int speed) { // Right Motor
  speed = constrain(speed, -255, 255);
  digitalWrite(BIN1, speed >= 0 ? HIGH : LOW);
  digitalWrite(BIN2, speed >= 0 ? LOW : HIGH);
  analogWrite(PWMB, abs(speed));
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(CAL_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(STBY, HIGH);
}

// ---------------- LOOP ----------------

void loop() {
  // Wait for Calibration Button
  while(digitalRead(CAL_BUTTON)); 
  delay(500);
  calibrate();

  // Wait for Start Button
  while(digitalRead(START_BUTTON));
  delay(500);

  while(1) {
    readSensors();
    
    // --- STATE 1: SHARP LEFT TURN ---
    // Detects when the line is only on the far-left sensor
    if (sensorValue[0] > 750 && sensorValue[2] < 300) {
      while(analogRead(sensorPins[2]) < (minValues[2] + 600)) { 
        motor1(-pivotSpeed); motor2(pivotSpeed); 
      }
    } 
    // --- STATE 2: SHARP RIGHT TURN ---
    // Detects when the line is only on the far-right sensor
    else if (sensorValue[4] > 750 && sensorValue[2] < 300) {
      while(analogRead(sensorPins[2]) < (minValues[2] + 600)) { 
        motor1(pivotSpeed); motor2(-pivotSpeed);
      }
    }
    // --- STATE 3: PID LINE FOLLOWING ---
    else if (onLine) {
      calculatePID();
    } 
    // --- STATE 4: RECOVERY (LINE LOST) ---
    else {
      if (previousError < 0) { motor1(-100); motor2(100); }
      else { motor1(100); motor2(-100); }
    }
  }
}

// ---------------- LOGIC FUNCTIONS ----------------

void readSensors() {
  onLine = false;
  for (int i = 0; i < 5; i++) {
    int raw = analogRead(sensorPins[i]);
    // Map raw analog values to a 0-1000 range based on calibration
    sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    
    // Check if at least one sensor sees the line
    if (sensorValue[i] > 600) onLine = true;
  }
}

void calculatePID() {
  // Weighting: Far Left (-20), Left (-10), Center (0), Right (10), Far Right (20)
  int error = (-20 * sensorValue[0] - 10 * sensorValue[1] + 10 * sensorValue[3] + 20 * sensorValue[4]) / 10;
  
  P = error;
  I += error;
  I = constrain(I, -100, 100); // Prevent Integral Windup
  D = error - previousError;

  int pidValue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  motor1(baseSpeed + pidValue);
  motor2(baseSpeed - pidValue);
}

void calibrate() {
  digitalWrite(LED_PIN, HIGH);
  for(int i=0; i<5; i++) {
    minValues[i] = 4095;
    maxValues[i] = 0;
  }
  
  long startTime = millis();
  while(millis() - startTime < 4000) { // Calibrate for 4 seconds
    motor1(80); motor2(-80); // Spin during calibration to capture white/black
    for(int i=0; i<5; i++) {
      int val = analogRead(sensorPins[i]);
      if(val < minValues[i]) minValues[i] = val;
      if(val > maxValues[i]) maxValues[i] = val;
    }
  }
  motor1(0); motor2(0);
  digitalWrite(LED_PIN, LOW);
}