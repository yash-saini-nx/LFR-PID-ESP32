#include <SparkFun_TB6612.h>

// -------- LINE SETTINGS --------
bool isBlackLine = 1;
unsigned int lineThickness = 25;
unsigned int numSensors = 5;
bool brakeEnabled = 0;

// -------- TB6612 CONNECTIONS --------
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
int sensorPins[5] = {4,5,6,7,15};

// Motor orientation
const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// -------- PID VARIABLES --------
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfSpeed = 120;
int currentSpeed = 30;

float Kp = 0.06;
float Kd = 1.5;
float Ki = 0;

int onLine = 1;

int minValues[7], maxValues[7], threshold[7];
int sensorValue[7];
bool brakeFlag = 0;

// --------------------------------

void setup() {

  Serial.begin(115200);

  analogReadResolution(12);

  pinMode(CAL_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  lineThickness = constrain(lineThickness, 10, 35);

  Serial.println("LFR Ready");
}

// --------------------------------

void loop() {

  while (digitalRead(CAL_BUTTON)) {}
  delay(1000);

  calibrate();

  while (digitalRead(START_BUTTON)) {}
  delay(1000);

  while (1) {

    readLine();

    if (currentSpeed < lfSpeed) currentSpeed++;

    if (onLine == 1) {

      linefollow();

      digitalWrite(LED_PIN, HIGH);
      brakeFlag = 0;
    }

    else {

      digitalWrite(LED_PIN, LOW);

      if (error > 0) {

        if (brakeEnabled == 1 && brakeFlag == 0) {
          motor1.drive(0);
          motor2.drive(0);
          delay(30);
        }

        motor1.drive(-100);
        motor2.drive(150);
        brakeFlag = 1;
      }

      else {

        if (brakeEnabled == 1 && brakeFlag == 0) {
          motor1.drive(0);
          motor2.drive(0);
          delay(30);
        }

        motor1.drive(150);
        motor2.drive(-100);
        brakeFlag = 1;
      }
    }
  }
}

// --------------------------------

void linefollow() {

  if (numSensors == 5) {
    error = (3 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 3 * sensorValue[5]);
  }

  if (lineThickness > 22) error = error * -1;

  if (isBlackLine) error = error * -1;

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);

  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  lsp = constrain(lsp,0,255);
  rsp = constrain(rsp,0,255);

  motor1.drive(lsp);
  motor2.drive(rsp);
}

// --------------------------------

void calibrate() {

  for (int i = 1; i < 6; i++) {

    int pin = sensorPins[i-1];

    minValues[i] = analogRead(pin);
    maxValues[i] = analogRead(pin);
  }

  for (int i = 0; i < 10000; i++) {

    motor1.drive(50);
    motor2.drive(-50);

    for (int j = 1; j < 6; j++) {

      int pin = sensorPins[j-1];
      int val = analogRead(pin);

      if (val < minValues[j]) minValues[j] = val;
      if (val > maxValues[j]) maxValues[j] = val;
    }
  }

  for (int i = 1; i < 6; i++) {

    threshold[i] = (minValues[i] + maxValues[i]) / 2;

    Serial.print(threshold[i]);
    Serial.print(" ");
  }

  Serial.println();

  motor1.drive(0);
  motor2.drive(0);
}

// --------------------------------

void readLine() {

  onLine = 0;

  for (int i = 1; i < 6; i++) {

    int pin = sensorPins[i-1];

    sensorValue[i] = map(analogRead(pin), minValues[i], maxValues[i], 0, 1000);

    sensorValue[i] = constrain(sensorValue[i],0,1000);

    if (isBlackLine == 1 && sensorValue[i] > 700) onLine = 1;
    if (isBlackLine == 0 && sensorValue[i] < 700) onLine = 1;
  }
}