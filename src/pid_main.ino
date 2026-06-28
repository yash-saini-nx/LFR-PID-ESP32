#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <math.h>

// ======================================================
//  Wi-Fi — ESP32 runs as Access Point
//  Connect phone/laptop to: LFR_Debug / password: 12345678
//  Then open dashboard and enter: ws://192.168.4.1:81
// ======================================================
const char* AP_SSID = "LFR_Debug";
const char* AP_PASS = "12345678";

WebSocketsServer wsServer(81);

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

// -------- BUTTONS / LED / BUZZER --------
#define CAL_BUTTON   41
#define START_BUTTON 42
#define LED_PIN      48
#define BUZZER_PIN   40

// -------- TOF PINS --------
#define SDA_PIN    17
#define SCL_PIN    18
#define XSHUT_PIN  16

// -------- MPU6050 GY-521 — Wire1 (separate I2C bus) --------
// SDA → GPIO9,  SCL → GPIO10
// Uses Wire1 so it does not conflict with VL53L1X on Wire (GPIO17/18)
#define MPU_SDA  9
#define MPU_SCL  10
#define MPU_ADDR 0x68

// MPU data — complementary filter (pitch/roll) + gyro integration (yaw)
bool  mpuReady = false;
float mpuPitch = 0, mpuRoll = 0, mpuYaw = 0;
float mpuAx = 0, mpuAy = 0, mpuAz = 0;   // m/s²
float mpuGx = 0, mpuGy = 0, mpuGz = 0;   // deg/s
float mpuGForce = 1.0f;
// Calibration offsets (filled during initMPU)
float gxOff=0,gyOff=0,gzOff=0,axOff=0,ayOff=0,azOff=0;
// Complementary filter coefficient — 0.96 = trust gyro, reject accel vibration
const float CF_ALPHA = 0.96f;
// Smart flags
bool  mpuTiltStop   = false;  // pitch or roll > TILT_STOP_DEG → stop bot
bool  mpuImpact     = false;  // accel spike > 3G
unsigned long mpuImpactMs = 0;
unsigned long lastMpuMs   = 0;
const float TILT_STOP_DEG = 50.0f;   // degrees — bot fallen over or steep ramp

// LFR-specific: gyro Z turn integrator for avoidance angle feedback
// Integrates yaw during pivot states so we know how far we actually turned
float pivotYawAccum = 0.0f;
bool  pivotTracking = false;

// PID improvement: gyro Y rate used to damp oscillation
// When bot is oscillating (high gyro Y rate), briefly boost Kd
float mpuKdBoost = 0.0f;   // added to Kd dynamically, decays to 0

// -------- SENSOR PINS --------
int sensorPins[5] = {4, 5, 6, 7, 15};

// -------- SPEED --------
int baseSpeed    = 90;
int maxSpeed     = 150;
int pivotSpeed   = 130;
int avoidSpeed   = 80;
int recoverSpeed = 90;

// -------- PID --------
float Kp = 0.32;
float Kd = 1.0;
float Ki = 0.001;

float P, I, D;
int previousError = 0;
int minValues[5], maxValues[5], sensorValue[5];
bool onLine = false;

// -------- MOTOR TRACKING --------
int lastM1 = 0, lastM2 = 0;

// -------- BUZZER --------
const int buzzerRes = 8;

// -------- TOF --------
Adafruit_VL53L1X tof;
const int OBSTACLE_MM = 160;
int lastDistance = 9999;
bool tofReady = false;

// -------- AVOIDANCE TIMINGS --------
const unsigned long TURN_90_MS        = 300;
const unsigned long BYPASS_FORWARD_MS = 450;
const unsigned long REJOIN_TIMEOUT_MS = 1200;

// -------- TELEMETRY TIMING --------
const unsigned long TELEM_INTERVAL_MS = 50;
unsigned long lastTelemTime = 0;

// ============================================================
//  BATTERY MONITORING
//  Voltage divider: R1=100kΩ (top), R2=47kΩ (bottom) on GPIO8
//  Divider ratio = 47 / (100+47) = 0.31973
//  ESP32 ADC: 12-bit (0-4095), Vref = 3.3V (with attenuation)
//
//  V_gpio = V_batt * 0.31973
//  V_batt  = ADC_raw * (3.3 / 4095) / 0.31973
//
//  Calibration constants — adjust VBATT_CAL if your 3.3V rail
//  is slightly off. Measure actual battery voltage with a multimeter
//  and tweak VBATT_CAL until readout matches.
// ============================================================
#define VBATT_PIN       8

const float DIVIDER_RATIO = 0.31973f;   // R2 / (R1+R2) = 47/(100+47)
const float ADC_VREF      = 3.3f;       // ESP32 ADC reference
const float ADC_MAX       = 4095.0f;
const float VBATT_CAL     = 1.00f;      // fine-tune multiplier (measure & adjust)

const float VBATT_FULL    = 8.40f;      // 4.20V/cell x2
const float VBATT_LOW     = 6.80f;      // low battery warning threshold
const float VBATT_EMPTY   = 6.40f;      // 0% (maps to bottom of percentage scale)

// Under-load voltage sag compensation
// When motors are running, actual cell voltage is higher than measured
// due to ESR-induced sag. We add this offset back during active states.
const float VBATT_LOAD_COMP_IDLE = 0.00f;   // no compensation at idle
const float VBATT_LOAD_COMP_RUN  = 0.15f;   // +150mV compensation under load

// Rolling average — 64 samples taken every 50ms = ~3.2s window
// This smooths out motor-switching noise on the ADC rail
#define VBATT_AVG_SIZE  64
float   vbattBuf[VBATT_AVG_SIZE];
uint8_t vbattIdx      = 0;
bool    vbattFull     = false;   // true once buffer has filled once
float   vbattRaw      = 8.40f;  // last compensated reading
float   vbattAvg      = 8.40f;  // rolling average (display value)
int     battPct       = 100;
bool    battLow       = false;
bool    battLockout   = false;   // blocks start when battery is low

// Warn beep timing — periodic beep every 10s when low, non-blocking
unsigned long lastBattWarnMs  = 0;
const unsigned long BATT_WARN_INTERVAL = 10000UL;

// -------- STATE MACHINE --------
enum BotState {
  WAIT_CALIBRATE,
  WAIT_START,
  FOLLOW_LINE,
  AVOID_LEFT_TURN,
  AVOID_LEFT_FORWARD,
  AVOID_RIGHT_TURN,
  AVOID_REJOIN_LINE,
  RETRACE_REJOIN_BACK,
  RETRACE_LEFT_TURN,
  RETRACE_SIDE_BACK,
  RETRACE_RIGHT_TURN,
  ERROR_STOP
};

BotState state           = WAIT_CALIBRATE;
unsigned long stateStart        = 0;
unsigned long sideForwardDone   = 0;
unsigned long rejoinForwardDone = 0;


// ============================================================
//  MOTORS
// ============================================================
void motor1(int speed) {
  speed  = constrain(speed, -255, 255);
  lastM1 = speed;
  digitalWrite(AIN1, speed >= 0 ? LOW  : HIGH);
  digitalWrite(AIN2, speed >= 0 ? HIGH : LOW);
  ledcWrite(PWMA, abs(speed));
}

void motor2(int speed) {
  speed  = constrain(speed, -255, 255);
  lastM2 = speed;
  digitalWrite(BIN1, speed >= 0 ? HIGH : LOW);
  digitalWrite(BIN2, speed >= 0 ? LOW  : HIGH);
  ledcWrite(PWMB, abs(speed));
}

void stopMotors() {
  motor1(0);
  motor2(0);
}


// ============================================================
//  BUZZER
// ============================================================
void toneOn(int freq, int duty = 128) {
  ledcChangeFrequency(BUZZER_PIN, freq, buzzerRes);
  digitalWrite(LED_PIN, HIGH);
  ledcWrite(BUZZER_PIN, duty);
}

void toneOff() {
  ledcWrite(BUZZER_PIN, 0);
  digitalWrite(LED_PIN, LOW);
}

void beepNote(int freq, int onMs, int offMs) {
  toneOn(freq, 128);
  delay(onMs);
  toneOff();
  delay(offMs);
}

void beepCalDone()  { beepNote(1800, 100, 50); }
void beepStart()    { beepNote(1200, 90, 60); beepNote(2400, 220, 80); }
void beepError()    { beepNote(2800, 110, 60); beepNote(1200, 220, 80); }

// Non-blocking low-battery warning beep (2 short pulses)
void beepBattLowNB() {
  // Uses ledcWrite directly — no delay() — safe inside loop()
  toneOn(2200, 90);
}
void beepBattWarn() {
  beepNote(2200, 80, 60);
  beepNote(2200, 80, 60);
}



// ============================================================
//  MPU6050 — raw I2C on Wire1, no library needed
// ============================================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg); Wire1.write(val);
  Wire1.endTransmission();
}
uint8_t mpuRead8(uint8_t reg) {
  Wire1.beginTransmission(MPU_ADDR); Wire1.write(reg); Wire1.endTransmission(false);
  Wire1.requestFrom(MPU_ADDR, 1);
  return Wire1.available() ? Wire1.read() : 0;
}
void mpuRead14(int16_t* ax,int16_t* ay,int16_t* az,int16_t* gx,int16_t* gy,int16_t* gz){
  Wire1.beginTransmission(MPU_ADDR); Wire1.write(0x3B); Wire1.endTransmission(false);
  Wire1.requestFrom(MPU_ADDR,14);
  if(Wire1.available()<14) return;
  *ax=(Wire1.read()<<8)|Wire1.read();
  *ay=(Wire1.read()<<8)|Wire1.read();
  *az=(Wire1.read()<<8)|Wire1.read();
  Wire1.read(); Wire1.read();  // skip temperature
  *gx=(Wire1.read()<<8)|Wire1.read();
  *gy=(Wire1.read()<<8)|Wire1.read();
  *gz=(Wire1.read()<<8)|Wire1.read();
}

void initMPU() {
  Wire1.begin(MPU_SDA, MPU_SCL, 400000);
  delay(100);
  mpuWrite(0x6B, 0x00);   // wake up
  delay(50);
  mpuWrite(0x1B, 0x08);   // gyro  ±500 °/s  (65.5 LSB/°/s)
  mpuWrite(0x1C, 0x08);   // accel ±4g        (8192 LSB/g)
  mpuWrite(0x1A, 0x04);   // DLPF ~20Hz — kills motor vibration noise

  if (mpuRead8(0x75) == 0x68) {
    mpuReady = true;
    Serial.println("MPU6050 ready on Wire1");
    // Calibration: 200 samples flat still
    long gxS=0,gyS=0,gzS=0,axS=0,ayS=0,azS=0;
    for(int i=0;i<200;i++){
      int16_t rax,ray,raz,rgx,rgy,rgz;
      mpuRead14(&rax,&ray,&raz,&rgx,&rgy,&rgz);
      axS+=rax;ayS+=ray;azS+=raz;gxS+=rgx;gyS+=rgy;gzS+=rgz;
      delay(5);
    }
    axOff=axS/200.0f/8192.0f*9.81f;
    ayOff=ayS/200.0f/8192.0f*9.81f;
    azOff=azS/200.0f/8192.0f*9.81f-9.81f;
    gxOff=gxS/200.0f/65.5f;
    gyOff=gyS/200.0f/65.5f;
    gzOff=gzS/200.0f/65.5f;
    Serial.println("MPU calibration done");
  } else {
    Serial.println("MPU6050 NOT FOUND");
  }
}

void updateMPU() {
  if (!mpuReady) return;
  unsigned long now = millis();
  float dt = (now - lastMpuMs) / 1000.0f;
  // Run at ~100Hz max; skip if called too soon
  if (dt < 0.008f) return;
  lastMpuMs = now;

  int16_t rax,ray,raz,rgx,rgy,rgz;
  mpuRead14(&rax,&ray,&raz,&rgx,&rgy,&rgz);

  mpuAx = rax/8192.0f*9.81f - axOff;
  mpuAy = ray/8192.0f*9.81f - ayOff;
  mpuAz = raz/8192.0f*9.81f - azOff;
  mpuGx = rgx/65.5f - gxOff;
  mpuGy = rgy/65.5f - gyOff;
  mpuGz = rgz/65.5f - gzOff;

  mpuGForce = sqrtf(mpuAx*mpuAx+mpuAy*mpuAy+mpuAz*mpuAz)/9.81f;

  // Complementary filter
  float aPitch = atan2f(-mpuAx, sqrtf(mpuAy*mpuAy+mpuAz*mpuAz))*180.0f/M_PI;
  float aRoll  = atan2f( mpuAy, mpuAz)*180.0f/M_PI;
  mpuPitch = CF_ALPHA*(mpuPitch+mpuGx*dt)+(1.0f-CF_ALPHA)*aPitch;
  mpuRoll  = CF_ALPHA*(mpuRoll +mpuGy*dt)+(1.0f-CF_ALPHA)*aRoll;
  mpuYaw  += mpuGz*dt;
  if(mpuYaw> 180.0f) mpuYaw-=360.0f;
  if(mpuYaw<-180.0f) mpuYaw+=360.0f;

  // Tilt-stop: if bot has fallen or is on extreme slope, halt
  float maxTilt = max(fabsf(mpuPitch), fabsf(mpuRoll));
  mpuTiltStop = (maxTilt > TILT_STOP_DEG);

  // Impact: accel spike > 3G
  if (mpuGForce > 3.0f) { mpuImpact=true; mpuImpactMs=now; }
  else if (now-mpuImpactMs>500) mpuImpact=false;

  // Pivot angle tracking (gyro Z integral during pivot states)
  bool inPivot = (state==AVOID_LEFT_TURN || state==AVOID_RIGHT_TURN ||
                  state==RETRACE_LEFT_TURN || state==RETRACE_RIGHT_TURN);
  if (inPivot && !pivotTracking) { pivotYawAccum=0; pivotTracking=true; }
  if (!inPivot) pivotTracking=false;
  if (pivotTracking) pivotYawAccum += mpuGz*dt;

  // Dynamic Kd boost: if gyro Y shows high oscillation rate during line follow,
  // temporarily increase Kd to dampen it, then let it decay
  if (state == FOLLOW_LINE) {
    float gyroMag = fabsf(mpuGy);
    if (gyroMag > 80.0f) {
      // Significant oscillation — boost Kd proportionally
      mpuKdBoost = (gyroMag - 80.0f) * 0.008f;
      mpuKdBoost = constrain(mpuKdBoost, 0.0f, 1.5f);
    } else {
      mpuKdBoost *= 0.92f;  // decay when oscillation settles
      if (mpuKdBoost < 0.01f) mpuKdBoost = 0.0f;
    }
  } else {
    mpuKdBoost = 0.0f;
  }
}

// ============================================================
//  WEBSOCKET LOG
// ============================================================
void wsSendLog(String msg, String type) {
  StaticJsonDocument<128> doc;
  doc["log"] = msg;
  doc["lt"]  = type;
  String out;
  serializeJson(doc, out);
  wsServer.broadcastTXT(out);
  Serial.println("[LOG/" + type + "] " + msg);
}


// ============================================================
//  BATTERY — read, average, compensate, compute percentage
// ============================================================

// Returns true if bot is actively moving (load compensation needed)
bool isRunning() {
  return (state == FOLLOW_LINE      ||
          state == AVOID_LEFT_TURN  ||
          state == AVOID_LEFT_FORWARD ||
          state == AVOID_RIGHT_TURN ||
          state == AVOID_REJOIN_LINE ||
          state == RETRACE_REJOIN_BACK ||
          state == RETRACE_LEFT_TURN ||
          state == RETRACE_SIDE_BACK ||
          state == RETRACE_RIGHT_TURN);
}

void updateBattery() {
  // --- Sample: take 4 ADC readings and median-of-4 to reject spike noise ---
  // (motor PWM switching causes brief spikes on ADC; sampling multiple times
  //  and taking the middle two values gives a clean reading)
  uint32_t s[4];
  for (int i = 0; i < 4; i++) {
    s[i] = analogRead(VBATT_PIN);
    delayMicroseconds(50);
  }
  // Simple sort of 4 values
  for (int i = 0; i < 3; i++)
    for (int j = i+1; j < 4; j++)
      if (s[j] < s[i]) { uint32_t t = s[i]; s[i] = s[j]; s[j] = t; }
  // Average of middle two (reject min and max outliers)
  uint32_t adcRaw = (s[1] + s[2]) / 2;

  // --- Convert ADC to battery voltage ---
  float vGpio  = (adcRaw / ADC_MAX) * ADC_VREF;
  float vBatt  = (vGpio / DIVIDER_RATIO) * VBATT_CAL;

  // --- Load compensation: add sag offset when motors are running ---
  float comp   = isRunning() ? VBATT_LOAD_COMP_RUN : VBATT_LOAD_COMP_IDLE;
  vBatt       += comp;

  // Clamp to sane range (protects against ADC noise at edges)
  vBatt = constrain(vBatt, 5.0f, 9.0f);
  vbattRaw = vBatt;

  // --- Push into rolling average buffer ---
  vbattBuf[vbattIdx] = vBatt;
  vbattIdx = (vbattIdx + 1) % VBATT_AVG_SIZE;
  if (vbattIdx == 0) vbattFull = true;

  // Compute average over filled portion of buffer
  uint8_t count = vbattFull ? VBATT_AVG_SIZE : vbattIdx;
  if (count == 0) count = 1;
  float sum = 0;
  for (uint8_t i = 0; i < count; i++) sum += vbattBuf[i];
  vbattAvg = sum / count;

  // --- Map to percentage: 8.4V=100%, 6.4V=0% ---
  // Use the AVERAGE voltage for display (stable)
  // Use the RAW (compensated) reading for threshold decisions (responsive)
  float pct = (vbattAvg - VBATT_EMPTY) / (VBATT_FULL - VBATT_EMPTY) * 100.0f;
  battPct   = (int)constrain(pct, 0.0f, 100.0f);

  // --- Low battery detection uses raw (instantaneous compensated) value ---
  // This ensures we catch a sudden sag quickly without waiting for avg to catch up
  bool wasLow = battLow;
  battLow     = (vbattRaw <= VBATT_LOW);
  battLockout = (vbattRaw <= VBATT_LOW);

  // Log state change
  if (battLow && !wasLow) {
    wsSendLog("BATTERY LOW: " + String(vbattRaw, 2) + "V (" + String(battPct) + "%) — bot locked", "err");
    beepBattWarn();
    // Force stop if running
    if (isRunning()) {
      stopMotors();
      state = ERROR_STOP;
      wsSendLog("Bot stopped due to low battery", "err");
    }
  }
  if (!battLow && wasLow) {
    wsSendLog("Battery recovered above threshold: " + String(vbattRaw, 2) + "V", "ok");
  }

  // Periodic warning beep while battery stays low (every 10s, non-blocking)
  if (battLow && (millis() - lastBattWarnMs >= BATT_WARN_INTERVAL)) {
    lastBattWarnMs = millis();
    beepBattWarn();
  }
}


// ============================================================
//  CALIBRATE
// ============================================================
void calibrate() {
  wsSendLog("Calibration started — rotating bot over line", "info");
  digitalWrite(LED_PIN, HIGH);

  for (int i = 0; i < 5; i++) {
    minValues[i] = 4095;
    maxValues[i] = 0;
  }

  long startTime = millis();
  while (millis() - startTime < 4000) {
    motor1(80);
    motor2(-80);
    for (int i = 0; i < 5; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
    wsServer.loop();
    updateBattery();
  }

  stopMotors();
  digitalWrite(LED_PIN, LOW);
  wsSendLog("Calibration done", "ok");
}


// ============================================================
//  WEBSOCKET EVENT HANDLER
// ============================================================
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, payload, length) != DeserializationError::Ok) return;

    const char* cmd = doc["cmd"];
    if (!cmd) return;

    String c = String(cmd);
    c.trim();
    c.toUpperCase();

    if (c == "CALIBRATE") {
      if (state == FOLLOW_LINE || state == ERROR_STOP) {
        wsSendLog("Cannot calibrate while running — reset first", "warn");
      } else if (battLockout) {
        wsSendLog("Cannot calibrate — battery too low (" + String(vbattRaw, 2) + "V)", "err");
      } else {
        wsSendLog("Remote calibration triggered", "info");
        calibrate();
        beepCalDone();
        state = WAIT_START;
        wsSendLog("Calibration complete — ready to start", "ok");
      }
    }
    else if (c == "START") {
      if (battLockout) {
        wsSendLog("START BLOCKED — battery low: " + String(vbattRaw, 2) + "V / " + String(battPct) + "%", "err");
        beepBattWarn();
      } else {
        if (state == WAIT_CALIBRATE) wsSendLog("Warning: starting without calibration", "warn");
        if (state == WAIT_START || state == WAIT_CALIBRATE) {
          beepStart();
          I = 0;
          previousError = 0;
          state = FOLLOW_LINE;
          wsSendLog("Bot started via dashboard", "ok");
        } else {
          wsSendLog("Cannot start from current state: " + String((int)state), "warn");
        }
      }
    }
    else if (c == "STOP") {
      stopMotors();
      state = ERROR_STOP;
      wsSendLog("E-STOP triggered from dashboard", "err");
      beepError();
    }
    else if (c == "RESET") {
      stopMotors();
      I = 0;
      previousError = 0;
      state = WAIT_CALIBRATE;
      wsSendLog("Bot reset — waiting for calibration", "info");
    }
    else {
      wsSendLog("Unknown command received: [" + c + "]", "warn");
    }
  }
  else if (type == WStype_CONNECTED) {
    wsSendLog("Dashboard connected", "ok");
    wsSendLog("Battery: " + String(vbattAvg, 2) + "V / " + String(battPct) + "%", battLow ? "err" : "ok");
  }
  else if (type == WStype_DISCONNECTED) {
    Serial.println("Dashboard disconnected");
  }
}


// ============================================================
//  TELEMETRY — battery fields added
// ============================================================
void sendTelemetry() {
  StaticJsonDocument<448> doc;
  JsonArray s = doc.createNestedArray("s");
  for (int i = 0; i < 5; i++) s.add(sensorValue[i]);
  doc["t"]    = lastDistance;
  doc["st"]   = (int)state;
  doc["ol"]   = onLine;
  doc["p"]    = (int)P;
  doc["i"]    = (int)(I * 100);
  doc["dd"]   = (int)D;
  doc["err"]  = previousError;
  doc["m1"]   = lastM1;
  doc["m2"]   = lastM2;
  // Battery fields
  doc["vr"]   = (int)(vbattRaw * 100);   // raw voltage x100 (e.g. 742 = 7.42V)
  doc["va"]   = (int)(vbattAvg * 100);   // avg voltage x100
  doc["bp"]   = battPct;                 // 0-100
  doc["bl"]   = battLow;                 // bool
  doc["blk"]  = battLockout;             // bool
  // MPU fields (x10 integers to save JSON bytes)
  doc["mpu"]  = mpuReady;
  doc["pit"]  = (int)(mpuPitch*10);      // pitch deg x10
  doc["rol"]  = (int)(mpuRoll*10);       // roll  deg x10
  doc["yaw"]  = (int)(mpuYaw*10);        // yaw   deg x10
  doc["gf"]   = (int)(mpuGForce*100);    // G-force x100
  doc["tilt"] = mpuTiltStop;
  doc["imp"]  = mpuImpact;
  doc["kdb"]  = (int)(mpuKdBoost*100);   // dynamic Kd boost x100
  String out;
  serializeJson(doc, out);
  wsServer.broadcastTXT(out);
}


// ============================================================
//  TOF
// ============================================================
void initTOF() {
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(100);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(100);

  if (tof.begin()) {
    tof.startRanging();
    tofReady = true;
    Serial.println("ToF OK");
    wsSendLog("VL53L1X ToF sensor ready", "ok");
  } else {
    tofReady = false;
    Serial.println("ToF NOT FOUND");
    wsSendLog("VL53L1X not found — obstacle detection disabled", "err");
  }
}

int readTOFmm() {
  if (!tofReady) return 9999;
  if (tof.dataReady()) {
    int d = tof.distance();
    if (d > 0) lastDistance = d;
    tof.clearInterrupt();
  }
  return lastDistance;
}

bool obstacleDetected() {
  int d = readTOFmm();
  return (d > 0 && d < OBSTACLE_MM);
}


// ============================================================
//  SENSORS & PID
// ============================================================
void readSensors() {
  onLine = false;
  for (int i = 0; i < 5; i++) {
    int raw = analogRead(sensorPins[i]);
    sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    if (sensorValue[i] > 600) onLine = true;
  }
}

void calculatePID() {
  int error = (-20 * sensorValue[0] - 10 * sensorValue[1]
               + 10 * sensorValue[3] + 20 * sensorValue[4]) / 10;
  P  = error;
  I += error;
  I  = constrain(I, -100, 100);
  D  = error - previousError;
  // mpuKdBoost is added when gyro detects oscillation — auto-damps the bot
  float effectiveKd = Kd + mpuKdBoost;
  int pidValue   = (Kp * P) + (Ki * I) + (effectiveKd * D);
  previousError  = error;
  int leftSpeed  = constrain(baseSpeed + pidValue, -maxSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed - pidValue, -maxSpeed, maxSpeed);
  motor1(leftSpeed);
  motor2(rightSpeed);
}


// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Configure ADC attenuation on battery pin for 0–3.3V range
  analogSetPinAttenuation(VBATT_PIN, ADC_11db);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(CAL_BUTTON,   INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN,      OUTPUT);
  pinMode(BUZZER_PIN,   OUTPUT);
  pinMode(VBATT_PIN,    INPUT);

  digitalWrite(STBY, HIGH);
  digitalWrite(LED_PIN, LOW);

  ledcAttach(PWMA,       20000, 8);
  ledcAttach(PWMB,       20000, 8);
  ledcAttach(BUZZER_PIN,  2000, buzzerRes);
  ledcWrite(BUZZER_PIN, 0);

  // Pre-fill battery buffer with 16 startup readings for a stable initial value
  for (int i = 0; i < 16; i++) {
    uint32_t raw = analogRead(VBATT_PIN);
    float vg     = (raw / ADC_MAX) * ADC_VREF;
    float vb     = (vg / DIVIDER_RATIO) * VBATT_CAL;
    vb           = constrain(vb, 5.0f, 9.0f);
    vbattBuf[i]  = vb;
    delay(5);
  }
  vbattIdx  = 16;
  vbattFull = false;
  float sum = 0;
  for (int i = 0; i < 16; i++) sum += vbattBuf[i];
  vbattAvg = sum / 16.0f;
  vbattRaw = vbattAvg;
  float pct = (vbattAvg - VBATT_EMPTY) / (VBATT_FULL - VBATT_EMPTY) * 100.0f;
  battPct   = (int)constrain(pct, 0.0f, 100.0f);
  battLow   = (vbattRaw <= VBATT_LOW);
  battLockout = battLow;

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  wsServer.begin();
  wsServer.onEvent(onWsEvent);
  Serial.println("WebSocket server started on port 81");
  Serial.print("Battery at boot: "); Serial.print(vbattAvg); Serial.print("V / "); Serial.print(battPct); Serial.println("%");
  if (battLow) Serial.println("WARNING: Battery low at boot!");

  initMPU();
  lastMpuMs = millis();
  if (mpuReady) wsSendLog("MPU6050 active — orientation & PID feedback live", "ok");
  else          wsSendLog("MPU6050 not found — running without IMU", "warn");

  initTOF();

  state = WAIT_CALIBRATE;
}


// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  wsServer.loop();
  readSensors();
  updateMPU();      // 100Hz IMU update — tilt-stop, dynamic Kd, pivot tracking
  updateBattery();   // runs every loop iteration — samples ADC, updates avg

  switch (state) {

    case WAIT_CALIBRATE:
      if (!digitalRead(CAL_BUTTON)) {
        delay(500);
        if (battLockout) {
          wsSendLog("CAL BLOCKED — battery low: " + String(vbattRaw, 2) + "V", "err");
          beepBattWarn();
          break;
        }
        calibrate();
        beepCalDone();
        state = WAIT_START;
        wsSendLog("Ready — press Start or click Start on dashboard", "ok");
      }
      break;

    case WAIT_START:
      if (!digitalRead(START_BUTTON)) {
        delay(500);
        if (battLockout) {
          wsSendLog("START BLOCKED — battery low: " + String(vbattRaw, 2) + "V / " + String(battPct) + "%", "err");
          beepBattWarn();
          break;
        }
        beepStart();
        I = 0;
        previousError = 0;
        state = FOLLOW_LINE;
        wsSendLog("Bot started via hardware button", "ok");
      }
      break;

    case FOLLOW_LINE:
      // MPU tilt-stop: if bot has fallen or tilted beyond safe angle, halt immediately
      if (mpuTiltStop) {
        stopMotors();
        state = ERROR_STOP;
        wsSendLog("TILT STOP: bot tilted " + String(max(fabsf(mpuPitch),fabsf(mpuRoll)),1) + "deg", "err");
        beepError();
        break;
      }
      if (obstacleDetected()) {
        stopMotors();
        delay(80);
        wsSendLog("Obstacle at " + String(lastDistance) + " mm — starting avoidance", "warn");
        state      = AVOID_LEFT_TURN;
        stateStart = millis();
        sideForwardDone   = 0;
        rejoinForwardDone = 0;
        break;
      }
      if (sensorValue[0] > 750 && sensorValue[2] < 300) {
        while (analogRead(sensorPins[2]) < (minValues[2] + 600)) {
          motor1(-pivotSpeed); motor2(pivotSpeed);
        }
      }
      else if (sensorValue[4] > 750 && sensorValue[2] < 300) {
        while (analogRead(sensorPins[2]) < (minValues[2] + 600)) {
          motor1(pivotSpeed); motor2(-pivotSpeed);
        }
      }
      else if (onLine) { calculatePID(); }
      else {
        if (previousError < 0) { motor1(-100); motor2( 100); }
        else                   { motor1( 100); motor2(-100); }
      }
      break;

    case AVOID_LEFT_TURN:
      motor1(-pivotSpeed); motor2(pivotSpeed);
      if (millis() - stateStart >= TURN_90_MS) { state = AVOID_LEFT_FORWARD; stateStart = millis(); }
      break;

    case AVOID_LEFT_FORWARD:
      motor1(avoidSpeed); motor2(avoidSpeed);
      sideForwardDone = millis() - stateStart;
      if (obstacleDetected()) { wsSendLog("Second obstacle on side path — retracing", "warn"); state = RETRACE_SIDE_BACK; stateStart = millis(); break; }
      if (sideForwardDone >= BYPASS_FORWARD_MS) { state = AVOID_RIGHT_TURN; stateStart = millis(); }
      break;

    case AVOID_RIGHT_TURN:
      motor1(pivotSpeed); motor2(-pivotSpeed);
      if (millis() - stateStart >= TURN_90_MS) { state = AVOID_REJOIN_LINE; stateStart = millis(); }
      break;

    case AVOID_REJOIN_LINE:
      motor1(recoverSpeed); motor2(recoverSpeed);
      rejoinForwardDone = millis() - stateStart;
      if (obstacleDetected()) { wsSendLog("Obstacle during rejoin — retracing", "warn"); state = RETRACE_REJOIN_BACK; stateStart = millis(); break; }
      if (sensorValue[2] > 650 || onLine) { wsSendLog("Line rejoined — resuming PID", "ok"); I = 0; previousError = 0; state = FOLLOW_LINE; }
      else if (rejoinForwardDone >= REJOIN_TIMEOUT_MS) { wsSendLog("ERROR: Line rejoin timeout", "err"); state = ERROR_STOP; stopMotors(); beepError(); }
      break;

    case RETRACE_REJOIN_BACK:
      motor1(-recoverSpeed); motor2(-recoverSpeed);
      if (millis() - stateStart >= rejoinForwardDone) { state = RETRACE_LEFT_TURN; stateStart = millis(); }
      break;

    case RETRACE_LEFT_TURN:
      motor1(-pivotSpeed); motor2(pivotSpeed);
      if (millis() - stateStart >= TURN_90_MS) { state = RETRACE_SIDE_BACK; stateStart = millis(); }
      break;

    case RETRACE_SIDE_BACK:
      motor1(-avoidSpeed); motor2(-avoidSpeed);
      if (millis() - stateStart >= sideForwardDone) { state = RETRACE_RIGHT_TURN; stateStart = millis(); }
      break;

    case RETRACE_RIGHT_TURN:
      motor1(pivotSpeed); motor2(-pivotSpeed);
      if (millis() - stateStart >= TURN_90_MS) { wsSendLog("ERROR: Avoidance retrace failed — stopped", "err"); state = ERROR_STOP; stopMotors(); beepError(); }
      break;

    case ERROR_STOP:
      stopMotors();
      break;
  }

  // Telemetry at fixed interval
  unsigned long now = millis();
  if (now - lastTelemTime >= TELEM_INTERVAL_MS) {
    lastTelemTime = now;
    sendTelemetry();
  }

  // Serial debug
  Serial.print("State:"); Serial.print(state);
  Serial.print("  Vbatt:"); Serial.print(vbattAvg, 2);
  Serial.print("V  Pct:"); Serial.print(battPct);
  Serial.print("%  TOF:"); Serial.print(readTOFmm());
  Serial.print("  S:");
  for (int i = 0; i < 5; i++) { Serial.print(sensorValue[i]); Serial.print(" "); }
  if (mpuReady) {
    Serial.print("  P:"); Serial.print(mpuPitch,1);
    Serial.print(" R:"); Serial.print(mpuRoll,1);
    Serial.print(" Y:"); Serial.print(mpuYaw,1);
    Serial.print(" G:"); Serial.print(mpuGForce,2);
    if (mpuKdBoost>0.01f) { Serial.print(" KdB:"); Serial.print(mpuKdBoost,2); }
  }
  Serial.println();

  delay(2);
}
