#include <Arduino.h>
#include <math.h>
#include "sbus.h"
#include <ModbusMaster.h>

/* =========================
   SBUS (Bolder Flight)
   ========================= */
bfs::SbusRx sbus_rx(&Serial1);
bfs::SbusData sbus_data;

/* =========================
   Pin Definitions
   ========================= */
#define DE_RE_PIN       42

#define POLISHER_S1     46
#define POLISHER_S2     48

#define CYLINDER_FWD_PIN  31
#define CYLINDER_BWD_PIN  33

#define EMERGENCY_PIN   35

#define M_RIGHT_FWD     11
#define M_RIGHT_REV     12
#define STOP_MODE_R     24
#define M0_R            26
#define MB_FREE_R       28
#define MM_R_PIN        13

#define LIMIT_RIGHT     20   // Limit_A
#define LIMIT_LEFT      21   // Limit_B

/* =========================
   Limit Switch Settings
   =========================
   Assumption: INPUT_PULLUP wiring
   - Switch pressed   -> LOW
   - Switch released  -> HIGH
*/
static const bool LIMIT_ACTIVE_LOW = true;

/* =========================
   Calibration / Trim
   ========================= */
float left_motor_trim  = 0.95f;
float right_motor_trim = 1.0f;

/* =========================
   SBUS State
   ========================= */
float percent[16];              // Channel values mapped to -100..+100
bool sbusFrameLost = false;
bool sbusFailsafe  = false;

unsigned long lastSignalTime = 0;
const unsigned long SIGNAL_TIMEOUT = 150;  // ms

/* =========================
   BLV Modbus (RS485)
   ========================= */
#define MB_SERIAL  Serial2
#define MB_BAUD    230400
#define SLAVE_1    0x01
#define SLAVE_2    0x02

ModbusMaster mb1;
ModbusMaster mb2;

/* =========================
   Drive Parameters
   ========================= */
int max_speed         = 3000;
int deadband_percent  = 5;
int max_speed_rotary  = 100;

bool monitorMode = false;

const float TH_DB = 8.0f;  // Throttle deadband
const float ST_DB = 8.0f;  // Steering deadband

const int32_t ACC_NORM  = 50;
const int32_t DEC_RUN   = 50;

const int32_t ACC_BRAKE = 10;
const int32_t DEC_BRAKE = 10;

bool wasNeutral = false;

static bool son1 = false;
static bool son2 = false;

/* =========================
   Utility Helpers
   ========================= */
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* =========================
   Limit Switch Helpers
   ========================= */
static inline bool limitPressedRaw(int pin) {
  int v = digitalRead(pin);
  return LIMIT_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

static inline bool limitRightPressed() { return limitPressedRaw(LIMIT_RIGHT); }
static inline bool limitLeftPressed()  { return limitPressedRaw(LIMIT_LEFT);  }

/* =========================
   RS485 Direction Control (ModbusMaster callbacks)
   ========================= */
void preTransmission() {
  digitalWrite(DE_RE_PIN, HIGH);  // Enable TX
  delayMicroseconds(120);
}

void postTransmission() {
  MB_SERIAL.flush();              // Ensure all bytes are sent
  delayMicroseconds(120);
  digitalWrite(DE_RE_PIN, LOW);   // Enable RX
  delayMicroseconds(80);
}

/* Clear pending RX bytes (noise/leftovers) before a Modbus transaction */
static inline void mbClearRx() {
  while (MB_SERIAL.available()) MB_SERIAL.read();
}

/* =========================
   BLV Modbus Command Helpers
   ========================= */
static inline ModbusMaster &nodeFor(uint8_t slaveId) {
  return (slaveId == SLAVE_1) ? mb1 : mb2;
}

/* Returns the S-ON bit state for each slave (bit0) */
static inline uint16_t sonBit(uint8_t id) {
  if (id == SLAVE_1) return son1 ? 0x0001 : 0x0000;
  return son2 ? 0x0001 : 0x0000;
}

/* Put int32 into two 16-bit registers: [HI_WORD, LO_WORD] */
static inline void setTx32(ModbusMaster &node, uint8_t idx, int32_t v) {
  uint32_t u  = (uint32_t)v;
  uint16_t hi = (uint16_t)(u >> 16); // (16-32)
  uint16_t lo = (uint16_t)(u & 0xFFFF); // (0-15)
  node.setTransmitBuffer(idx, hi);
  node.setTransmitBuffer(idx + 1, lo);
}

/* Optional Modbus error reporter */
static inline void mbReportIfError(const char *tag, uint8_t slaveId, uint8_t res) {
  if (res == 0) return;  // 0 = success
  Serial.print("[MB] ");
  Serial.print(tag);
  Serial.print(" slave=");
  Serial.print(slaveId);
  Serial.print(" err=0x");
  Serial.println(res, HEX);
}

/* =========================
   BLV Commands (write-only style)
   =========================
   - Command2: write 2 registers starting at 0x007C (upper, lower)
   - Direct Data Operation: write 14 registers starting at 0x005A
*/
uint8_t write_cmd2_lower(uint8_t slaveId, uint16_t lower) {
  ModbusMaster &node = nodeFor(slaveId);

  mbClearRx();
  node.clearTransmitBuffer();
  node.setTransmitBuffer(0, 0x0000);  // Command2 upper word
  node.setTransmitBuffer(1, lower);   // Command2 lower word

  uint8_t res = node.writeMultipleRegisters(0x007C, 2);
  mbReportIfError("CMD2", slaveId, res);
  return res;
}

void set_son(uint8_t slaveId, bool on) {
  if (slaveId == SLAVE_1) son1 = on;
  if (slaveId == SLAVE_2) son2 = on;

  uint16_t lower = on ? 0x0001 : 0x0000;  // bit0 = S-ON
  write_cmd2_lower(slaveId, lower);
}

void stop_pulse(uint8_t slaveId) {
  const uint16_t STOP_BIT = (1u << 5);     // bit5 = STOP
  uint16_t keep = sonBit(slaveId);         // keep current S-ON bit

  write_cmd2_lower(slaveId, (uint16_t)(keep | STOP_BIT));
  delay(5);
  write_cmd2_lower(slaveId, keep);
}

void direct_velocity(uint8_t slaveId, int32_t rpm, int32_t acc, int32_t dec) {
  ModbusMaster &node = nodeFor(slaveId);

  mbClearRx();
  node.clearTransmitBuffer();

  // 14 registers = 7 x int32 (each uses 2 registers)
  setTx32(node,  0, 0x00000030);
  setTx32(node,  2, 0);
  setTx32(node,  4, rpm);
  setTx32(node,  6, acc);
  setTx32(node,  8, dec);
  setTx32(node, 10, 1000);
  setTx32(node, 12, 1);

  uint8_t res = node.writeMultipleRegisters(0x005A, 14);
  mbReportIfError("VEL", slaveId, res);
}

void brake_blv_both_now() {
  stop_pulse(SLAVE_1);
  delay(5);
  stop_pulse(SLAVE_2);
  delay(5);

  direct_velocity(SLAVE_1, 0, ACC_BRAKE, DEC_BRAKE);
  delay(5);
  direct_velocity(SLAVE_2, 0, ACC_BRAKE, DEC_BRAKE);
  delay(5);
}

void hold_zero_blv() {
  direct_velocity(SLAVE_1, 0, ACC_BRAKE, DEC_BRAKE);
  delay(5);
  direct_velocity(SLAVE_2, 0, ACC_BRAKE, DEC_BRAKE);
  delay(5);
}

/* =========================
   SBUS Conversion
   ========================= */
float sbusToPercent(int value, int minPulse = 172, int midPulse = 992, int maxPulse = 1811) {
  float p = 0.0f;

  if (value == midPulse) {
    p = 0.0f;
  } else if (value > midPulse) {
    p = ((float)(value - midPulse) / (float)(maxPulse - midPulse)) * 100.0f;
  } else {
    p = -((float)(midPulse - value) / (float)(midPulse - minPulse)) * 100.0f;
  }
  return clampf(p, -100.0f, 100.0f);
}

void updateSbusFromLibrary() {
  if (!sbus_rx.Read()) return;

  sbus_data = sbus_rx.data();

  lastSignalTime = millis();
  sbusFrameLost  = sbus_data.lost_frame;
  sbusFailsafe   = sbus_data.failsafe;

  for (int i = 0; i < 16; i++) {
    percent[i] = sbusToPercent(sbus_data.ch[i]);
  }
}

void debugLimitSwitches() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 200) return;
  lastPrint = millis();

  Serial.print("LIMIT L:");
  Serial.print(limitLeftPressed() ? "PRESSED" : "OPEN");
  Serial.print(" R:");
  Serial.println(limitRightPressed() ? "PRESSED" : "OPEN");
}

/* =========================
   Local Outputs: Rotary / Linear / Polisher
   ========================= */
void move_right(int speed) {
  digitalWrite(M_RIGHT_FWD, HIGH);
  digitalWrite(M_RIGHT_REV, LOW);
  analogWrite(MM_R_PIN, speed);
}

void move_left(int speed) {
  digitalWrite(M_RIGHT_FWD, LOW);
  digitalWrite(M_RIGHT_REV, HIGH);
  analogWrite(MM_R_PIN, speed);
}

void stop_rotary() {
  digitalWrite(M_RIGHT_FWD, LOW);
  digitalWrite(M_RIGHT_REV, LOW);
  analogWrite(MM_R_PIN, 0);
}

void linear_actuator_controller() {
  bool limFwd = limitRightPressed();
  bool limBwd = limitLeftPressed();

  if (percent[2] < -30) {
    if (limFwd) {
      digitalWrite(CYLINDER_FWD_PIN, LOW);
      digitalWrite(CYLINDER_BWD_PIN, LOW);
    } else {
      digitalWrite(CYLINDER_FWD_PIN, HIGH);
      digitalWrite(CYLINDER_BWD_PIN, LOW);
    }
  } else if (percent[2] > 30) {
    if (limBwd) {
      digitalWrite(CYLINDER_FWD_PIN, LOW);
      digitalWrite(CYLINDER_BWD_PIN, LOW);
    } else {
      digitalWrite(CYLINDER_BWD_PIN, HIGH);
      digitalWrite(CYLINDER_FWD_PIN, LOW);
    }
  } else {
    digitalWrite(CYLINDER_FWD_PIN, LOW);
    digitalWrite(CYLINDER_BWD_PIN, LOW);
  }
}

void polish_start() {
  digitalWrite(POLISHER_S1, HIGH);
  digitalWrite(POLISHER_S2, HIGH);
}

void polish_stop() {
  digitalWrite(POLISHER_S1, LOW);
  digitalWrite(POLISHER_S2, LOW);
  delay(2);
}

void polisherControl() {
  if (percent[4] > 52) polish_start();
  else if (percent[4] < -50) polish_stop();
}

/* =========================
   Drive Mixing (BLV)
   ========================= */
int32_t rpmFromPercent(float p) {
  if (fabs(p) < deadband_percent) return 0;
  p = clampf(p, -100.0f, 100.0f);
  return (int32_t)(p / 100.0f * max_speed);
}

void drive_blv_from_rc() {
  float throttle = percent[1];
  float steer    = -percent[3];

  // Neutral detection (inside deadband window)
  bool neutralNow = (fabs(throttle) < TH_DB && fabs(steer) < ST_DB);
  if (neutralNow) {
    if (!wasNeutral) {
      brake_blv_both_now();
      wasNeutral = true;
    } else {
      hold_zero_blv();
    }
    return;
  }
  wasNeutral = false;

  // Skip very small changes to reduce Modbus traffic
  static float last_throttle = 0;
  static float last_steer = 0;
  const float MOVEMENT_THRESHOLD = 1.5f;

  if (fabs(throttle - last_throttle) < MOVEMENT_THRESHOLD &&
      fabs(steer - last_steer) < MOVEMENT_THRESHOLD) {
    return;
  }
  last_throttle = throttle;
  last_steer = steer;

  // Spin in place (throttle near zero, steer active)
  if (fabs(throttle) < TH_DB && fabs(steer) > ST_DB) {
    float left  = steer;
    float right = -steer;

    left  *= left_motor_trim;
    right *= right_motor_trim;

    left  = clampf(left, -100.0f, 100.0f);
    right = clampf(right, -100.0f, 100.0f);

    int32_t leftRpm  = rpmFromPercent(left);
    int32_t rightRpm = rpmFromPercent(right);

    leftRpm = -leftRpm;  // invert left motor direction

    direct_velocity(SLAVE_1, leftRpm, ACC_NORM, DEC_RUN);
    delay(3);
    direct_velocity(SLAVE_2, rightRpm, ACC_NORM, DEC_RUN);
    return;
  }

  // Reverse steering when moving backward
  if (throttle < 0) steer = -steer;

  float left  = throttle - steer;
  float right = throttle + steer;

  left  *= left_motor_trim;
  right *= right_motor_trim;

  left  = clampf(left, -100.0f, 100.0f);
  right = clampf(right, -100.0f, 100.0f);

  int32_t leftRpm  = rpmFromPercent(left);
  int32_t rightRpm = rpmFromPercent(right);

  leftRpm = -leftRpm;  // invert left motor direction

  direct_velocity(SLAVE_1, leftRpm, ACC_NORM, DEC_RUN);
  delay(3);
  direct_velocity(SLAVE_2, rightRpm, ACC_NORM, DEC_RUN);
}

/* =========================
   Safety / Serial Commands
   ========================= */
void stop_all_outputs_safe() {
  brake_blv_both_now();
  polish_stop();
  digitalWrite(CYLINDER_FWD_PIN, LOW);
  digitalWrite(CYLINDER_BWD_PIN, LOW);
  stop_rotary();
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == 'm' || c == 'M') {
      monitorMode = true;
      stop_all_outputs_safe();
      wasNeutral = false;
    } else if (c == 'r' || c == 'R') {
      monitorMode = false;
      wasNeutral = false;
    } else if (c == 's' || c == 'S') {
      stop_all_outputs_safe();
      wasNeutral = false;
    }
  }
}

void emergencyChecker() {
  static bool emergencyState = false;

  if (percent[5] > 60.0f) emergencyState = true;
  else if (percent[5] < 40.0f) emergencyState = false;

  // Active-low emergency output (LOW = active)
  digitalWrite(EMERGENCY_PIN, emergencyState ? LOW : HIGH);
}

/* =========================
   Setup / Loop
   ========================= */
void setup() {
  Serial.begin(115200);
  delay(300);

  // Modbus RTU settings: 230400, 8E1
  MB_SERIAL.begin(MB_BAUD, SERIAL_8E1);

  // Start SBUS receiver
  sbus_rx.Begin();

  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);  // Start in RX mode

  pinMode(CYLINDER_FWD_PIN, OUTPUT);
  pinMode(CYLINDER_BWD_PIN, OUTPUT);
  pinMode(POLISHER_S1, OUTPUT);
  pinMode(POLISHER_S2, OUTPUT);

  pinMode(M_RIGHT_FWD, OUTPUT);
  pinMode(M_RIGHT_REV, OUTPUT);
  pinMode(STOP_MODE_R, OUTPUT);
  pinMode(M0_R, OUTPUT);
  pinMode(MB_FREE_R, OUTPUT);
  pinMode(MM_R_PIN, OUTPUT);

  pinMode(EMERGENCY_PIN, OUTPUT);

  pinMode(LIMIT_RIGHT, INPUT_PULLUP);
  pinMode(LIMIT_LEFT,  INPUT_PULLUP);

  // ModbusMaster initialization (two slaves on the same bus)
  mb1.begin(SLAVE_1, MB_SERIAL);
  mb2.begin(SLAVE_2, MB_SERIAL);

  mb1.preTransmission(preTransmission);
  mb1.postTransmission(postTransmission);
  mb2.preTransmission(preTransmission);
  mb2.postTransmission(postTransmission);

  // Initial outputs
  digitalWrite(M_RIGHT_FWD, LOW);
  digitalWrite(M_RIGHT_REV, LOW);
  digitalWrite(STOP_MODE_R, LOW);
  digitalWrite(M0_R, HIGH);
  digitalWrite(MB_FREE_R, HIGH);
  digitalWrite(MM_R_PIN, LOW);

  digitalWrite(CYLINDER_FWD_PIN, LOW);
  digitalWrite(CYLINDER_BWD_PIN, LOW);
  polish_stop();

  // Enable BLV S-ON then brake/hold at zero
  set_son(SLAVE_1, true);
  delay(200);
  set_son(SLAVE_2, true);
  delay(300);

  brake_blv_both_now();
}

void loop() {
  updateSbusFromLibrary();
  debugLimitSwitches();
  handleSerialCommands();

  // Safety: SBUS timeout or failsafe => stop everything
  if (millis() - lastSignalTime > SIGNAL_TIMEOUT || sbusFailsafe) {
    stop_all_outputs_safe();
    digitalWrite(EMERGENCY_PIN, LOW);
    delay(50);
    return;
  }

  emergencyChecker();

  // Monitor mode: outputs forced to safe state
  if (monitorMode) {
    stop_all_outputs_safe();
    delay(20);
    return;
  }

  linear_actuator_controller();
  polisherControl();

  // Drive update period
  static unsigned long lastDrive = 0;
  const unsigned long DRIVE_PERIOD_MS = 40;

  if (millis() - lastDrive >= DRIVE_PERIOD_MS) {
    lastDrive = millis();
    drive_blv_from_rc();
  }
}
