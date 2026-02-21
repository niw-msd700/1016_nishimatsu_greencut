#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "sbus.h"
#include <stdlib.h>

// =====================================================
// =================== MESSAGE STRUCT ===================
// =====================================================
struct CmdVel {
  float vx;
  float wz;
  bool valid;
};

struct AuxCmd {
  float rotary;    // -100..100
  float lin;       // -100..100
  float polisher;  // -100..100 (use thresholds like RC)
  float emergency; // -100..100 (use hysteresis like RC)
  bool valid;
};

// =====================================================
// =================== Pin Definition ===================
// =====================================================
#define DE_RE_PIN 42
#define polisher_S1 46
#define polisher_S2 48
#define linact_fwd_pin 36
#define linact_bwd_pin 39
#define emergency_pin 35

#define mRightFWD 30
#define mRightREV 31
#define stopModeR 32
#define m0R 33
#define mbFreeR 34
#define mmRPin 4

#define limit_right 20
#define limit_left 21

// =====================================================
// =================== Drive Tuning =====================
// =====================================================
float left_motor_trim = 0.35f;
float right_motor_trim = 0.40f;

int max_speed = 5300;
int deadband_percent = 5;

const float TH_DB = 8.0f;
const float ST_DB = 8.0f;

const int32_t ACC_NORM = 50;
const int32_t DEC_RUN = 50;

const int32_t ACC_BRAKE = 10;
const int32_t DEC_BRAKE = 10;

bool wasNeutral = false;

float MAX_VX_MPS = 0.6f;
float MAX_WZ_RPS = 2.0f;

const bool RC_FORWARD_IS_NEGATIVE = false;
const bool RC_RIGHT_IS_POSITIVE = true;
const bool INVERT_STEER_WHEN_REVERSE = true;

const bool INVERT_LEFT_MOTOR = true;
const bool INVERT_RIGHT_MOTOR = false;

const bool INVERT_WZ_WHEN_SPIN_ONLY = true;

const bool SERIAL_VX_INVERT = true;
const bool SERIAL_WZ_INVERT = true;

// =====================================================
// =================== Mode Selection ===================
// =====================================================
static constexpr int MODE_CH_INDEX = 7;   // CH8 (0-based)
static constexpr int SBUS_MID_PULSE = 992;

// =====================================================
// =================== Serial State =====================
// =====================================================
CmdVel lastSerialCmd = {0.0f, 0.0f, false};
unsigned long lastSerialCmdMs = 0;
const unsigned long SERIAL_CMD_TIMEOUT_MS = 300;

AuxCmd lastAuxCmd = {0.0f, 0.0f, -100.0f, -100.0f, false}; // default polisher OFF, emergency OFF-input
unsigned long lastAuxCmdMs = 0;
const unsigned long AUX_CMD_TIMEOUT_MS = 300;

// =====================================================
// =================== SBUS (Library) ===================
// =====================================================
static constexpr bool SBUS_INVERTED = false;
bfs::SbusRx sbus_rx(&Serial1, SBUS_INVERTED);
bfs::SbusData sbus_data;

int sbusChannels[16];
bool sbusFrameLost = false;
bool sbusFailsafe = false;
float percent[16];

// =====================================================
// ================= BLV Modbus (TX-only) ===============
// =====================================================
#define MB_SERIAL Serial2
#define MB_BAUD 230400
#define SLAVE_1 0x01
#define SLAVE_2 0x02

static bool son1 = false;
static bool son2 = false;

// =====================================================
// ======================= Helpers ======================
// =====================================================
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float applyDeadband(float v, float db) {
  return (fabs(v) < db) ? 0.0f : v;
}

static inline float clampAbs(float v, float maxAbs) {
  if (v > maxAbs) return maxAbs;
  if (v < -maxAbs) return -maxAbs;
  return v;
}

// =====================================================
// =================== Modbus CRC16 =====================
// =====================================================
uint16_t crc16_modbus(const uint8_t *data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

static inline void rs485_tx() {
  digitalWrite(DE_RE_PIN, HIGH);
  delayMicroseconds(150);
}

static inline void rs485_rx() {
  MB_SERIAL.flush();
  delayMicroseconds(150);
  digitalWrite(DE_RE_PIN, LOW);
  delayMicroseconds(100);
}

static inline void put_i32_be(uint8_t *p, int32_t v) {
  p[0] = (uint8_t)((v >> 24) & 0xFF);
  p[1] = (uint8_t)((v >> 16) & 0xFF);
  p[2] = (uint8_t)((v >> 8) & 0xFF);
  p[3] = (uint8_t)(v & 0xFF);
}

void modbus_send(const uint8_t *pdu, int lenNoCrc) {
  uint8_t buf[128];
  if (lenNoCrc + 2 > (int)sizeof(buf)) return;

  memcpy(buf, pdu, lenNoCrc);
  uint16_t crc = crc16_modbus(buf, lenNoCrc);
  buf[lenNoCrc] = crc & 0xFF;
  buf[lenNoCrc + 1] = (crc >> 8) & 0xFF;

  delayMicroseconds(400);
  while (MB_SERIAL.available()) MB_SERIAL.read();

  rs485_tx();
  MB_SERIAL.write(buf, lenNoCrc + 2);
  MB_SERIAL.flush();
  rs485_rx();

  delayMicroseconds(300);
  while (MB_SERIAL.available()) MB_SERIAL.read();

  delay(4);
}

// =====================================================
// ===================== BLV Commands ===================
// =====================================================
static inline uint16_t sonBit(uint8_t id) {
  if (id == SLAVE_1) return son1 ? 0x0001 : 0x0000;
  return son2 ? 0x0001 : 0x0000;
}

void write_cmd2_lower(uint8_t slaveId, uint16_t lower) {
  uint8_t f[] = {
    slaveId, 0x10, 0x00, 0x7C, 0x00, 0x02,
    0x04,
    0x00, 0x00,
    (uint8_t)((lower >> 8) & 0xFF),
    (uint8_t)(lower & 0xFF)
  };
  modbus_send(f, sizeof(f));
}

void set_son(uint8_t slaveId, bool on) {
  if (slaveId == SLAVE_1) son1 = on;
  if (slaveId == SLAVE_2) son2 = on;

  uint16_t lower = on ? 0x0001 : 0x0000;
  write_cmd2_lower(slaveId, lower);
}

void stop_pulse(uint8_t slaveId) {
  const uint16_t STOP_BIT = (1u << 5);
  uint16_t keep = sonBit(slaveId);
  write_cmd2_lower(slaveId, keep | STOP_BIT);
  delay(5);
  write_cmd2_lower(slaveId, keep);
}

void direct_velocity(uint8_t slaveId, int32_t rpm, int32_t acc, int32_t dec) {
  uint8_t f[7 + 28];
  int i = 0;

  f[i++] = slaveId;
  f[i++] = 0x10;
  f[i++] = 0x00;
  f[i++] = 0x5A;
  f[i++] = 0x00;
  f[i++] = 0x0E;
  f[i++] = 0x1C;

  uint8_t *d = &f[i];
  put_i32_be(d + 0, 0x00000030);
  put_i32_be(d + 4, 0);
  put_i32_be(d + 8, rpm);
  put_i32_be(d + 12, acc);
  put_i32_be(d + 16, dec);
  put_i32_be(d + 20, 1000);
  put_i32_be(d + 24, 1);

  modbus_send(f, sizeof(f));
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

// =====================================================
// =================== SBUS -> Percent ==================
// =====================================================
float sbusToPercent(int value, int minPulse = 172, int midPulse = 992, int maxPulse = 1811) {
  float p;
  if (value == midPulse) p = 0.0f;
  else if (value > midPulse) p = ((float)(value - midPulse) / (float)(maxPulse - midPulse)) * 100.0f;
  else p = -((float)(midPulse - value) / (float)(midPulse - minPulse)) * 100.0f;
  return clampf(p, -100.0f, 100.0f);
}

void readAllChannels(float outputPercent[16]) {
  for (int i = 0; i < 16; i++) outputPercent[i] = sbusToPercent(sbusChannels[i]);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print("RC[%] ");
    for (int i = 0; i < 16; i++) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(":");
      Serial.print(outputPercent[i], 1);
      if (i < 15) Serial.print(" ");
    }
    Serial.print(" | FS:");
    Serial.print(sbusFailsafe);
    Serial.print(" LF:");
    Serial.print(sbusFrameLost);
    Serial.println();
  }
}

// =====================================================
// ===== Local Rotary / Linear / Polisher / Emergency ====
// =====================================================
int max_speed_rotary = 100;

void move_right(int speed) {
  digitalWrite(mRightFWD, LOW);
  digitalWrite(mRightREV, HIGH);
  analogWrite(mmRPin, speed);
}

void move_left(int speed) {
  digitalWrite(mRightFWD, HIGH);
  digitalWrite(mRightREV, LOW);
  analogWrite(mmRPin, speed);
}

void stop_rotary() {
  digitalWrite(mRightFWD, LOW);
  digitalWrite(mRightREV, LOW);
  analogWrite(mmRPin, 0);
}

void rotary_from_percent(float p) {
  int speed_rotary = abs(p / 100.0f * max_speed_rotary);
  if (p > 30) move_right(speed_rotary);
  else if (p < -30) move_left(speed_rotary);
  else stop_rotary();
}

void linear_from_percent(float p) {
  if (p < -30) {
    digitalWrite(linact_fwd_pin, HIGH);
    digitalWrite(linact_bwd_pin, LOW);
  } else if (p > 30) {
    digitalWrite(linact_bwd_pin, HIGH);
    digitalWrite(linact_fwd_pin, LOW);
  } else {
    digitalWrite(linact_fwd_pin, LOW);
    digitalWrite(linact_bwd_pin, LOW);
  }
}

void polish_start() {
  digitalWrite(polisher_S1, HIGH);
  digitalWrite(polisher_S2, HIGH);
}

void polish_stop() {
  digitalWrite(polisher_S1, LOW);
  digitalWrite(polisher_S2, LOW);
  delay(2);
}

void polisher_from_percent(float p) {
  if (p > 52) polish_start();
  else if (p < -50) polish_stop();
  // else: keep state (hysteresis-like behavior)
}

void emergency_from_percent(float p) {
  static bool emergencyState = false;
  if (p > 60.0f) emergencyState = true;
  else if (p < 40.0f) emergencyState = false;
  digitalWrite(emergency_pin, emergencyState ? LOW : HIGH); // active LOW
}

// RC wrappers (same behavior you had)
void polisherControlRC() { polisher_from_percent(percent[4]); }   // CH5
void emergencyCheckerRC() { emergency_from_percent(percent[5]); } // CH6

// Serial safe fallback
void emergency_force_active() {
  digitalWrite(emergency_pin, LOW); // active LOW (force emergency)
}

// =====================================================
// ===================== Drive Core =====================
// =====================================================
int32_t rpmFromPercent(float p) {
  if (fabs(p) < deadband_percent) return 0;
  p = clampf(p, -100.0f, 100.0f);
  return (int32_t)(p / 100.0f * max_speed);
}

CmdVel rc_to_cmdvel() {
  float th_cmd = percent[1]; // CH2
  float st_cmd = percent[3]; // CH4

  if (RC_FORWARD_IS_NEGATIVE) th_cmd = -th_cmd;
  if (RC_RIGHT_IS_POSITIVE) st_cmd = -st_cmd;

  th_cmd = applyDeadband(th_cmd, TH_DB);
  st_cmd = applyDeadband(st_cmd, ST_DB);

  CmdVel cmd;
  cmd.vx = (th_cmd / 100.0f) * MAX_VX_MPS;
  cmd.wz = (st_cmd / 100.0f) * MAX_WZ_RPS;
  cmd.valid = true;

  if (INVERT_STEER_WHEN_REVERSE && cmd.vx < 0.0f) cmd.wz = -cmd.wz;

  if (INVERT_WZ_WHEN_SPIN_ONLY) {
    bool spinOnly = (th_cmd == 0.0f && st_cmd != 0.0f);
    if (spinOnly) cmd.wz = -cmd.wz;
  }

  return cmd;
}

void drive_from_cmdvel(const CmdVel &cmdIn) {
  if (!cmdIn.valid) return;

  CmdVel cmd = cmdIn;
  cmd.vx = clampAbs(cmd.vx, MAX_VX_MPS);
  cmd.wz = clampAbs(cmd.wz, MAX_WZ_RPS);

  float th_cmd = 0.0f;
  float st_cmd = 0.0f;

  if (fabs(MAX_VX_MPS) > 1e-6f) th_cmd = (cmd.vx / MAX_VX_MPS) * 100.0f;
  if (fabs(MAX_WZ_RPS) > 1e-6f) st_cmd = (cmd.wz / MAX_WZ_RPS) * 100.0f;

  th_cmd = clampf(th_cmd, -100.0f, 100.0f);
  st_cmd = clampf(st_cmd, -100.0f, 100.0f);

  th_cmd = applyDeadband(th_cmd, TH_DB);
  st_cmd = applyDeadband(st_cmd, ST_DB);

  bool neutralNow = (th_cmd == 0.0f && st_cmd == 0.0f);
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

  float left = th_cmd - st_cmd;
  float right = th_cmd + st_cmd;

  left *= left_motor_trim;
  right *= right_motor_trim;

  left = clampf(left, -100.0f, 100.0f);
  right = clampf(right, -100.0f, 100.0f);

  int32_t leftRpm = rpmFromPercent(left);
  int32_t rightRpm = rpmFromPercent(right);

  if (INVERT_LEFT_MOTOR) leftRpm = -leftRpm;
  if (INVERT_RIGHT_MOTOR) rightRpm = -rightRpm;

  direct_velocity(SLAVE_1, leftRpm, ACC_NORM, DEC_RUN);
  delay(3);
  direct_velocity(SLAVE_2, rightRpm, ACC_NORM, DEC_RUN);
}

// =====================================================
// ================== Unified Serial Parser =============
// =====================================================
// Commands:
//   @T vx wz
//   @R percent
//   @L percent
//   @P percent
//   @E percent
bool read_commands_from_serial(CmdVel &tw, bool &gotTw, AuxCmd &aux, bool &gotAux) {
  static char line[96];
  static uint8_t idx = 0;

  gotTw = false;
  gotAux = false;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (idx == 0) continue;
      line[idx] = '\0';
      idx = 0;

      if (line[0] != '@') continue;

      char cmd = line[1];
      char *p = line + 2;
      while (*p == ' ' || *p == '\t') p++;

      // @T vx wz
      if (cmd == 'T' || cmd == 't') {
        char *end1 = nullptr;
        double vx = strtod(p, &end1);
        if (end1 == p) { Serial.print("BAD "); Serial.println(line); continue; }

        p = end1;
        while (*p == ' ' || *p == '\t') p++;

        char *end2 = nullptr;
        double wz = strtod(p, &end2);
        if (end2 == p) { Serial.print("BAD "); Serial.println(line); continue; }

        tw.vx = (float)vx;
        tw.wz = (float)wz;
        tw.valid = true;
        gotTw = true;

        Serial.print("ACK T ");
        Serial.print(tw.vx, 3);
        Serial.print(" ");
        Serial.println(tw.wz, 3);
        continue;
      }

      // Single number for R/L/P/E
      char *endv = nullptr;
      double v = strtod(p, &endv);
      if (endv == p) { Serial.print("BAD "); Serial.println(line); continue; }

      float pv = clampf((float)v, -100.0f, 100.0f);

      if (cmd == 'R' || cmd == 'r') {
        aux.rotary = pv;
        aux.valid = true;
        gotAux = true;
        Serial.print("ACK R ");
        Serial.println(aux.rotary, 1);
        continue;
      }

      if (cmd == 'L' || cmd == 'l') {
        aux.lin = pv;
        aux.valid = true;
        gotAux = true;
        Serial.print("ACK L ");
        Serial.println(aux.lin, 1);
        continue;
      }

      if (cmd == 'P' || cmd == 'p') {
        aux.polisher = pv;
        aux.valid = true;
        gotAux = true;
        Serial.print("ACK P ");
        Serial.println(aux.polisher, 1);
        continue;
      }

      if (cmd == 'E' || cmd == 'e') {
        aux.emergency = pv;
        aux.valid = true;
        gotAux = true;
        Serial.print("ACK E ");
        Serial.println(aux.emergency, 1);
        continue;
      }

      continue;
    }

    if (idx < sizeof(line) - 1) line[idx++] = c;
    else idx = 0;
  }

  return gotTw || gotAux;
}

// =====================================================
// ======================= Setup ========================
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  MB_SERIAL.begin(MB_BAUD, SERIAL_8E1);
  sbus_rx.Begin();

  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);

  pinMode(linact_fwd_pin, OUTPUT);
  pinMode(linact_bwd_pin, OUTPUT);
  pinMode(polisher_S1, OUTPUT);
  pinMode(polisher_S2, OUTPUT);

  pinMode(mRightFWD, OUTPUT);
  pinMode(mRightREV, OUTPUT);
  pinMode(stopModeR, OUTPUT);
  pinMode(m0R, OUTPUT);
  pinMode(mbFreeR, OUTPUT);
  pinMode(mmRPin, OUTPUT);

  pinMode(emergency_pin, OUTPUT);

  digitalWrite(mRightFWD, LOW);
  digitalWrite(mRightREV, LOW);
  digitalWrite(stopModeR, LOW);
  digitalWrite(m0R, HIGH);
  digitalWrite(mbFreeR, HIGH);
  digitalWrite(mmRPin, LOW);

  digitalWrite(linact_fwd_pin, LOW);
  digitalWrite(linact_bwd_pin, LOW);
  polish_stop();
  digitalWrite(emergency_pin, HIGH); // emergency inactive (active LOW)

  set_son(SLAVE_1, true);
  delay(200);
  set_son(SLAVE_2, true);
  delay(300);

  brake_blv_both_now();
}

// =====================================================
// ======================== Loop ========================
// =====================================================
void loop() {
  // --- Read SBUS ---
  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.data();
    for (int i = 0; i < 16; i++) sbusChannels[i] = sbus_data.ch[i];
    sbusFrameLost = sbus_data.lost_frame;
    sbusFailsafe = sbus_data.failsafe;
    readAllChannels(percent);
  }

  // --- 25Hz update gate (drive + aux) ---
  static unsigned long lastTick = 0;
  const unsigned long PERIOD_MS = 40;
  if (millis() - lastTick < PERIOD_MS) return;
  lastTick = millis();

  bool rcMode = (sbusChannels[MODE_CH_INDEX] > SBUS_MID_PULSE);

  // --- Serial mode: parse unified commands ---
  if (!rcMode) {
    CmdVel twTmp;
    AuxCmd auxTmp = lastAuxCmd; // keep last values when only one field updated

    bool gotTw = false, gotAux = false;
    read_commands_from_serial(twTmp, gotTw, auxTmp, gotAux);

    if (gotTw) {
      lastSerialCmd = twTmp;
      lastSerialCmdMs = millis();
      Serial.print("CMD ");
      Serial.print(twTmp.vx, 3);
      Serial.print(" ");
      Serial.println(twTmp.wz, 3);
    }

    if (gotAux) {
      lastAuxCmd = auxTmp;
      lastAuxCmdMs = millis();
    }
  }

  // --- Build drive cmd (RC or Serial) ---
  CmdVel cmd = {0.0f, 0.0f, true};

  if (rcMode) {
    cmd = rc_to_cmdvel();
  } else {
    bool fresh = (millis() - lastSerialCmdMs) <= SERIAL_CMD_TIMEOUT_MS;
    if (fresh && lastSerialCmd.valid) {
      cmd = lastSerialCmd;
      cmd.valid = true;
    } else {
      cmd.vx = 0.0f;
      cmd.wz = 0.0f;
      cmd.valid = true;
    }

    if (SERIAL_VX_INVERT) cmd.vx = -cmd.vx;
    if (SERIAL_WZ_INVERT) cmd.wz = -cmd.wz;
  }

  // --- Apply AUX (RC or Serial) ---
  if (rcMode) {
    rotary_from_percent(percent[0]);      // CH1
    linear_from_percent(percent[2]);      // CH3
    polisherControlRC();                  // CH5
    emergencyCheckerRC();                 // CH6
  } else {
    bool auxFresh = (millis() - lastAuxCmdMs) <= AUX_CMD_TIMEOUT_MS;

    if (!auxFresh || !lastAuxCmd.valid) {
      rotary_from_percent(0.0f);
      linear_from_percent(0.0f);
      polish_stop();
      emergency_force_active();           // safety default
    } else {
      rotary_from_percent(lastAuxCmd.rotary);
      linear_from_percent(lastAuxCmd.lin);
      polisher_from_percent(lastAuxCmd.polisher);
      emergency_from_percent(lastAuxCmd.emergency);
    }
  }

  // --- Debug mode print ---
  static unsigned long lastModePrint = 0;
  if (millis() - lastModePrint >= 250) {
    lastModePrint = millis();
    Serial.print("MODE:");
    Serial.print(rcMode ? "RC" : "SER");
    Serial.print(" CH8raw:");
    Serial.print(sbusChannels[MODE_CH_INDEX]);
    Serial.print(" lastV:");
    Serial.print(lastSerialCmd.vx, 3);
    Serial.print(" lastW:");
    Serial.println(lastSerialCmd.wz, 3);
  }

  // --- Drive output ---
  drive_from_cmdvel(cmd);
}