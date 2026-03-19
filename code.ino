#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "sbus.h"
#include <stdlib.h>


// =====================================================
// =================== MESSAGE STRUCT ===================
// =====================================================
struct AuxCmd {
  float rotary;     // -100..100
  float lin;        // -100..100
  float polisher;   // -100..100
  float emergency;  // -100..100
  bool valid;
};

struct LinakReading {
  uint16_t raw_filt;
  float pos_mm;
};

struct CmdVel {
  float vx;
  float wz;
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

#define linak_fwd_pin 36
#define linak_bwd_pin 39
#define linak_feedback A13
#define PIN_END_IN 44
#define PIN_END_OUT 45

#define limit_right 20
#define limit_left 21

#define voltage_reader_top A1
#define voltage_reader_bot A0

float left_motor_trim = 1.0f;
float right_motor_trim = 1.0f;

int max_speed = 3800;
int deadband_percent = 5;

const float TH_DB = 8.0f;
const float ST_DB = 8.0f;

const int32_t ACC_NORM = 80;
const int32_t DEC_RUN = 60;
const int32_t ACC_BRAKE = 10;
const int32_t DEC_BRAKE = 10;

const int32_t TORQUE_NORMAL = 1500;
const int32_t TORQUE_ROTATE = 2100;
const int32_t TORQUE_BRAKE = 1500;

bool wasNeutral = false;

float MAX_VX_MPS = 1.0f;
float MAX_WZ_RPS = 1.0f;

const bool RC_FORWARD_IS_NEGATIVE = false;
const bool RC_RIGHT_IS_POSITIVE = true;
const bool INVERT_STEER_WHEN_REVERSE = true;

const bool INVERT_LEFT_MOTOR = true;
const bool INVERT_RIGHT_MOTOR = false;

const bool SERIAL_VX_INVERT = true;
const bool SERIAL_WZ_INVERT = true;

// =====================================================
// =================== Mode Selection ===================
// =====================================================
static constexpr int MODE_CH_INDEX = 7;  // CH8 (0-based)
static constexpr int SBUS_MID_PULSE = 992;

// =====================================================
// =================== Serial State =====================
// =====================================================
// One serial-link indicator:

unsigned long lastSerialRxMs = 0;
bool serialRxSeen = false;
const unsigned long SERIAL_LINK_TIMEOUT_MS = 300;

// Per-message freshness (not a separate "link indicator"):
CmdVel lastSerialCmd = { 0.0f, 0.0f, false };
unsigned long lastSerialCmdMs = 0;
const unsigned long SERIAL_TWIST_TIMEOUT_MS = 300;

AuxCmd lastAuxCmd = { 0.0f, 0.0f, -100.0f, -100.0f, false };  // default polisher OFF, emergency released
unsigned long lastAuxCmdMs = 0;
const unsigned long AUX_VALUES_TIMEOUT_MS = 300;

// =====================================================
// =================== Cylinder Feedback ================
// =====================================================

const uint8_t N_SAMPLES = 16;  // moving average
const float EMA_ALPHA = 0.25;  // 0..1
float emaRaw = 0;

const uint16_t RAW_MIN = 0;     // nilai ADC saat tidak extend (dari pengukuranmu)
const uint16_t RAW_MAX = 1017;  // nilai ADC saat extend maksimal (dari pengukuranmu)
const float STROKE_CM = 15.5f;  // panjang maksimal dalam cm (dari pengukuranmu)

LinakReading g_linak;
unsigned long g_linak_ms = 0;

void updateLinakFast() {
  uint16_t raw = readAvg(linak_feedback);
  emaRaw = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * emaRaw;
  g_linak.raw_filt = (uint16_t)(emaRaw + 0.5f);

  float pos01 = (float)((int)g_linak.raw_filt - (int)RAW_MIN) / (float)((int)RAW_MAX - (int)RAW_MIN);
  pos01 = clamp01(pos01);
  float pos_cm = pos01 * STROKE_CM;
  g_linak.pos_mm = pos_cm * 10.0f;

  g_linak_ms = millis();
}

void cylinder_feedback() {
  bool endIn = digitalRead(PIN_END_IN);
  bool endOut = digitalRead(PIN_END_OUT);
  uint16_t raw = readAvg(linak_feedback);

  emaRaw = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * emaRaw;
  uint16_t rf = (uint16_t)(emaRaw + 0.5f);

  // Mapping langsung dari ADC ke posisi aktual (cm/mm)
  float pos01 = (float)((int)rf - (int)RAW_MIN) / (float)((int)RAW_MAX - (int)RAW_MIN);
  pos01 = clamp01(pos01);
  float pos_cm = pos01 * STROKE_CM;
  float pos_mm = pos_cm * 10.0f;

  Serial.print("raw=");
  Serial.print(rf);
  Serial.print(" cm pos_mm=");
  Serial.println(pos_mm, 1);
}

// =====================================================
// ================== Serial Section =============
// =====================================================
// Commands:
//   @T vx wz
//   @R percent
//   @L percent
//   @P percent
//   @E percent
bool read_commands_from_serial(bool &gotTw, AuxCmd &aux, bool &gotAux) {
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

      if (cmd == 'T' || cmd == 't') {
        char *end1 = nullptr;
        double vx = strtod(p, &end1);
        if (end1 == p) continue;

        p = end1;
        while (*p == ' ' || *p == '\t') p++;

        char *end2 = nullptr;
        double wz = strtod(p, &end2);
        if (end2 == p) continue;
      }

      char *endv = nullptr;
      double v = strtod(p, &endv);
      if (endv == p) continue;

      float pv = clampf((float)v, -100.0f, 100.0f);

      if (cmd == 'R' || cmd == 'r') {
        aux.rotary = pv;
        aux.valid = true;
        gotAux = true;
        continue;
      }

      if (cmd == 'L' || cmd == 'l') {
        aux.lin = pv;
        aux.valid = true;
        gotAux = true;
        continue;
      }

      if (cmd == 'P' || cmd == 'p') {
        aux.polisher = pv;
        aux.valid = true;
        gotAux = true;
        continue;
      }

      if (cmd == 'E' || cmd == 'e') {
        aux.emergency = pv;
        aux.valid = true;
        gotAux = true;
        continue;
      }

      continue;
    }

    if (idx < sizeof(line) - 1) line[idx++] = c;
    else idx = 0;
  }

  return gotTw || gotAux;
}

LinakReading readLinakPosition() {
  LinakReading r;

  uint16_t raw = readAvg(linak_feedback);
  emaRaw = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * emaRaw;
  r.raw_filt = (uint16_t)(emaRaw + 0.5f);

  float pos01 = (float)((int)r.raw_filt - (int)RAW_MIN) / (float)((int)RAW_MAX - (int)RAW_MIN);
  pos01 = clamp01(pos01);
  float pos_cm = pos01 * STROKE_CM;
  r.pos_mm = pos_cm * 10.0f;

  return r;
}

void sendDataToSerial() {
  float batTop = readBatteryTop();
  float batBot = readBatteryBottom();

  Serial.print("@machineData,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(g_linak.raw_filt);
  Serial.print(",");
  Serial.print(g_linak.pos_mm, 1);
  Serial.print(",");
  Serial.print(batTop, 2);
  Serial.print(",");
  Serial.println(batBot, 2);
}

// =====================================================
// =================== Voltage Reader ===================
// =====================================================
const float VREF = 5.0f;
const float R_TOP = 47000.0f;
const float R_BOT = 10000.0f;
const float DIV_GAIN = (R_TOP + R_BOT) / R_BOT;

const uint8_t VIN_SAMPLES = 8;
const float VIN_EMA_ALPHA = 0.2f;

static float vinEmaTop = 0.0f;
static float vinEmaBot = 0.0f;

uint16_t readAvgN(uint8_t pin, uint8_t n) {
  uint32_t s = 0;
  for (uint8_t i = 0; i < n; i++) {
    s += analogRead(pin);
    delayMicroseconds(100);
  }
  return (uint16_t)(s / n);
}

float readBattery(uint8_t adcPin, float &emaState) {
  uint16_t adc = readAvgN(adcPin, VIN_SAMPLES);

  if (emaState == 0.0f) emaState = adc;
  emaState = VIN_EMA_ALPHA * adc + (1.0f - VIN_EMA_ALPHA) * emaState;

  float vout = emaState * (VREF / 1023.0f);
  float vin = vout * DIV_GAIN;
  return vin;
}

float readBatteryTop() {
  return readBattery(voltage_reader_top, vinEmaTop);
}

float readBatteryBottom() {
  return readBattery(voltage_reader_bot, vinEmaBot);
}

// =====================================================
// ======================= Helpers ======================
// =====================================================
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static inline float applyDeadband(float v, float db) {
  return (fabs(v) < db) ? 0.0f : v;
}

static inline float clampAbs(float v, float maxAbs) {
  if (v > maxAbs) return maxAbs;
  if (v < -maxAbs) return -maxAbs;
  return v;
}


static constexpr bool SBUS_INVERTED = false;
bfs::SbusRx sbus_rx(&Serial1, SBUS_INVERTED);
bfs::SbusData sbus_data;

int sbusChannels[16];
bool sbusFrameLost = false;
bool sbusFailsafe = false;
float percent[16];

static constexpr int SBUS_14_MIN = 368;
static constexpr int SBUS_14_MID = 1024;
static constexpr int SBUS_14_MAX = 1680;

static constexpr int SBUS_5_16_MIN = 144;
static constexpr int SBUS_5_16_MID = 1024;
static constexpr int SBUS_5_16_MAX = 1904;

static constexpr float SBUS_ZERO_DZ = 3.0f;

#define MB_SERIAL Serial2
#define MB_BAUD 230400
#define SLAVE_1 0x01
#define SLAVE_2 0x02

static bool son1 = false;
static bool son2 = false;

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

void direct_velocity(uint8_t slaveId, int32_t rpm, int32_t acc, int32_t dec, int32_t torqueLimit = TORQUE_NORMAL) {
  uint8_t f[35];
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
  put_i32_be(d + 20, torqueLimit);
  put_i32_be(d + 24, 1);

  modbus_send(f, sizeof(f));
}

void brake_blv_both_now() {
  stop_pulse(SLAVE_1);
  delay(5);
  stop_pulse(SLAVE_2);
  delay(5);
  direct_velocity(SLAVE_1, 0, ACC_BRAKE, DEC_BRAKE, TORQUE_BRAKE);
  delay(5);
  direct_velocity(SLAVE_2, 0, ACC_BRAKE, DEC_BRAKE, TORQUE_BRAKE);
  delay(5);
}

void hold_zero_blv() {
  direct_velocity(SLAVE_1, 0, ACC_BRAKE, DEC_BRAKE, TORQUE_BRAKE);
  delay(5);
  direct_velocity(SLAVE_2, 0, ACC_BRAKE, DEC_BRAKE, TORQUE_BRAKE);
  delay(5);
}

float sbusToPercentNormalized(int raw, int minPulse, int midPulse, int maxPulse, float deadzonePct = 3.0f) {
  raw = constrain(raw, minPulse, maxPulse);
  float p = 0.0f;
  if (raw < midPulse) {
    p = -100.0f * ((float)(midPulse - raw) / (float)(midPulse - minPulse));
  } else if (raw > midPulse) {
    p = 100.0f * ((float)(raw - midPulse) / (float)(maxPulse - midPulse));
  }
  if (fabs(p) < deadzonePct) p = 0.0f;
  return clampf(p, -100.0f, 100.0f);
}

void readAllChannels(float outputPercent[16]) {
  for (int i = 0; i < 16; i++) {
    if (i < 4) {
      outputPercent[i] = sbusToPercentNormalized(
        sbusChannels[i], SBUS_14_MIN, SBUS_14_MID, SBUS_14_MAX, SBUS_ZERO_DZ);
    } else {
      outputPercent[i] = sbusToPercentNormalized(
        sbusChannels[i], SBUS_5_16_MIN, SBUS_5_16_MID, SBUS_5_16_MAX, SBUS_ZERO_DZ);
    }
  }

  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint >= 200) {
  //   lastPrint = millis();
  //   Serial.print("RC ");
  //   for (int i = 0; i < 16; i++) {
  //     Serial.print("CH");
  //     Serial.print(i + 1);
  //     Serial.print(":");
  //     Serial.print(outputPercent[i], 1);
  //     if (i < 15) Serial.print(" ");
  //   }
  //   Serial.print(" | FS:");
  //   Serial.print(sbusFailsafe);
  //   Serial.print(" LF:");
  //   Serial.println(sbusFrameLost);
  // }
}

int max_speed_rotary = 100;

void move_right(int speed) {
  digitalWrite(mbFreeR, HIGH);
  digitalWrite(mRightFWD, LOW);
  digitalWrite(mRightREV, HIGH);
  analogWrite(mmRPin, speed);
}

void move_left(int speed) {
  digitalWrite(mbFreeR, HIGH);
  digitalWrite(mRightFWD, HIGH);
  digitalWrite(mRightREV, LOW);
  analogWrite(mmRPin, speed);
}

void stop_rotary() {
  digitalWrite(mRightFWD, LOW);
  digitalWrite(mRightREV, LOW);
  digitalWrite(mbFreeR, LOW);
  analogWrite(mmRPin, 0);
}

void rotary_from_percent(float p) {
  int speed_rotary = abs((int)(p / 100.0f * max_speed_rotary));
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
}

void emergency_from_percent(float p) {
  static bool emergencyState = false;
  if (p > 60.0f) emergencyState = true;
  else if (p < 40.0f) emergencyState = false;
  digitalWrite(emergency_pin, emergencyState ? LOW : HIGH);
}

void polisherControlRC() {
  polisher_from_percent(percent[4]);
}
void emergencyCheckerRC() {
  emergency_from_percent(percent[5]);
}

void emergency_force_active() {
  digitalWrite(emergency_pin, LOW);
}

int32_t rpmFromPercent(float p) {
  const float db = (float)deadband_percent;
  if (fabs(p) < db) return 0;

  float sign = (p > 0) ? 1.0f : -1.0f;
  float abs_p = fabs(p);
  float scaled = (abs_p - db) / (100.0f - db) * 100.0f;
  scaled = clampf(scaled, 0.0f, 100.0f);

  return (int32_t)(sign * scaled / 100.0f * (float)max_speed);
}

CmdVel rc_to_cmdvel() {
  float th_cmd = percent[1];
  float st_cmd = percent[3];

  if (RC_FORWARD_IS_NEGATIVE) th_cmd = -th_cmd;
  if (!RC_RIGHT_IS_POSITIVE) st_cmd = -st_cmd;

  th_cmd = applyDeadband(th_cmd, TH_DB);
  st_cmd = applyDeadband(st_cmd, ST_DB);

  CmdVel cmd;
  cmd.vx = (th_cmd / 100.0f) * MAX_VX_MPS;
  cmd.wz = (st_cmd / 100.0f) * MAX_WZ_RPS;
  cmd.valid = true;

  if (INVERT_STEER_WHEN_REVERSE && cmd.vx < 0.0f) cmd.wz = -cmd.wz;

  return cmd;
}

uint16_t readAvg(uint8_t pin) {
  uint32_t s = 0;
  for (uint8_t i = 0; i < N_SAMPLES; i++) {
    s += analogRead(pin);
    delayMicroseconds(400);
  }
  return (uint16_t)(s / N_SAMPLES);
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

  bool neutralNow = (fabs(th_cmd) < 0.5f && fabs(st_cmd) < 0.5f);
  if (neutralNow) {
    if (!wasNeutral) {
      brake_blv_both_now();
      wasNeutral = true;
    }
    return;
  }
  wasNeutral = false;

  bool isRotating = (fabs(th_cmd) < 0.5f && fabs(st_cmd) > 0.5f);
  int32_t torque = isRotating ? TORQUE_ROTATE : TORQUE_NORMAL;

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

  static int32_t lastLeftRpm = 0;
  static int32_t lastRightRpm = 0;
  static unsigned long lastSentMs = 0;

  const int32_t RPM_CHANGE_THRESHOLD = 30;
  const unsigned long REFRESH_MS = 200;

  bool changed = (abs(leftRpm - lastLeftRpm) >= RPM_CHANGE_THRESHOLD || abs(rightRpm - lastRightRpm) >= RPM_CHANGE_THRESHOLD);
  bool timeout = (millis() - lastSentMs >= REFRESH_MS);

  if (!changed && !timeout) return;

  lastLeftRpm = leftRpm;
  lastRightRpm = rightRpm;
  lastSentMs = millis();

  direct_velocity(SLAVE_1, leftRpm, ACC_NORM, DEC_RUN, torque);
  delay(5);
  direct_velocity(SLAVE_2, rightRpm, ACC_NORM, DEC_RUN, torque);
}

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

      if (cmd == 'T' || cmd == 't') {
        char *end1 = nullptr;
        double vx = strtod(p, &end1);
        if (end1 == p) {
          Serial.print("BAD ");
          Serial.println(line);
          continue;
        }

        p = end1;
        while (*p == ' ' || *p == '\t') p++;

        char *end2 = nullptr;
        double wz = strtod(p, &end2);
        if (end2 == p) {
          Serial.print("BAD ");
          Serial.println(line);
          continue;
        }

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

      char *endv = nullptr;
      double v = strtod(p, &endv);
      if (endv == p) {
        Serial.print("BAD ");
        Serial.println(line);
        continue;
      }

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
    } else {
      if (idx < sizeof(line) - 1) line[idx++] = c;
      else idx = 0;
    }
  }

  return gotTw || gotAux;
}

// Helper: tulis 1 register 32-bit ke parameter BLV
void write_param_32(uint8_t slaveId, uint16_t regAddr, int32_t value) {
  uint8_t f[] = {
    slaveId,
    0x10,
    (uint8_t)((regAddr >> 8) & 0xFF),
    (uint8_t)(regAddr & 0xFF),
    0x00, 0x02,  // 2 register (32-bit = 2x16-bit)
    0x04,        // 4 bytes
    (uint8_t)((value >> 24) & 0xFF),
    (uint8_t)((value >> 16) & 0xFF),
    (uint8_t)((value >> 8) & 0xFF),
    (uint8_t)(value & 0xFF)
  };
  modbus_send(f, sizeof(f));
  delay(10);
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
  digitalWrite(emergency_pin, HIGH);

  pinMode(PIN_END_IN, INPUT);
  pinMode(PIN_END_OUT, INPUT);

  pinMode(voltage_reader_top, INPUT);
  pinMode(voltage_reader_bot, INPUT);

  uint16_t r0 = readAvg(linak_feedback);
  emaRaw = r0;

  set_son(SLAVE_1, true);
  delay(200);
  set_son(SLAVE_2, true);
  delay(300);

  //ATL
  write_param_32(SLAVE_1, 0x0296, 1);
  write_param_32(SLAVE_2, 0x0296, 1);

  brake_blv_both_now();
}

void loop() {
  // --- Read SBUS ---
  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.data();
    for (int i = 0; i < 16; i++) {
      Serial.print(sbus_data.ch[i]);
      if (i == 15) {
        Serial.println("");
      } else {
        Serial.print(",");
      }
    }

    for (int i = 0; i < 16; i++) {
      sbusChannels[i] = sbus_data.ch[i];
    }

    readAllChannels(percent);
  }

  // cylinder_feedback();

  // updateLinakFast();

  // static unsigned long lastTel = 0;
  // const unsigned long machineData_PERIOD_MS = 20;  // 50 Hz
  // if (millis() - lastTel >= machineData_PERIOD_MS) {
  //   lastTel = millis();
  //   sendDataToSerial();
  // }

  static unsigned long lastTick = 0;
  const unsigned long PERIOD_MS = 40;
  if (millis() - lastTick < PERIOD_MS) return;
  lastTick = millis();

  bool rcMode = (percent[MODE_CH_INDEX] > 0.0f);

  if (!rcMode) {
    CmdVel twTmp;
    AuxCmd auxTmp = lastAuxCmd;

    bool gotTw = false, gotAux = false;
    bool gotAny = read_commands_from_serial(twTmp, gotTw, auxTmp, gotAux);

    if (gotTw) {
      lastSerialCmd = twTmp;
      lastSerialCmdMs = millis();
    }

    if (gotAux) {
      lastAuxCmd = auxTmp;
      lastAuxCmdMs = millis();
    }

    if (gotAny) {
      serialRxSeen = true;
      lastSerialRxMs = millis();
    }
  }

  CmdVel cmd = { 0.0f, 0.0f, true };

  if (rcMode) {
    cmd = rc_to_cmdvel();

    // AUX (RC)
    rotary_from_percent(percent[0]);     // CH1
    linear_from_percent(percent[2]);     // CH3
    polisher_from_percent(percent[4]);   // CH5
    emergency_from_percent(percent[5]);  // CH6

  } else {
    // One serial-link indicator:
    bool serialOk = serialRxSeen && ((millis() - lastSerialRxMs) <= SERIAL_LINK_TIMEOUT_MS);

    if (!serialOk) {
      // Total serial loss -> full safe
      cmd.vx = 0.0f;
      cmd.wz = 0.0f;
      cmd.valid = true;

      rotary_from_percent(0.0f);
      linear_from_percent(0.0f);
      polish_stop();
      emergency_force_active();

    } else {
      // Drive uses twist if fresh; otherwise stop drive (but link still OK)
      bool twFresh = lastSerialCmd.valid && ((millis() - lastSerialCmdMs) <= SERIAL_TWIST_TIMEOUT_MS);

      if (twFresh) {
        cmd = lastSerialCmd;
        cmd.valid = true;
      } else {
        cmd.vx = 0.0f;
        cmd.wz = 0.0f;
        cmd.valid = true;
      }

      if (SERIAL_VX_INVERT) cmd.vx = -cmd.vx;
      if (SERIAL_WZ_INVERT) cmd.wz = -cmd.wz;

      // AUX: if values stale, neutralize rotary/linear and stop polisher.
      bool auxFresh = lastAuxCmd.valid && ((millis() - lastAuxCmdMs) <= AUX_VALUES_TIMEOUT_MS);

      rotary_from_percent(auxFresh ? lastAuxCmd.rotary : 0.0f);
      linear_from_percent(auxFresh ? lastAuxCmd.lin : 0.0f);

      if (auxFresh) polisher_from_percent(lastAuxCmd.polisher);
      else polish_stop();

      // Emergency follows last commanded value whenever serial link is OK.
      // Default value is -100 (released) until a command arrives.
      emergency_from_percent(lastAuxCmd.emergency);
    }
  }

  // --- Drive output ---
  drive_from_cmdvel(cmd);
}