#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <EEPROM.h>
#include "sbus.h"


// Message structs
struct AuxCmd {
  float rotary;
  float lin;
  float polisher;
  float emergency;
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


// Pin definitions
#define DE_RE_PIN 43
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

#define limit_right A14
#define limit_left A15

#define voltage_reader_top A1
#define voltage_reader_bot A0

#define ENC_A 2
#define ENC_B 3
#define ENC_Z 18


// Tuning parameters
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
bool enable_right = true;
bool enable_left = true;

float MAX_VX_MPS = 1.0f;
float MAX_WZ_RPS = 1.0f;

const bool RC_FORWARD_IS_NEGATIVE = false;
const bool RC_RIGHT_IS_POSITIVE = true;
const bool INVERT_STEER_WHEN_REVERSE = true;

const bool INVERT_LEFT_MOTOR = true;
const bool INVERT_RIGHT_MOTOR = false;

const bool SERIAL_VX_INVERT = true;
const bool SERIAL_WZ_INVERT = true;


// Mode selection
static constexpr int MODE_CH_INDEX = 7;
static constexpr int SBUS_MID_PULSE = 992;


// Serial state
unsigned long lastSerialRxMs = 0;
bool serialRxSeen = false;
const unsigned long SERIAL_LINK_TIMEOUT_MS = 300;

CmdVel lastSerialCmd = { 0.0f, 0.0f, false };
unsigned long lastSerialCmdMs = 0;
const unsigned long SERIAL_TWIST_TIMEOUT_MS = 300;

AuxCmd lastAuxCmd = { 0.0f, 0.0f, -100.0f, -100.0f, false };
unsigned long lastAuxCmdMs = 0;
const unsigned long AUX_VALUES_TIMEOUT_MS = 300;


// Encoder state — declared here so EEPROM functions below can reference it
volatile long encoderCount = 0;
volatile bool zPulse = false;

// EEPROM encoder persistence
#define EEPROM_ADDR_ENCODER 0  // 4 bytes (long)
#define EEPROM_ADDR_VALID 4    // 1 byte  (magic)
#define EEPROM_MAGIC 0xA5

const unsigned long EEPROM_SAVE_INTERVAL_MS = 5000;  // save every 5 seconds

void loadEncoderFromEEPROM() {
  uint8_t magic;
  EEPROM.get(EEPROM_ADDR_VALID, magic);

  if (magic == EEPROM_MAGIC) {
    long saved;
    EEPROM.get(EEPROM_ADDR_ENCODER, saved);
    noInterrupts();
    encoderCount = saved;
    interrupts();
    Serial.print("[EEPROM] Encoder restored: ");
    Serial.println(saved);
  } else {
    Serial.println("[EEPROM] No saved encoder — starting from 0");
  }
}

void saveEncoderToEEPROM() {
  noInterrupts();
  long count = encoderCount;
  interrupts();

  EEPROM.put(EEPROM_ADDR_ENCODER, count);
  EEPROM.put(EEPROM_ADDR_VALID, (uint8_t)EEPROM_MAGIC);
}

void resetEncoderEEPROM() {
  long zero = 0L;
  EEPROM.put(EEPROM_ADDR_ENCODER, zero);
  EEPROM.put(EEPROM_ADDR_VALID, (uint8_t)EEPROM_MAGIC);
  noInterrupts();
  encoderCount = 0;
  interrupts();
  Serial.println("[EEPROM] Encoder reset to 0");
}


// Linear actuator feedback
const uint8_t N_SAMPLES = 16;
const float EMA_ALPHA = 0.25f;
float emaRaw = 0;

const uint16_t RAW_MIN = 0;
const uint16_t RAW_MAX = 1017;
const float STROKE_CM = 15.5f;

LinakReading g_linak;
unsigned long g_linak_ms = 0;

float g_rpm = 0.0f;
float g_angle = 0.0f;


// Helpers
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

uint16_t readAvg(uint8_t pin) {
  uint32_t s = 0;
  for (uint8_t i = 0; i < N_SAMPLES; i++) {
    s += analogRead(pin);
    delayMicroseconds(400);
  }
  return (uint16_t)(s / N_SAMPLES);
}

uint16_t readAvgN(uint8_t pin, uint8_t n) {
  uint32_t s = 0;
  for (uint8_t i = 0; i < n; i++) {
    s += analogRead(pin);
    delayMicroseconds(100);
  }
  return (uint16_t)(s / n);
}


// Linear actuator
void updateLinakFast() {
  uint16_t raw = readAvg(linak_feedback);
  emaRaw = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * emaRaw;
  g_linak.raw_filt = (uint16_t)(emaRaw + 0.5f);

  float pos01 = (float)((int)g_linak.raw_filt - (int)RAW_MIN)
                / (float)((int)RAW_MAX - (int)RAW_MIN);
  pos01 = clamp01(pos01);
  g_linak.pos_mm = pos01 * STROKE_CM * 10.0f;
  g_linak_ms = millis();
}

LinakReading readLinakPosition() {
  LinakReading r;
  uint16_t raw = readAvg(linak_feedback);
  emaRaw = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * emaRaw;
  r.raw_filt = (uint16_t)(emaRaw + 0.5f);

  float pos01 = (float)((int)r.raw_filt - (int)RAW_MIN)
                / (float)((int)RAW_MAX - (int)RAW_MIN);
  pos01 = clamp01(pos01);
  r.pos_mm = pos01 * STROKE_CM * 10.0f;
  return r;
}


// Voltage reader
const float VREF = 5.0f;
const float R_TOP = 47000.0f;
const float R_BOT = 10000.0f;
const float DIV_GAIN = (R_TOP + R_BOT) / R_BOT;
const uint8_t VIN_SAMPLES = 8;
const float VIN_EMA_ALPHA = 0.2f;

static float vinEmaTop = 0.0f;
static float vinEmaBot = 0.0f;

float readBattery(uint8_t adcPin, float &emaState) {
  uint16_t adc = readAvgN(adcPin, VIN_SAMPLES);
  if (emaState == 0.0f) emaState = adc;
  emaState = VIN_EMA_ALPHA * adc + (1.0f - VIN_EMA_ALPHA) * emaState;
  float vout = emaState * (VREF / 1023.0f);
  return vout * DIV_GAIN;
}

float readBatteryTop() {
  return readBattery(voltage_reader_top, vinEmaTop);
}
float readBatteryBottom() {
  return readBattery(voltage_reader_bot, vinEmaBot);
}


// Encoder
const int PPR = 1000;
const int CPR = PPR * 4;

void onEncoderA() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) encoderCount++;
  else encoderCount--;
}

void onEncoderB() {
  if (digitalRead(ENC_A) != digitalRead(ENC_B)) encoderCount++;
  else encoderCount--;
}

void onEncoderZ() {
  zPulse = true;
}

void setupEncoder() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), onEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), onEncoderZ, RISING);
}

float getAngleDeg() {
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return (float)(count % CPR) / (float)CPR * 360.0f;
}

void updateEncoderData() {
  static long lastCount = 0;
  static unsigned long lastMs = 0;

  unsigned long now = millis();
  unsigned long dt = now - lastMs;
  if (dt < 20) return;

  noInterrupts();
  long count = encoderCount;
  interrupts();

  g_angle = (float)(count % CPR) / (float)CPR * 360.0f;
  g_rpm = (float)(count - lastCount) / (float)CPR * (60000.0f / (float)dt);
  lastCount = count;
  lastMs = now;
}


// SBUS
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

float sbusToPercentNormalized(int raw, int minP, int midP, int maxP,
                              float dz = 3.0f) {
  raw = constrain(raw, minP, maxP);
  float p = 0.0f;
  if (raw < midP) p = -100.0f * ((float)(midP - raw) / (float)(midP - minP));
  else if (raw > midP) p = 100.0f * ((float)(raw - midP) / (float)(maxP - midP));
  if (fabs(p) < dz) p = 0.0f;
  return clampf(p, -100.0f, 100.0f);
}

void readAllChannels(float out[16]) {
  for (int i = 0; i < 16; i++) {
    if (i < 4)
      out[i] = sbusToPercentNormalized(sbusChannels[i],
                                       SBUS_14_MIN, SBUS_14_MID, SBUS_14_MAX, SBUS_ZERO_DZ);
    else
      out[i] = sbusToPercentNormalized(sbusChannels[i],
                                       SBUS_5_16_MIN, SBUS_5_16_MID, SBUS_5_16_MAX, SBUS_ZERO_DZ);
  }
}


// Modbus RS-485
// Shared bus: wheel-motor BLV drives (SLAVE_1/SLAVE_2) + GA500 polisher
// inverter (GA500_SLAVE) all on one RS-485 line at 115200 bps — this is the
// GA500's max supported MEMOBUS/Modbus speed (H5-02=8), so the BLV drives
// must also be reconfigured to 115200 bps to match (all nodes on one RS-485
// bus must share the same baud/framing). Framing is 8E1 for all three.
// Caveat: transactions on a shared bus are sequential/blocking — if the
// GA500 ever fails to respond, poll_polisher_current() can stall this bus
// for up to its ~50 ms timeout, delaying the next wheel-motor command.
#define MB_SERIAL Serial3
#define MB_BAUD 115200
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
  p[0] = (v >> 24) & 0xFF;
  p[1] = (v >> 16) & 0xFF;
  p[2] = (v >> 8) & 0xFF;
  p[3] = v & 0xFF;
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
  return (id == SLAVE_1) ? (son1 ? 1 : 0) : (son2 ? 1 : 0);
}

void write_cmd2_lower(uint8_t slaveId, uint16_t lower) {
  uint8_t f[] = {
    slaveId, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04,
    0x00, 0x00,
    (uint8_t)((lower >> 8) & 0xFF),
    (uint8_t)(lower & 0xFF)
  };
  modbus_send(f, sizeof(f));
}

void set_son(uint8_t slaveId, bool on) {
  if (slaveId == SLAVE_1) son1 = on;
  if (slaveId == SLAVE_2) son2 = on;
  write_cmd2_lower(slaveId, on ? 0x0001 : 0x0000);
}

void stop_pulse(uint8_t slaveId) {
  const uint16_t STOP_BIT = (1u << 5);
  uint16_t keep = sonBit(slaveId);
  write_cmd2_lower(slaveId, keep | STOP_BIT);
  delay(5);
  write_cmd2_lower(slaveId, keep);
}

void direct_velocity(uint8_t slaveId, int32_t rpm, int32_t acc, int32_t dec,
                     int32_t torqueLimit = TORQUE_NORMAL) {
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

void write_param_32(uint8_t slaveId, uint16_t regAddr, int32_t value) {
  uint8_t f[] = {
    slaveId, 0x10,
    (uint8_t)((regAddr >> 8) & 0xFF), (uint8_t)(regAddr & 0xFF),
    0x00, 0x02, 0x04,
    (uint8_t)((value >> 24) & 0xFF), (uint8_t)((value >> 16) & 0xFF),
    (uint8_t)((value >> 8) & 0xFF), (uint8_t)(value & 0xFF)
  };
  modbus_send(f, sizeof(f));
  delay(10);
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


// GA500 inverter (polisher) — shares the same RS-485 bus/UART (MB_SERIAL,
// DE_RE_PIN) as the wheel-motor BLV drives above, now that everything runs
// at 115200 bps. On the GA500: set H5-01 = GA500_SLAVE (must not collide
// with SLAVE_1/SLAVE_2 above), H5-02 = 8 (115.2k), H5-03 = 1 (even parity),
// and DIP switch S2 = ON (it is the last device on this bus segment).
// Verified externally 2026-07-27 via a Mac + Humandata USB-003 RS-485
// adapter: FC03 read of 0x0026 returned raw=10 (1.0 A) while the polisher
// was running, matching the GA500 keypad display (0.99 A) — the address,
// scaling, and drive-side H5 settings are all confirmed correct.
#define GA500_SLAVE 0x03   // must match GA500 parameter H5-01

// U1-03 [Output Current], MEMOBUS/Modbus register 0026h — fixed 0.1 A per
// count regardless of drive frame size (GA500 Technical Reference, Table 6.16).
#define GA500_REG_OUTPUT_CURRENT 0x0026

static float g_polisherCurrentA  = 0.0f;
static bool  g_polisherCurrentOk = false;

// Diagnostics from the most recent modbus_read_holding() call — lets the
// caller tell "GA500 never answered" (rxLen=0) apart from "answered but the
// frame was short/wrong/corrupted" (0 < rxLen < expected, or rxLen matches
// but validation still failed) without needing a logic analyzer.
static uint8_t g_lastRxLen = 0;
static uint8_t g_lastRxBuf[8];

// Reads `qty` (1-16) holding registers starting at regAddr via function code
// 03h. outData[0..qty-1] receive the big-endian register values. Returns
// false on timeout, a short/garbled frame, CRC mismatch, or an exception
// response (func byte echoed as 0x83 instead of 0x03).
bool modbus_read_holding(uint8_t slaveId, uint16_t regAddr, uint8_t qty, uint16_t *outData) {
  uint8_t req[8] = {
    slaveId, 0x03,
    (uint8_t)(regAddr >> 8), (uint8_t)(regAddr & 0xFF),
    0, qty
  };
  uint16_t crc = crc16_modbus(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  while (MB_SERIAL.available()) MB_SERIAL.read();
  rs485_tx();
  MB_SERIAL.write(req, sizeof(req));
  MB_SERIAL.flush();
  rs485_rx();

  const uint8_t expectLen = 5 + qty * 2;  // slave+func+bytecount+data+CRC(2)
  uint8_t resp[3 + 16 * 2 + 2];
  uint8_t got = 0;
  unsigned long start = millis();
  while (got < expectLen && (millis() - start) < 50) {
    if (MB_SERIAL.available()) resp[got++] = MB_SERIAL.read();
  }

  g_lastRxLen = got;
  memcpy(g_lastRxBuf, resp, got > sizeof(g_lastRxBuf) ? sizeof(g_lastRxBuf) : got);

  if (got < expectLen) return false;                        // timeout / short frame
  if (resp[0] != slaveId) return false;
  if (resp[1] != 0x03 || resp[2] != qty * 2) return false;   // wrong func / exception / bad byte count

  uint16_t rxCrc = crc16_modbus(resp, expectLen - 2);
  if (resp[expectLen - 2] != (rxCrc & 0xFF) || resp[expectLen - 1] != ((rxCrc >> 8) & 0xFF))
    return false;

  for (uint8_t i = 0; i < qty; i++)
    outData[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[3 + i * 2 + 1];
  return true;
}

// Polls the polisher inverter's output current. Call periodically (e.g. 5 Hz);
// updates g_polisherCurrentA / g_polisherCurrentOk. Blocks up to ~50 ms
// (the read timeout above) on the caller.
void poll_polisher_current() {
  uint16_t reg;
  g_polisherCurrentOk = modbus_read_holding(GA500_SLAVE, GA500_REG_OUTPUT_CURRENT, 1, &reg);
  if (g_polisherCurrentOk) g_polisherCurrentA = (float)reg / 10.0f;
}

// Higher-resolution current. Register 0042h is the MEMOBUS address for
// U1-03 [Output Current] using the drive's internal monitor-number scheme
// (the same address stored by default in H5-27 for the Function 5A
// read-back mechanism), as opposed to 0026h above which is the fixed-scale
// (0.1 A/count) address from the Communications Data Table. Per the GA500
// manual, 0042h returns a RAW ratio: 8192 counts = 100% of drive rated
// current.
//
// Drive nameplate catalog code GA50AB010ABA decodes to model B010,
// single-phase 200V class (Table 1.2, GA500 Technical Reference). Rated
// output current is 9.6 A at the default Normal Duty setting (C6-01=1),
// or 8.0 A if C6-01 has been changed to Heavy Duty (0) — swap the constant
// below if that parameter is set differently on this drive.
// Cross-checked 2026-07-27: raw=847 -> 847/8192*9.6 = 0.99 A, matching the
// 0026h/keypad reading of 1.00 A at the same instant.
#define GA500_REG_OUTPUT_CURRENT_RAW 0x0042
static const float GA500_RATED_CURRENT_A = 9.6f;  // B010, Normal Duty (C6-01=1, default)

static uint16_t g_polisherRaw42       = 0;
static bool     g_polisherRaw42Ok     = false;
static float    g_polisherCurrentHiResA = 0.0f;

void poll_polisher_current_raw() {
  uint16_t reg;
  g_polisherRaw42Ok = modbus_read_holding(GA500_SLAVE, GA500_REG_OUTPUT_CURRENT_RAW, 1, &reg);
  if (g_polisherRaw42Ok) {
    g_polisherRaw42 = reg;
    g_polisherCurrentHiResA = (reg / 8192.0f) * GA500_RATED_CURRENT_A;
  }
}


// Limit switch & rotary motor
void stop_rotary() {
  digitalWrite(mRightFWD, LOW);
  digitalWrite(mRightREV, LOW);
  digitalWrite(mbFreeR, LOW);
  analogWrite(mmRPin, 0);
}

int max_speed_rotary = 100;

void move_right(int speed) {
  if (digitalRead(limit_right) == LOW) {
    digitalWrite(mbFreeR, HIGH);
    digitalWrite(mRightFWD, LOW);
    digitalWrite(mRightREV, HIGH);
    analogWrite(mmRPin, speed);
  } else {
    stop_rotary();
  }
}

void move_left(int speed) {
  if (digitalRead(limit_left) == LOW) {
    digitalWrite(mbFreeR, HIGH);
    digitalWrite(mRightFWD, HIGH);
    digitalWrite(mRightREV, LOW);
    analogWrite(mmRPin, speed);
  } else {
    stop_rotary();
  }
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


// Polisher & emergency
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

void emergency_force_active() {
  digitalWrite(emergency_pin, LOW);
}


// Drive
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

void drive_from_cmdvel(const CmdVel &cmdIn) {
  if (!cmdIn.valid) return;

  CmdVel cmd = cmdIn;
  cmd.vx = clampAbs(cmd.vx, MAX_VX_MPS);
  cmd.wz = clampAbs(cmd.wz, MAX_WZ_RPS);

  float th_cmd = 0.0f, st_cmd = 0.0f;
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

  float left = clampf((th_cmd - st_cmd) * left_motor_trim, -100.0f, 100.0f);
  float right = clampf((th_cmd + st_cmd) * right_motor_trim, -100.0f, 100.0f);

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


// Serial commands
// Format: @T vx wz | @R pct | @L pct | @P pct | @E pct | @Z (reset encoder)
bool read_commands_from_serial(CmdVel &tw, bool &gotTw,
                               AuxCmd &aux, bool &gotAux) {
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

      // @Z reset encoder
      if (cmd == 'Z' || cmd == 'z') {
        resetEncoderEEPROM();
        Serial.println("ACK Z encoder=0");
        continue;
      }

      // Single-value commands: @R @L @P @E
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
      } else if (cmd == 'L' || cmd == 'l') {
        aux.lin = pv;
        aux.valid = true;
        gotAux = true;
        Serial.print("ACK L ");
        Serial.println(aux.lin, 1);
      } else if (cmd == 'P' || cmd == 'p') {
        aux.polisher = pv;
        aux.valid = true;
        gotAux = true;
        Serial.print("ACK P ");
        Serial.println(aux.polisher, 1);
      } else if (cmd == 'E' || cmd == 'e') {
        aux.emergency = pv;
        aux.valid = true;
        gotAux = true;
        Serial.print("ACK E ");
        Serial.println(aux.emergency, 1);
      }
    } else {
      if (idx < sizeof(line) - 1) line[idx++] = c;
      else idx = 0;
    }
  }

  return gotTw || gotAux;
}


// Telemetry
// Build full line into buffer then send in one write — avoids repeated blocking on TX buffer
void sendMachineData() {
  noInterrupts();
  long encSnap = encoderCount;
  interrupts();

  float batTop = readBatteryTop();
  float batBot = readBatteryBottom();
  bool limR = digitalRead(limit_right);
  bool limL = digitalRead(limit_left);

  // AVR snprintf does not support %f — use dtostrf to convert floats first
  char sPos[10], sBatT[10], sBatB[10], sAngle[10], sRpm[10];
  dtostrf(g_linak.pos_mm, 1, 1, sPos);
  dtostrf(batTop,         1, 2, sBatT);
  dtostrf(batBot,         1, 2, sBatB);
  dtostrf(g_angle,        1, 1, sAngle);
  dtostrf(g_rpm,          1, 1, sRpm);

  char buf[128];
  int len = snprintf(buf, sizeof(buf),
    "@machineData,%lu,%u,%s,%s,%s,%d,%d,%s,%s,%ld\n",
    millis(), g_linak.raw_filt, sPos, sBatT, sBatB,
    limR ? 1 : 0, limL ? 1 : 0, sAngle, sRpm, encSnap);
  if (len > 0 && len < (int)sizeof(buf))
    Serial.write(buf, len);
}

void sendSbusData() {
  char buf[128];
  int len = snprintf(buf, sizeof(buf),
    "@sbusData,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
    sbusChannels[0],  sbusChannels[1],  sbusChannels[2],  sbusChannels[3],
    sbusChannels[4],  sbusChannels[5],  sbusChannels[6],  sbusChannels[7],
    sbusChannels[8],  sbusChannels[9],  sbusChannels[10], sbusChannels[11],
    sbusChannels[12], sbusChannels[13], sbusChannels[14], sbusChannels[15]);
  if (len > 0 && len < (int)sizeof(buf))
    Serial.write(buf, len);
}

void sendPolisherData() {
  char sCur[10];
  dtostrf(g_polisherCurrentA, 1, 2, sCur);
  char sHiRes[10];
  dtostrf(g_polisherCurrentHiResA, 1, 3, sHiRes);

  // Fields: ms, amps(0.1A, 0026h), ok, bytes, raw042, ok042, hiResAmps(0042h, ~0.001A)
  char buf[96];
  int len = snprintf(buf, sizeof(buf), "@polisherData,%lu,%s,%d,%u,%u,%d,%s\n",
    millis(), sCur, g_polisherCurrentOk ? 1 : 0, g_lastRxLen,
    g_polisherRaw42, g_polisherRaw42Ok ? 1 : 0, sHiRes);
  if (len > 0 && len < (int)sizeof(buf))
    Serial.write(buf, len);
}


void setup() {
  Serial.begin(230400);
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
  pinMode(limit_right, INPUT);
  pinMode(limit_left, INPUT);

  // Force pins 20 & 21 LOW to avoid interfering with limit switches
  // (hardware fix preferred — disconnect these pins)
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);

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

  // Init linear actuator EMA
  uint16_t r0 = readAvg(linak_feedback);
  emaRaw = r0;

  // Setup encoder and restore position from EEPROM
  setupEncoder();
  loadEncoderFromEEPROM();

  set_son(SLAVE_1, true);
  delay(200);
  set_son(SLAVE_2, true);
  delay(300);

  write_param_32(SLAVE_1, 0x0296, 1);
  write_param_32(SLAVE_2, 0x0296, 1);

  brake_blv_both_now();

  Serial.println("[BOOT] Robot ready.");
}


void loop() {

  // Read SBUS
  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.data();
    // for (int i = 0; i < 16; i++) {
    //   Serial.print(sbus_data.ch[i]);
    //   if (i == 15) {
    //     Serial.println("");
    //   } else {
    //     Serial.print(",");
    //   }
    // }
    for (int i = 0; i < 16; i++) {
      sbusChannels[i] = sbus_data.ch[i];
    }
    readAllChannels(percent);
  }

  // Update sensor data
  updateEncoderData();
  updateLinakFast();

  // machineData at 50 Hz
  static unsigned long lastMachTel = 0;
  if (millis() - lastMachTel >= 20) {
    lastMachTel = millis();
    // sendMachineData();
  }

  // sbusData at 25 Hz, offset 10 ms from machineData to avoid TX collision
  static unsigned long lastSbusTel = 0;
  if (millis() - lastSbusTel >= 40) {
    lastSbusTel = millis() - 10;
    // sendSbusData();
  }

  // Polisher (GA500) output current at 5 Hz, shared bus with wheel motors
  static unsigned long lastPolisherPoll = 0;
  if (millis() - lastPolisherPoll >= 200) {
    lastPolisherPoll = millis();
    poll_polisher_current();
    poll_polisher_current_raw();
    sendPolisherData();
  }

  // Save encoder to EEPROM every 5 seconds
  static unsigned long lastEepromSave = 0;
  if (millis() - lastEepromSave >= EEPROM_SAVE_INTERVAL_MS) {
    lastEepromSave = millis();
    saveEncoderToEEPROM();
  }

  // Main control loop at 25 Hz
  static unsigned long lastTick = 0;
  const unsigned long PERIOD_MS = 40;
  if (millis() - lastTick < PERIOD_MS) return;
  lastTick = millis();

  bool rcMode = (percent[MODE_CH_INDEX] > 0.0f);

  // Read serial commands (serial mode only)
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

  // Build command
  CmdVel cmd = { 0.0f, 0.0f, true };

  if (rcMode) {
    cmd = rc_to_cmdvel();
    rotary_from_percent(percent[0]);
    linear_from_percent(percent[2]);
    polisher_from_percent(percent[4]);
    emergency_from_percent(percent[5]);

  } else {
    bool serialOk = serialRxSeen && ((millis() - lastSerialRxMs) <= SERIAL_LINK_TIMEOUT_MS);

    if (!serialOk) {
      // Serial link lost — full safe state
      cmd.vx = 0.0f;
      cmd.wz = 0.0f;
      cmd.valid = true;
      rotary_from_percent(0.0f);
      linear_from_percent(0.0f);
      polish_stop();
      emergency_force_active();

    } else {
      bool twFresh = lastSerialCmd.valid && ((millis() - lastSerialCmdMs) <= SERIAL_TWIST_TIMEOUT_MS);

      cmd.vx = twFresh ? lastSerialCmd.vx : 0.0f;
      cmd.wz = twFresh ? lastSerialCmd.wz : 0.0f;
      cmd.valid = true;

      if (SERIAL_VX_INVERT) cmd.vx = -cmd.vx;
      if (SERIAL_WZ_INVERT) cmd.wz = -cmd.wz;

      bool auxFresh = lastAuxCmd.valid && ((millis() - lastAuxCmdMs) <= AUX_VALUES_TIMEOUT_MS);

      rotary_from_percent(auxFresh ? lastAuxCmd.rotary : 0.0f);
      linear_from_percent(auxFresh ? lastAuxCmd.lin : 0.0f);

      if (auxFresh) polisher_from_percent(lastAuxCmd.polisher);
      else polish_stop();

      emergency_from_percent(lastAuxCmd.emergency);
    }
  }

  // Drive output
  drive_from_cmdvel(cmd);
}