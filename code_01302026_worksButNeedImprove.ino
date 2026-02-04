#include <Arduino.h>
#include <math.h>
#include "sbus.h"

// ===== SBUS via library (Bolder Flight) =====
bfs::SbusRx sbus_rx(&Serial1);
bfs::SbusData sbus_data;

// ===== Section: Pin Definition =====
#define DE_RE_PIN 42
#define polisher_S1 46
#define polisher_S2 48
#define linact_fwd_pin 31
#define linact_bwd_pin 33
#define emergency_pin 35

#define mRightFWD 11
#define mRightREV 12
#define stopModeR 24
#define m0R 26
#define mbFreeR 28
#define mmRPin 13

#define limit_right 20 // Limit_A
#define limit_left 21 // Limit_B

// ===== Limit switch behavior =====
// Asumsi INPUT_PULLUP: switch ditekan => LOW
static const bool LIMIT_ACTIVE_LOW = true;

float left_motor_trim = 0.95f;
float right_motor_trim = 1.0f;

// ===== Section: SBUS =====
float percent[16];              // output -100..100
bool sbusFrameLost = false;
bool sbusFailsafe  = false;

unsigned long lastSignalTime = 0;
const unsigned long SIGNAL_TIMEOUT = 150;

// ===== Section: BLV Modbus (TX-only) =====
#define MB_SERIAL Serial2
#define MB_BAUD 230400
#define SLAVE_1 0x01
#define SLAVE_2 0x02

// ===== Section: Parameters =====
int max_speed = 3000;
int deadband_percent = 5;
int max_speed_rotary = 100;

bool monitorMode = false;

const float TH_DB = 8.0f;
const float ST_DB = 8.0f;

const int32_t ACC_NORM = 50;
const int32_t DEC_RUN  = 50;

// HARD BRAKE
const int32_t ACC_BRAKE = 10;
const int32_t DEC_BRAKE = 10;

bool wasNeutral = false;

static bool son1 = false;
static bool son2 = false;

// ===== Section: Helpers =====
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ===== Limit Switch Helpers =====
static inline bool limitPressedRaw(int pin) {
  int v = digitalRead(pin);
  return LIMIT_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

static inline bool limitRightPressed() { return limitPressedRaw(limit_right); }
static inline bool limitLeftPressed()  { return limitPressedRaw(limit_left);  }

// ===== Section: Modbus CRC16 =====
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
  buf[lenNoCrc]     = crc & 0xFF;
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

// ===== Section: BLV Commands =====
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
  const uint16_t STOP_BIT = (1u << 5);  // 0x0020
  uint16_t keep = sonBit(slaveId);      // 0x0001 jika S-ON ON
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
  put_i32_be(d + 0,  0x00000030);
  put_i32_be(d + 4,  0);
  put_i32_be(d + 8,  rpm);
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

// ===== SBUS Percent Convert =====
float sbusToPercent(int value, int minPulse = 172, int midPulse = 992, int maxPulse = 1811) {
  float p;

  if (value == midPulse) {
    p = 0.0f;
  } else if (value > midPulse) {
    p = ((float)(value - midPulse) / (float)(maxPulse - midPulse)) * 100.0f;
  } else {
    p = -((float)(midPulse - value) / (float)(midPulse - minPulse)) * 100.0f;
  }

  return clampf(p, -100.0f, 100.0f);
}

// ===== Update SBUS via Library =====
void updateSbusFromLibrary() {
  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.data();

    lastSignalTime = millis();
    sbusFrameLost = sbus_data.lost_frame;
    sbusFailsafe  = sbus_data.failsafe;

    for (int i = 0; i < 16; i++) {
      percent[i] = sbusToPercent(sbus_data.ch[i]);
    }

    // Debug RC (rate-limited)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      Serial.print("RC[%] ");
      for (int i = 0; i < 16; i++) {
        Serial.print("CH");
        Serial.print(i + 1);
        Serial.print(":");
        Serial.print(percent[i], 1);
        if (i < 15) Serial.print(" ");
      }
      Serial.print(" | FS:");
      Serial.print(sbusFailsafe);
      Serial.print(" LF:");
      Serial.print(sbusFrameLost);
      Serial.println();
    }
  }
}

// ===== Limit Debug (optional) =====
void debugLimitSwitches() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print("LIMIT L:");
    Serial.print(limitLeftPressed() ? "PRESSED" : "OPEN");
    Serial.print(" R:");
    Serial.println(limitRightPressed() ? "PRESSED" : "OPEN");
  }
}

// ===== Section: Local Rotary / Linear / Polisher =====
void move_right(int speed) {
  digitalWrite(mRightFWD, HIGH);
  digitalWrite(mRightREV, LOW);
  analogWrite(mmRPin, speed);
}
void move_left(int speed) {
  digitalWrite(mRightFWD, LOW);
  digitalWrite(mRightREV, HIGH);
  analogWrite(mmRPin, speed);
}
void stop_rotary() {
  digitalWrite(mRightFWD, LOW);
  digitalWrite(mRightREV, LOW);
  analogWrite(mmRPin, 0);
}

void rotary_controller() {
  bool limR = limitRightPressed();
  bool limL = limitLeftPressed();

  int speed_rotary = abs((int)(percent[0] / 100.0f * max_speed_rotary));

  if (percent[0] > 30) {
    if (limR) stop_rotary();
    else move_right(speed_rotary);
  }

  else if (percent[0] < -30) {
    if (limL) stop_rotary();
    else move_left(speed_rotary);
  }
  else {
    stop_rotary();
  }
}

void linear_actuator_controller() {
  // NOTE: Aku pakai limit_left sebagai batas mundur dan limit_right sebagai batas maju.
  // Kalau di hardware kamu beda, tinggal swap (tukar) limFwd/limBwd.
  bool limFwd = limitRightPressed();
  bool limBwd = limitLeftPressed();

  if (percent[2] < -30) {
    // gerak "forward" (sesuai kode awalmu)
    if (limFwd) {
      digitalWrite(linact_fwd_pin, LOW);
      digitalWrite(linact_bwd_pin, LOW);
    } else {
      digitalWrite(linact_fwd_pin, HIGH);
      digitalWrite(linact_bwd_pin, LOW);
    }
  } else if (percent[2] > 30) {
    // gerak "backward"
    if (limBwd) {
      digitalWrite(linact_fwd_pin, LOW);
      digitalWrite(linact_bwd_pin, LOW);
    } else {
      digitalWrite(linact_bwd_pin, HIGH);
      digitalWrite(linact_fwd_pin, LOW);
    }
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

void polisherControl() {
  if (percent[4] > 52) polish_start();
  else if (percent[4] < -50) polish_stop();
}

// ===== Section: Drive Mixing =====
int32_t rpmFromPercent(float p) {
  if (fabs(p) < deadband_percent) return 0;
  p = clampf(p, -100.0f, 100.0f);
  return (int32_t)(p / 100.0f * max_speed);
}

void drive_blv_from_rc() {
  float throttle = percent[1];
  float steer = -percent[3];

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

  static float last_throttle = 0;
  static float last_steer = 0;
  const float MOVEMENT_THRESHOLD = 1.5f;

  if (fabs(throttle - last_throttle) < MOVEMENT_THRESHOLD &&
      fabs(steer - last_steer) < MOVEMENT_THRESHOLD) {
    return;
  }
  last_throttle = throttle;
  last_steer = steer;

  if (fabs(throttle) < TH_DB && fabs(steer) > ST_DB) {
    float spin = steer;
    float left = spin;
    float right = -spin;

    left  = left  * left_motor_trim;
    right = right * right_motor_trim;

    left  = clampf(left, -100.0f, 100.0f);
    right = clampf(right, -100.0f, 100.0f);

    int32_t leftRpm  = rpmFromPercent(left);
    int32_t rightRpm = rpmFromPercent(right);

    leftRpm = -leftRpm;

    direct_velocity(SLAVE_1, leftRpm, ACC_NORM, DEC_RUN);
    delay(3);
    direct_velocity(SLAVE_2, rightRpm, ACC_NORM, DEC_RUN);
    return;
  }

  if (throttle < 0) steer = -steer;

  float left  = throttle - steer;
  float right = throttle + steer;

  left  = left  * left_motor_trim;
  right = right * right_motor_trim;

  left  = clampf(left, -100.0f, 100.0f);
  right = clampf(right, -100.0f, 100.0f);

  int32_t leftRpm  = rpmFromPercent(left);
  int32_t rightRpm = rpmFromPercent(right);

  leftRpm = -leftRpm;

  direct_velocity(SLAVE_1, leftRpm, ACC_NORM, DEC_RUN);
  delay(3);
  direct_velocity(SLAVE_2, rightRpm, ACC_NORM, DEC_RUN);
}

// ===== Section: Safety / Serial Commands =====
void stop_all_outputs_safe() {
  brake_blv_both_now();
  polish_stop();
  digitalWrite(linact_fwd_pin, LOW);
  digitalWrite(linact_bwd_pin, LOW);
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

  digitalWrite(emergency_pin, emergencyState ? LOW : HIGH);
}

// ===== Section: Main =====
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

  // ===== Limit switch pins =====
  pinMode(limit_right, INPUT_PULLUP);
  pinMode(limit_left, INPUT_PULLUP);

  digitalWrite(mRightFWD, LOW);
  digitalWrite(mRightREV, LOW);
  digitalWrite(stopModeR, LOW);
  digitalWrite(m0R, HIGH);
  digitalWrite(mbFreeR, HIGH);
  digitalWrite(mmRPin, LOW);

  digitalWrite(linact_fwd_pin, LOW);
  digitalWrite(linact_bwd_pin, LOW);
  polish_stop();

  set_son(SLAVE_1, true);
  delay(200);
  set_son(SLAVE_2, true);
  delay(300);

  brake_blv_both_now();
}

void loop() {
  updateSbusFromLibrary();

  // Optional debug limit switch:
  debugLimitSwitches();

  handleSerialCommands();

  if (millis() - lastSignalTime > SIGNAL_TIMEOUT || sbusFailsafe) {
    stop_all_outputs_safe();
    digitalWrite(emergency_pin, LOW);
    delay(50);
    return;
  }

  emergencyChecker();

  if (monitorMode) {
    stop_all_outputs_safe();
    delay(20);
    return;
  }

  linear_actuator_controller();
  // rotary_controller();
  polisherControl();

  static unsigned long lastDrive = 0;
  const unsigned long DRIVE_PERIOD_MS = 40;
  if (millis() - lastDrive >= DRIVE_PERIOD_MS) {
    lastDrive = millis();
    drive_blv_from_rc();
  }
}
