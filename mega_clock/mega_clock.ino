// Mega clock controller - 3 clocks (6 hands) per Mega, commanded by the ESP32 over I2C.
// Upload the same sketch to both Megas, changing only I2C_ADDRESS.

#include <Wire.h>

// ================= BOARD =================
// 0x10 for Mega A, 0x11 for Mega B
const uint8_t I2C_ADDRESS = 0x10;

// ================= PINS =================
// STEP: motors 0-5 on PORTA bits 0-5 = D22-D27
// DIR:  motors 0-5 on PORTF bits 0-5 = A0-A5
// Clock c uses motor 2c (hand 1) and motor 2c+1 (hand 2)
const int NUM_CLOCKS = 3;
const int NUM_MOTORS = NUM_CLOCKS * 2;
const uint8_t MOTOR_MASK = (1 << NUM_MOTORS) - 1;

const int SENSOR_PINS[NUM_CLOCKS] = {2, 3, 4};
const int DETECTED = HIGH;          // sensor reads 1 when blocked

const int EN_PIN = 10;              // shared by all drivers - LOW = on, HIGH = off

// ================= MOTOR SETUP =================
const long STEPS_PER_REV = 180L * 64;    // steps per turn of the hand (11,520)

// Real angles past the sensor trigger point, matching the test rig calibration
const float M1_DEGREES = 300.0;     // hand 1 position after homing
const float M2_DEGREES = 28.0;      // hand 2 position after homing

const long M1_STEPS = (long)(STEPS_PER_REV * M1_DEGREES / 360.0 + 0.5);
const long M2_STEPS = (long)(STEPS_PER_REV * M2_DEGREES / 360.0 + 0.5);

// ================= TIMING =================
const unsigned long TICK_US = 200;  // each motor steps at most once per tick (smaller = faster)

const long HOME_MAX_STEPS = STEPS_PER_REV * 2;   // give up after 2 turns
const long CLEAR_MARGIN   = 100;                 // extra steps past the sensor when clearing
const long CHECK_STEPS    = STEPS_PER_REV / 4;   // 90° - if still blocked after this, it's the other hand

// ================= I2C COMMANDS =================
const uint8_t CMD_HOME   = 0x01;    // [cmd]
const uint8_t CMD_MOVE   = 0x02;    // [cmd, motor, angleHi, angleLo, dir]  angle in 0.1°, dir 0 = CW, 1 = CCW
const uint8_t CMD_ENABLE = 0x03;    // [cmd, 0 = off / 1 = on]

// ================= STATE =================
long pos[NUM_MOTORS];               // steps from the sensor trigger point (valid once homed)
long remaining[NUM_MOTORS];         // steps left in the current move
bool cw[NUM_MOTORS];

enum HomeState : uint8_t {
  H_NONE,       // never homed
  H_CLEAR_A,    // moving hand 1 off the sensor
  H_CLEAR_B,    // still blocked, moving hand 2 off
  H_SEEK_A,     // hand 1 running to the sensor
  H_LEAVE_B,    // making sure hand 2 starts clear of the sensor
  H_SEEK_B,     // hand 2 running to the sensor
  H_FINISH,     // waiting for hand 2's final move
  H_DONE,
  H_ERROR
};
HomeState homeState[NUM_CLOCKS];
long homeCount[NUM_CLOCKS];

// I2C buffers - filled in interrupts, handled in loop()
volatile uint8_t rxBuf[8];
volatile uint8_t rxLen = 0;
volatile bool rxReady = false;
volatile uint8_t statusBuf[3];      // [busy, homed mask, error mask]

unsigned long lastTick = 0;

// ================= MOTION =================
void startMove(int m, long steps, bool dirCW) {
  cw[m] = dirCW;
  if (dirCW) PORTF |= _BV(m);
  else       PORTF &= ~_BV(m);
  remaining[m] = steps;
}

void setEnabled(bool on) {
  digitalWrite(EN_PIN, on ? LOW : HIGH);
  if (on) delay(5);                 // let the drivers power up the coils
}

// ================= HOMING =================
void nextState(int c, HomeState s) {
  homeState[c] = s;
  homeCount[c] = 0;
}

void fail(int c, const char *msg) {
  homeState[c] = H_ERROR;
  Serial.print("Clock ");
  Serial.print(c);
  Serial.print(" ERROR: ");
  Serial.println(msg);
}

// Advances one clock's homing by at most one step. Clocks home in parallel.
void homingTick(int c) {
  int a = c * 2;
  int b = c * 2 + 1;
  HomeState st = homeState[c];

  if (st == H_NONE || st == H_DONE || st == H_ERROR) return;
  if (remaining[a] > 0 || remaining[b] > 0) return;   // let the current move finish

  bool det = digitalRead(SENSOR_PINS[c]) == DETECTED;
  long &cnt = homeCount[c];

  switch (st) {
    case H_CLEAR_A:
      if (!det) { startMove(a, CLEAR_MARGIN, true); nextState(c, H_SEEK_A); }
      else if (++cnt <= CHECK_STEPS) startMove(a, 1, true);
      else nextState(c, H_CLEAR_B);
      break;

    case H_CLEAR_B:
      if (!det) { startMove(b, CLEAR_MARGIN, true); nextState(c, H_SEEK_A); }
      else if (++cnt <= HOME_MAX_STEPS) startMove(b, 1, true);
      else fail(c, "sensor still blocked");
      break;

    case H_SEEK_A:
      if (det) { pos[a] = 0; startMove(a, M1_STEPS, true); nextState(c, H_LEAVE_B); }
      else if (++cnt <= HOME_MAX_STEPS) startMove(a, 1, true);
      else fail(c, "hand 1 didn't reach the sensor");
      break;

    case H_LEAVE_B:
      if (!det) nextState(c, H_SEEK_B);
      else if (++cnt <= HOME_MAX_STEPS) startMove(b, 1, true);
      else fail(c, "hand 2 stuck on the sensor");
      break;

    case H_SEEK_B:
      if (det) { pos[b] = 0; startMove(b, M2_STEPS, true); nextState(c, H_FINISH); }
      else if (++cnt <= HOME_MAX_STEPS) startMove(b, 1, true);
      else fail(c, "hand 2 didn't reach the sensor");
      break;

    case H_FINISH:
      nextState(c, H_DONE);
      Serial.print("Clock ");
      Serial.print(c);
      Serial.println(" homed");
      break;

    default:
      break;
  }
}

// ================= TICK =================
// Steps every motor that has a move in progress, all at the same instant.
void tick() {
  for (int c = 0; c < NUM_CLOCKS; c++) homingTick(c);

  uint8_t mask = 0;
  for (int m = 0; m < NUM_MOTORS; m++) {
    if (remaining[m] > 0) {
      mask |= _BV(m);
      remaining[m]--;
      pos[m] += cw[m] ? 1 : -1;
      if (pos[m] >= STEPS_PER_REV) pos[m] -= STEPS_PER_REV;
      if (pos[m] < 0) pos[m] += STEPS_PER_REV;
    }
  }

  if (mask) {
    PORTA |= mask;
    delayMicroseconds(2);
    PORTA &= ~mask;
  }
}

void updateStatus() {
  uint8_t busy = 0, homed = 0, errors = 0;
  for (int c = 0; c < NUM_CLOCKS; c++) {
    HomeState st = homeState[c];
    if (st == H_DONE) homed |= _BV(c);
    else if (st == H_ERROR) errors |= _BV(c);
    else if (st != H_NONE) busy = 1;
  }
  for (int m = 0; m < NUM_MOTORS; m++) {
    if (remaining[m] > 0) busy = 1;
  }

  noInterrupts();
  statusBuf[0] = busy;
  statusBuf[1] = homed;
  statusBuf[2] = errors;
  interrupts();
}

// ================= COMMANDS =================
void handleCommand(const uint8_t *buf, uint8_t len) {
  if (len == 0) return;

  switch (buf[0]) {
    case CMD_HOME:
      Serial.println("Homing all clocks");
      setEnabled(true);
      for (int c = 0; c < NUM_CLOCKS; c++) nextState(c, H_CLEAR_A);
      break;

    case CMD_MOVE: {
      if (len < 5) return;
      uint8_t m = buf[1];
      uint16_t angle = ((uint16_t)buf[2] << 8) | buf[3];   // 0.1° units
      bool dirCW = buf[4] == 0;

      if (m >= NUM_MOTORS || angle >= 3600) return;
      if (homeState[m / 2] != H_DONE) { Serial.println("Move ignored - clock not homed"); return; }
      if (remaining[m] > 0) { Serial.println("Move ignored - hand busy"); return; }

      long target = (long)angle * STEPS_PER_REV / 3600;
      long delta = dirCW ? (target - pos[m] + STEPS_PER_REV) % STEPS_PER_REV
                         : (pos[m] - target + STEPS_PER_REV) % STEPS_PER_REV;
      startMove(m, delta, dirCW);

      Serial.print("Motor ");
      Serial.print(m);
      Serial.print(" -> ");
      Serial.print(angle / 10.0, 1);
      Serial.println(dirCW ? " deg CW" : " deg CCW");
      break;
    }

    case CMD_ENABLE:
      if (len < 2) return;
      setEnabled(buf[1] != 0);
      Serial.println(buf[1] ? "Motors on" : "Motors off");
      break;
  }
}

// ================= I2C CALLBACKS (run in interrupts - keep short) =================
void onReceive(int count) {
  uint8_t i = 0;
  while (Wire.available()) {
    uint8_t v = Wire.read();
    if (i < sizeof(rxBuf)) rxBuf[i++] = v;
  }
  rxLen = i;
  rxReady = true;
}

void onRequest() {
  uint8_t out[3] = {statusBuf[0], statusBuf[1], statusBuf[2]};
  Wire.write(out, 3);
}

// ================= SETUP =================
void setup() {
  DDRA  |= MOTOR_MASK;              // STEP pins as outputs
  PORTA &= ~MOTOR_MASK;
  DDRF  |= MOTOR_MASK;              // DIR pins as outputs

  pinMode(EN_PIN, OUTPUT);
  setEnabled(false);                // motors off until told otherwise

  for (int c = 0; c < NUM_CLOCKS; c++) pinMode(SENSOR_PINS[c], INPUT);

  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  Serial.print("Mega ready at I2C address 0x");
  Serial.println(I2C_ADDRESS, HEX);
}

// ================= LOOP =================
void loop() {
  if (rxReady) {
    uint8_t buf[sizeof(rxBuf)];
    uint8_t len;
    noInterrupts();
    len = rxLen;
    for (uint8_t i = 0; i < len; i++) buf[i] = rxBuf[i];
    rxReady = false;
    interrupts();
    handleCommand(buf, len);
  }

  unsigned long now = micros();
  if (now - lastTick >= TICK_US) {
    lastTick = now;
    tick();
    updateStatus();
  }
}
