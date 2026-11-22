// 24-hand clock controller - one Mega, 12 clocks in a 4 x 3 grid, 2 hands each.
//
//    1   2   3   4        Clock c has hand 2c-1 (top/front) and hand 2c
//    5   6   7   8        (bottom/back), and one IR sensor shared by both.
//    9  10  11  12
//
// CALIBRATE (once, and after any hardware change): type f to free the motors
// (or use j to jog), set every top (odd) hand pointing UP and every bottom
// (even) hand pointing DOWN, then type c. The results are saved in EEPROM, so
// they survive power-off and re-uploading - no values to paste.
//
// HOME: at power-on (once calibrated) and with h. Each hand runs CW to its
// sensor, then moves its calibrated angle, ending pointing up.
//
// Serial Monitor: 9600 baud, line ending "Newline". Commands:
//   h            home all hands
//   c            calibrate (top hands up, bottom hands down first)
//   d            show the stored calibration
//   n 5 -1.5     nudge hand 5's home position by -1.5° (saved); then h to check
//   j 5 10       jog hand 5 by 10° (- = CCW) - works before homing
//   w            wiring check: turns hands 1-24 90° CW one at a time, printing pins,
//                then a min/max voltage summary for every sensor
//   w 9          wiring check starting from hand 9
//   w 1 360      full turn per hand, so every hand passes its sensor
//   v            fast sensor sweep (~3 s): all top hands turn once together, then all
//                bottom hands, with background and peak voltage for every sensor
//   x            crosstalk test: each hand turns once alone; reports any other
//                clock's sensor that fires (~35 s)
//   t            test every hand in turn: should turn CW from UP to RIGHT
//   t 5          test hand 5 only
//   s            sensor monitor (any key to stop)
//   b            box around the edge of the grid
//   u            all hands up
//   0 - 9        show that digit (top half on this Mega)
//   p            start / pause the continuous pattern
//   f            free the motors so hands can be turned by hand
// Angles: 0 = up, 90 = right, 180 = down, 270 = left.

#include <EEPROM.h>

// ================= PINS =================
const int EN_PIN = 19;              // shared by all drivers - LOW = on, HIGH = off

const int NUM_PAIRS = 12;           // clocks
const int NUM_HANDS = NUM_PAIRS * 2;

// Odd hands are the top (front) hand on each shaft
//                    clock:   1       2       3       4       5       6       7       8       9      10      11      12
// Test wiring: clocks 5<->6 and 7<->8 are swapped on the cable, and on clocks
// 5-8 the top and bottom hands are swapped, so their pins are rearranged here
const uint8_t STEP_PINS[NUM_HANDS] = {  3,  5,   7,  9,  11, 13,  15, 17,  31, 33,  35, 37,  23, 25,  27, 29,  53, 51,  49, 47,  45, 43,  41, 39 };
const uint8_t DIR_PINS[NUM_HANDS]  = {  2,  4,   6,  8,  10, 12,  14, 16,  30, 32,  34, 36,  22, 24,  26, 28,  52, 50,  48, 46,  44, 42,  40, 38 };
const uint8_t SENSOR_PINS[NUM_PAIRS] = { A0, A1, A2, A3, A4, A5, A8, A9, A10, A11, A12, A13 };

const int DETECTED = HIGH;          // sensor reads 1 when a hand reflects
const int CW = LOW;                 // DIR level that turns the hands clockwise from the front

// ================= MOTOR SETUP =================
const long STEPS_PER_REV = 180L * 64;    // steps per turn of the hand (11,520)

// Where each hand is set by hand before calibrating: top hands up, bottom
// hands down, so each clock's pair forms one straight vertical line.
const float CAL_START_TOP    = 0;
const float CAL_START_BOTTOM = 180;

// ================= SHAPES =================
// Box around the edge of the grid. Top hands draw the top and bottom edges'
// joins on one layer and bottom hands on the other, so every line meets a
// hand at the same height. Clocks 6 and 7 are inside the box and park at
// 7:30 (225°).
//                      clock:    1        2        3        4        5        6        7        8        9       10       11       12
const float BOX_DEGREES[NUM_HANDS] = { 90,180, 270, 90,  90,270, 270,180, 180,  0, 225,225, 225,225, 180,  0,   0, 90,  90,270, 270, 90,   0,270 };
const float UP_DEGREES[NUM_HANDS]  = { 0 };

// Digits: this Mega's 12 clocks draw the top half of a digit (the other Mega
// draws the bottom half). One row per digit, two angles (top, bottom hand) per clock.
//                           clock:    1        2        3        4        5        6        7        8        9       10       11       12
const int16_t DIGITS[10][NUM_HANDS] PROGMEM = {     // stored in flash, not RAM
  /* 0 */                      { 90,180, 270, 90,  90,270, 270,180, 180,  0,  90,180, 270,180, 180,  0,   0,180, 180,  0, 180,  0,   0,180 },
  /* 1 */                      { 0 },
  /* 2 */                      { 0 },
  /* 3 */                      { 0 },
  /* 4 */                      { 0 },
  /* 5 */                      { 0 },
  /* 6 */                      { 0 },
  /* 7 */                      { 0 },
  /* 8 */                      { 0 },
  /* 9 */                      { 0 },
};
const bool DIGIT_DEFINED[10] = { true, false, false, false, false, false, false, false, false, false };

// Continuous pattern: every step turns each hand 45° (+ = CW, - = CCW).
// Top hands CW, bottom hands CCW, so each clock's hands scissor open and shut.
//                      clock:    1        2        3        4        5        6        7        8        9       10       11       12
const float SPIN_MOVE[NUM_HANDS]   = { 45,-45,  45,-45,  45,-45,  45,-45,  45,-45,  45,-45,  45,-45,  45,-45,  45,-45,  45,-45,  45,-45,  45,-45 };

// ================= TIMING =================
// Homing and calibration must use the same speed, or the sensor triggers at a
// slightly different point and the stored calibration no longer matches.
const unsigned int STEP_DELAY_US = 50;            // homing + calibration, same as the sensor sweep (smaller = faster)
const unsigned long TICK_US = STEP_DELAY_US * 2UL;

const unsigned int PATTERN_STEP_DELAY_US = 200;   // shapes, pattern, jog and test
const unsigned long PATTERN_TICK_US = PATTERN_STEP_DELAY_US * 2UL;

const long HOME_MAX_STEPS = STEPS_PER_REV * 2;   // give up after 2 turns
const long CLEAR_MARGIN   = 100;                 // extra steps past the sensor when clearing
const long CHECK_STEPS    = STEPS_PER_REV / 4;   // 90° - if still blocked after this, it's the other hand

// true = keep the motors powered after moves so hands can't drift
const bool HOLD_POSITION = true;

// ================= STATE =================
long remaining[NUM_HANDS];          // steps left in each hand's current homing move
long calSteps[NUM_HANDS];           // calibration: steps from the start position to the trigger
long handPos[NUM_HANDS];            // steps CW from "up" - valid once homed
bool homed = false;
long moved[NUM_HANDS];              // homing: steps each hand has turned so far
long triggerAt[NUM_HANDS];          // homing: value of moved[] when the sensor saw the hand

enum PairState : uint8_t {
  // homing
  H_CLEAR_A,    // moving the top hand off the sensor
  H_CLEAR_B,    // still blocked, so moving the bottom hand off
  H_SEEK_A,     // top hand running to the sensor
  H_LEAVE_B,    // making sure the bottom hand starts clear of the sensor
  H_SEEK_B,     // bottom hand running to the sensor
  // calibration
  C_LEAVE_A,    // top hand moving off the sensor (if it starts on it)
  C_SEEK_A,     // top hand running to the sensor, counting steps
  C_LEAVE_B,
  C_SEEK_B,
  // both
  S_FINISH,     // waiting for the last move to end
  S_DONE,
  S_ERROR
};
PairState pairState[NUM_PAIRS];
long pairCount[NUM_PAIRS];

// ================= FAST PIN ACCESS =================
// digitalWrite() takes ~4 µs; with 24 hands that's longer than a step tick.
// These write the port registers directly (well under 1 µs each).
volatile uint8_t *stepPort[NUM_HANDS];
volatile uint8_t *dirPort[NUM_HANDS];
volatile uint8_t *sensorPort[NUM_PAIRS];
uint8_t stepMask[NUM_HANDS], dirMask[NUM_HANDS], sensorMask[NUM_PAIRS];

void setupPins() {
  for (int m = 0; m < NUM_HANDS; m++) {
    pinMode(STEP_PINS[m], OUTPUT);
    pinMode(DIR_PINS[m], OUTPUT);
    stepPort[m] = portOutputRegister(digitalPinToPort(STEP_PINS[m]));
    stepMask[m] = digitalPinToBitMask(STEP_PINS[m]);
    dirPort[m]  = portOutputRegister(digitalPinToPort(DIR_PINS[m]));
    dirMask[m]  = digitalPinToBitMask(DIR_PINS[m]);
  }
  for (int p = 0; p < NUM_PAIRS; p++) {
    pinMode(SENSOR_PINS[p], INPUT);
    sensorPort[p] = portInputRegister(digitalPinToPort(SENSOR_PINS[p]));
    sensorMask[p] = digitalPinToBitMask(SENSOR_PINS[p]);
  }
}

inline void stepHigh(int m) { *stepPort[m] |= stepMask[m]; }
inline void stepLow(int m)  { *stepPort[m] &= ~stepMask[m]; }

inline void setDirection(int m, bool cw) {
  bool high = cw ? (CW == HIGH) : (CW != HIGH);
  if (high) *dirPort[m] |= dirMask[m];
  else      *dirPort[m] &= ~dirMask[m];
}

inline bool sensorDetected(int p) {
  bool high = (*sensorPort[p] & sensorMask[p]) != 0;
  return high == (DETECTED == HIGH);
}

// ================= ENABLE =================
bool steppersOn = false;

void enableSteppers() {
  if (steppersOn) return;           // already on - no delay, so back-to-back moves run smoothly
  digitalWrite(EN_PIN, LOW);
  delay(5);                         // let the drivers power up the coils
  steppersOn = true;
}

void disableSteppers() {
  digitalWrite(EN_PIN, HIGH);
  steppersOn = false;
}

long degreesToSteps(float degrees) {
  return (long)(STEPS_PER_REV * degrees / 360.0 + 0.5);
}

bool isTopHand(int m) { return m % 2 == 0; }

void printHand(int m) {
  Serial.print("Hand ");
  Serial.print(m + 1);
  Serial.print(" (clock ");
  Serial.print(m / 2 + 1);
  Serial.print(isTopHand(m) ? ", top)" : ", bottom)");
}

// ================= CALIBRATION STORAGE (EEPROM) =================
// Angle each hand moves past its sensor trigger point to end up pointing up.
float homeDegrees[NUM_HANDS];
bool calibrated = false;

const uint32_t CAL_MAGIC = 0xC10C2401;   // changes if the stored layout changes
struct CalData {
  uint32_t magic;
  float homeDegrees[NUM_HANDS];
};

void loadCalibration() {
  CalData d;
  EEPROM.get(0, d);
  calibrated = d.magic == CAL_MAGIC;
  for (int m = 0; m < NUM_HANDS; m++) homeDegrees[m] = calibrated ? d.homeDegrees[m] : 0;
}

void saveCalibration() {
  CalData d;
  d.magic = CAL_MAGIC;
  for (int m = 0; m < NUM_HANDS; m++) d.homeDegrees[m] = homeDegrees[m];
  EEPROM.put(0, d);
  calibrated = true;
}

void printCalibration() {
  if (!calibrated) {
    Serial.println("Not calibrated yet - type c");
    return;
  }
  Serial.println("Stored calibration (degrees past the sensor trigger to reach up):");
  for (int m = 0; m < NUM_HANDS; m++) {
    printHand(m);
    Serial.print(": ");
    Serial.println(homeDegrees[m], 2);
  }
}

// ================= PAIR STATE MACHINE =================
void nextState(int p, PairState s) {
  pairState[p] = s;
  pairCount[p] = 0;
}

void fail(int p, const char *msg) {
  pairState[p] = S_ERROR;
  Serial.print("Clock ");
  Serial.print(p + 1);
  Serial.print(" ERROR: ");
  Serial.println(msg);
}

// Advances one clock by at most one step
void pairTick(int p) {
  int a = p * 2;                    // top hand
  int b = p * 2 + 1;                // bottom hand
  PairState st = pairState[p];

  if (st == S_DONE || st == S_ERROR) return;
  if (remaining[a] > 0 || remaining[b] > 0) return;   // let the current move finish

  bool det = sensorDetected(p);
  long &cnt = pairCount[p];

  switch (st) {
    // ---------- homing ----------
    case H_CLEAR_A:
      if (!det) { remaining[a] = CLEAR_MARGIN; nextState(p, H_SEEK_A); }
      else if (++cnt <= CHECK_STEPS) remaining[a] = 1;
      else nextState(p, H_CLEAR_B);
      break;

    case H_CLEAR_B:
      if (!det) { remaining[b] = CLEAR_MARGIN; nextState(p, H_SEEK_A); }
      else if (++cnt <= HOME_MAX_STEPS) remaining[b] = 1;
      else fail(p, "sensor still blocked - check for obstruction or bright light");
      break;

    case H_SEEK_A:
      if (det) { triggerAt[a] = moved[a]; remaining[a] = degreesToSteps(homeDegrees[a]); nextState(p, H_LEAVE_B); }
      else if (++cnt <= HOME_MAX_STEPS) remaining[a] = 1;
      else fail(p, "top hand didn't reach the sensor");
      break;

    case H_LEAVE_B:
      if (!det) nextState(p, H_SEEK_B);
      else if (++cnt <= HOME_MAX_STEPS) remaining[b] = 1;
      else fail(p, "bottom hand stuck on the sensor");
      break;

    case H_SEEK_B:
      if (det) { triggerAt[b] = moved[b]; remaining[b] = degreesToSteps(homeDegrees[b]); nextState(p, S_FINISH); }
      else if (++cnt <= HOME_MAX_STEPS) remaining[b] = 1;
      else fail(p, "bottom hand didn't reach the sensor");
      break;

    // ---------- calibration (hands start at their CAL_START positions) ----------
    case C_LEAVE_A:
      if (!det) nextState(p, C_SEEK_A);
      else if (++calSteps[a] <= STEPS_PER_REV) remaining[a] = 1;
      else fail(p, "top hand stuck on the sensor");
      break;

    case C_SEEK_A:
      if (det) {
        remaining[a] = STEPS_PER_REV - calSteps[a];   // finish the turn, back to the start
        nextState(p, C_LEAVE_B);
      }
      else if (++calSteps[a] < STEPS_PER_REV) remaining[a] = 1;
      else fail(p, "top hand didn't reach the sensor in one turn");
      break;

    case C_LEAVE_B:
      if (!det) nextState(p, C_SEEK_B);
      else if (++calSteps[b] <= STEPS_PER_REV) remaining[b] = 1;
      else fail(p, "bottom hand stuck on the sensor");
      break;

    case C_SEEK_B:
      if (det) {
        remaining[b] = STEPS_PER_REV - calSteps[b];
        nextState(p, S_FINISH);
      }
      else if (++calSteps[b] < STEPS_PER_REV) remaining[b] = 1;
      else fail(p, "bottom hand didn't reach the sensor in one turn");
      break;

    case S_FINISH:
      nextState(p, S_DONE);
      break;

    default:
      break;
  }
}

// Steps every hand that has a homing move in progress, all at the same time
void stepTick() {
  bool stepped[NUM_HANDS];
  for (int m = 0; m < NUM_HANDS; m++) {
    stepped[m] = remaining[m] > 0;
    if (stepped[m]) {
      stepHigh(m);
      remaining[m]--;
      moved[m]++;
    }
  }
  delayMicroseconds(3);
  for (int m = 0; m < NUM_HANDS; m++) {
    if (stepped[m]) stepLow(m);
  }
}

// Runs every clock from firstState until all are done or failed.
// Returns true if every clock finished without an error.
bool runAllPairs(PairState firstState) {
  enableSteppers();

  // Always turn clockwise so each hand approaches its sensor from the same side
  for (int m = 0; m < NUM_HANDS; m++) {
    setDirection(m, true);
    remaining[m] = 0;
    calSteps[m] = 0;
    moved[m] = 0;
    triggerAt[m] = -1;
  }
  delayMicroseconds(10);
  for (int p = 0; p < NUM_PAIRS; p++) nextState(p, firstState);

  unsigned long lastTick = micros();
  while (true) {
    bool finished = true;
    for (int p = 0; p < NUM_PAIRS; p++) {
      if (pairState[p] != S_DONE && pairState[p] != S_ERROR) finished = false;
    }
    if (finished) break;

    while (micros() - lastTick < TICK_US) {}
    lastTick = micros();

    for (int p = 0; p < NUM_PAIRS; p++) pairTick(p);
    stepTick();
  }

  bool ok = true;
  for (int p = 0; p < NUM_PAIRS; p++) {
    if (pairState[p] == S_ERROR) ok = false;
  }
  if (!ok || !HOLD_POSITION) disableSteppers();
  return ok;
}

// ================= HOMING =================
// When we already knew where the hands were, checks that each sensor fired at
// the angle calibration found. A big difference means the sensor saw something
// other than the hand tip (the other hand, a hand's tail, glare...).
void printHomingCheck(const long startPos[]) {
  Serial.println("Homing check (angle each hand was at when its sensor fired):");
  Serial.println("Hand  Clock  Expected  Actual   Diff");
  int bad = 0;
  for (int m = 0; m < NUM_HANDS; m++) {
    long expected = (STEPS_PER_REV - degreesToSteps(homeDegrees[m])) % STEPS_PER_REV;
    Serial.print(m + 1 < 10 ? "  " : " ");
    Serial.print(m + 1);
    Serial.print(isTopHand(m) ? "T" : "B");
    Serial.print("   ");
    Serial.print(m / 2 + 1 < 10 ? " " : "");
    Serial.print(m / 2 + 1);
    Serial.print("    ");
    Serial.print(expected * 360.0 / STEPS_PER_REV, 1);
    Serial.print("    ");
    if (triggerAt[m] < 0) {
      Serial.println("  -   (not reached)");
      continue;
    }
    long actual = (startPos[m] + triggerAt[m]) % STEPS_PER_REV;
    long diff = (actual - expected + STEPS_PER_REV + STEPS_PER_REV / 2) % STEPS_PER_REV - STEPS_PER_REV / 2;
    float diffDeg = diff * 360.0 / STEPS_PER_REV;
    Serial.print(actual * 360.0 / STEPS_PER_REV, 1);
    Serial.print("    ");
    Serial.print(diffDeg, 1);
    if (fabs(diffDeg) > 2.0) { Serial.print("   <- WRONG"); bad++; }
    Serial.println();
  }
  if (bad == 0) Serial.println("All sensors fired where expected.");
}

// Homes all hands from wherever they are, ending pointing up. Safe to call any time.
bool homeAll() {
  if (!calibrated) {
    Serial.println("Not calibrated yet - set top hands up, bottom hands down, then type c");
    return false;
  }
  Serial.println("Homing all hands...");
  bool wasHomed = homed;
  long startPos[NUM_HANDS];
  for (int m = 0; m < NUM_HANDS; m++) startPos[m] = handPos[m];

  bool ok = runAllPairs(H_CLEAR_A);
  Serial.println(ok ? "Homing done - all hands up" : "Homing finished with errors - motors off");
  if (wasHomed) printHomingCheck(startPos);

  homed = ok;
  if (ok) {
    for (int m = 0; m < NUM_HANDS; m++) handPos[m] = 0;
  }
  return ok;
}

// ================= CALIBRATION =================
// Top hands must be pointing up and bottom hands down before calling this.
bool calibrateAll() {
  Serial.println("Calibrating - top hands up, bottom hands down...");
  bool ok = runAllPairs(C_LEAVE_A);

  if (!ok) {
    Serial.println("Calibration failed - nothing saved, motors off");
    homed = false;
    return false;
  }

  // The trigger is calSteps CW from the start position, so the angle past the
  // trigger that brings the hand to up (0°) is 360 - start - trigger
  for (int m = 0; m < NUM_HANDS; m++) {
    float start = isTopHand(m) ? CAL_START_TOP : CAL_START_BOTTOM;
    long homeSteps = (2 * STEPS_PER_REV - degreesToSteps(start) - calSteps[m]) % STEPS_PER_REV;
    homeDegrees[m] = homeSteps * 360.0 / STEPS_PER_REV;
  }
  saveCalibration();
  Serial.println("Calibration saved to EEPROM.");
  printCalibration();

  // Calibration finishes with each hand back at its start position
  homed = true;
  for (int m = 0; m < NUM_HANDS; m++) {
    handPos[m] = degreesToSteps(isTopHand(m) ? CAL_START_TOP : CAL_START_BOTTOM) % STEPS_PER_REV;
  }
  Serial.println("Type h to test homing.");
  return true;
}

// ================= MOVING =================
// Moves each hand by its own number of steps (+ = CW, - = CCW). All hands
// start and finish together: the longest move runs at full speed and the
// rest are spread evenly across the same time.
void moveSteps(const long signedSteps[]) {
  moveStepsAt(signedSteps, PATTERN_TICK_US);
}

// Same, at a chosen speed (microseconds between steps)
void moveStepsAt(const long signedSteps[], unsigned long tickUs) {
  long steps[NUM_HANDS];
  long acc[NUM_HANDS];
  long total = 0;
  for (int m = 0; m < NUM_HANDS; m++) {
    steps[m] = labs(signedSteps[m]);
    acc[m] = 0;
    if (steps[m] > total) total = steps[m];
    setDirection(m, signedSteps[m] >= 0);
    handPos[m] = ((handPos[m] + signedSteps[m]) % STEPS_PER_REV + STEPS_PER_REV) % STEPS_PER_REV;
  }
  if (total == 0) return;

  enableSteppers();
  delayMicroseconds(10);

  unsigned long lastTick = micros();
  for (long i = 0; i < total; i++) {
    while (micros() - lastTick < tickUs) {}
    lastTick = micros();

    bool stepped[NUM_HANDS];
    for (int m = 0; m < NUM_HANDS; m++) {
      acc[m] += steps[m];
      stepped[m] = acc[m] >= total;
      if (stepped[m]) {
        acc[m] -= total;
        stepHigh(m);
      }
    }
    delayMicroseconds(3);
    for (int m = 0; m < NUM_HANDS; m++) {
      if (stepped[m]) stepLow(m);
    }
  }

  if (!HOLD_POSITION) disableSteppers();
}

// Moves every hand CW to its target angle (0 = up)
void moveHandsTo(const float targetDegrees[]) {
  if (!homed) {
    Serial.println("Not homed - type h first");
    return;
  }
  long s[NUM_HANDS];
  for (int m = 0; m < NUM_HANDS; m++) {
    long target = degreesToSteps(targetDegrees[m] + 360.0) % STEPS_PER_REV;
    s[m] = (target - handPos[m] + STEPS_PER_REV) % STEPS_PER_REV;
  }
  moveSteps(s);
}

// Moves the hands to a digit from the DIGITS table (in flash)
void showDigit(int d) {
  float targets[NUM_HANDS];
  for (int m = 0; m < NUM_HANDS; m++) targets[m] = (int16_t)pgm_read_word(&DIGITS[d][m]);
  moveHandsTo(targets);
  disableSteppers();                // EN high once the digit is drawn - motors off
}

// Moves each hand by its own angle: + = CW, - = CCW
void moveHandsBy(const float moveDegrees[]) {
  if (!homed) {
    Serial.println("Not homed - type h first");
    return;
  }
  long s[NUM_HANDS];
  for (int m = 0; m < NUM_HANDS; m++) {
    s[m] = degreesToSteps(fabs(moveDegrees[m])) * (moveDegrees[m] < 0 ? -1 : 1);
  }
  moveSteps(s);
}

// Moves one hand by an angle. Works before homing, e.g. to line hands up for calibration.
void jogHand(int m, float degrees, unsigned long tickUs) {
  long s[NUM_HANDS] = { 0 };
  s[m] = degreesToSteps(fabs(degrees)) * (degrees < 0 ? -1 : 1);
  moveStepsAt(s, tickUs);
}

// ================= HAND TEST =================
void waitForKey() {
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {}
  delay(20);
  while (Serial.available()) Serial.read();
}

// Turns one hand a quarter turn CW, from up to right, then on round back to up
void testHand(int m) {
  float targets[NUM_HANDS];
  for (int i = 0; i < NUM_HANDS; i++) targets[i] = 0;
  targets[m] = 90;

  printHand(m);
  Serial.println(": should turn CW from UP to RIGHT. Send any key to continue.");
  moveHandsTo(targets);
  waitForKey();

  targets[m] = 0;
  moveHandsTo(targets);
}

void testHands(int only) {
  if (!homed && !homeAll()) return;
  if (only >= 0) {
    testHand(only);
  } else {
    for (int m = 0; m < NUM_HANDS; m++) testHand(m);
  }
  Serial.println("Hand test done");
}

// ================= WIRING CHECK =================
// Turns each hand CW in order (default 90°), printing its clock and pins, so
// you can see whether the hand that moves is the one the code thinks it is.
// While each hand moves, its clock's sensor is read every degree, and a
// min/max summary for every sensor is printed at the end. Use w 1 360 so
// every hand passes its sensor - with 90° most hands never reach it.
// Needs no homing or calibration. Send any key to stop early.
const unsigned int WIRING_CHECK_STEP_DELAY_US = 50;    // smaller = faster (homing uses the same)
const unsigned long WIRING_CHECK_TICK_US = WIRING_CHECK_STEP_DELAY_US * 2UL;   // ~1.2 s per full turn
const unsigned long WIRING_CHECK_PAUSE_MS = 400;       // gap between hands
const long WIRING_SAMPLE_STEPS = STEPS_PER_REV / 360;  // read the sensor every 1°

float sensorMin[NUM_PAIRS], sensorMax[NUM_PAIRS];

float readSensorVolts(int p) {
  return analogRead(SENSOR_PINS[p]) * 5.0 / 1023.0;
}

void printSensorSummary() {
  Serial.println();
  Serial.println("Sensor summary (volts while each clock's hands were turning):");
  Serial.println("Clock  Pin   Min    Max");
  for (int p = 0; p < NUM_PAIRS; p++) {
    if (sensorMax[p] < 0) continue;   // clock not tested
    Serial.print(p + 1 < 10 ? "  " : " ");
    Serial.print(p + 1);
    Serial.print("    A");
    Serial.print(SENSOR_PINS[p] - A0);
    Serial.print(SENSOR_PINS[p] - A0 < 10 ? "   " : "  ");
    Serial.print(sensorMin[p], 2);
    Serial.print("   ");
    Serial.print(sensorMax[p], 2);
    if (sensorMax[p] < 3.0) Serial.print("   <- never above 3.0 V: hand not seen (or didn't pass the sensor)");
    else if (sensorMin[p] > 1.0) Serial.print("   <- high background: stray light or wiring?");
    Serial.println();
  }
}

void wiringCheck(int first, float degrees) {
  Serial.print("Wiring check - each hand turns ");
  Serial.print(degrees, 0);
  Serial.println(" CW in turn. Send any key to stop.");
  while (Serial.available()) Serial.read();

  for (int p = 0; p < NUM_PAIRS; p++) {
    sensorMin[p] = 99;
    sensorMax[p] = -1;
  }

  bool stopped = false;
  for (int m = first; m < NUM_HANDS && !stopped; m++) {
    int p = m / 2;
    printHand(m);
    Serial.print("  STEP pin ");
    Serial.print(STEP_PINS[m]);
    Serial.print("  DIR pin ");
    Serial.print(DIR_PINS[m]);
    Serial.print("  sensor A");
    Serial.println(SENSOR_PINS[p] - A0);

    // Move in 1° pieces, reading this clock's sensor after each one
    long total = degreesToSteps(degrees);
    long s[NUM_HANDS] = { 0 };
    for (long done = 0; done < total; done += WIRING_SAMPLE_STEPS) {
      s[m] = min(WIRING_SAMPLE_STEPS, total - done);
      moveStepsAt(s, WIRING_CHECK_TICK_US);

      float v = readSensorVolts(p);
      if (v < sensorMin[p]) sensorMin[p] = v;
      if (v > sensorMax[p]) sensorMax[p] = v;
    }

    unsigned long start = millis();
    while (millis() - start < WIRING_CHECK_PAUSE_MS) {
      if (Serial.available()) {
        while (Serial.available()) Serial.read();
        Serial.println("Wiring check stopped");
        stopped = true;
        break;
      }
    }
  }
  if (!stopped) Serial.println("Wiring check done");
  printSensorSummary();
}

// ================= SENSOR SWEEP =================
// Fast sensor check: all 12 top hands turn one full turn together, then all
// 12 bottom hands, while every sensor is read each degree. Prints each
// clock's background (min) and the peak seen from its top and bottom hand.
// Takes about 3 seconds. Needs no homing or calibration.
void sensorSweep() {
  Serial.println("Sensor sweep - top hands turn once, then bottom hands...");
  float minV[NUM_PAIRS], topMax[NUM_PAIRS], bottomMax[NUM_PAIRS];
  for (int p = 0; p < NUM_PAIRS; p++) {
    minV[p] = 99;
    topMax[p] = 0;
    bottomMax[p] = 0;
  }

  // Detection zones: every angle range where the digital input (what homing
  // uses) saw the sensor fire. A normal hand gives exactly one zone.
  const int MAX_ZONES = 4;
  static int16_t zoneStart[2][NUM_PAIRS][MAX_ZONES], zoneEnd[2][NUM_PAIRS][MAX_ZONES];
  static uint8_t zoneCount[2][NUM_PAIRS];
  bool inZone[NUM_PAIRS];

  for (int layer = 0; layer < 2; layer++) {        // 0 = top hands, 1 = bottom hands
    long s[NUM_HANDS] = { 0 };
    for (int p = 0; p < NUM_PAIRS; p++) { zoneCount[layer][p] = 0; inZone[p] = false; }

    for (long done = 0; done < STEPS_PER_REV; done += WIRING_SAMPLE_STEPS) {
      for (int p = 0; p < NUM_PAIRS; p++) s[p * 2 + layer] = WIRING_SAMPLE_STEPS;
      moveStepsAt(s, WIRING_CHECK_TICK_US);

      for (int p = 0; p < NUM_PAIRS; p++) {
        float v = readSensorVolts(p);
        if (v < minV[p]) minV[p] = v;
        float &peak = layer == 0 ? topMax[p] : bottomMax[p];
        if (v > peak) peak = v;

        int16_t angle = (int16_t)(handPos[p * 2 + layer] * 360L / STEPS_PER_REV);
        bool det = sensorDetected(p);
        uint8_t &n = zoneCount[layer][p];
        if (det && !inZone[p] && n < MAX_ZONES) { zoneStart[layer][p][n] = angle; zoneEnd[layer][p][n] = angle; n++; }
        if (det && n > 0) zoneEnd[layer][p][n - 1] = angle;
        inZone[p] = det;
      }
    }
  }

  Serial.println();
  Serial.println("Clock  Pin   Min    Top max  Bottom max");
  for (int p = 0; p < NUM_PAIRS; p++) {
    Serial.print(p + 1 < 10 ? "  " : " ");
    Serial.print(p + 1);
    Serial.print("    A");
    Serial.print(SENSOR_PINS[p] - A0);
    Serial.print(SENSOR_PINS[p] - A0 < 10 ? "   " : "  ");
    Serial.print(minV[p], 2);
    Serial.print("   ");
    Serial.print(topMax[p], 2);
    Serial.print("     ");
    Serial.print(bottomMax[p], 2);
    if (topMax[p] < 3.0)    Serial.print("   <- top hand not seen");
    if (bottomMax[p] < 3.0) Serial.print("   <- bottom hand not seen");
    if (minV[p] > 1.0)      Serial.print("   <- high background: stray light or wiring?");
    Serial.println();
  }
  Serial.println("A hand needs to read above 3.0 V to be detected.");

  // Where each sensor fired. Homing stops at the FIRST zone a hand reaches
  // going CW, so any extra zone gives a wrong home from some start positions.
  Serial.println();
  Serial.println(homed ? "Angles where each sensor fired (0 = up, hand tip angle):"
                       : "Angles where each sensor fired (not homed - angles only relative):");
  for (int p = 0; p < NUM_PAIRS; p++) {
    for (int layer = 0; layer < 2; layer++) {
      int m = p * 2 + layer;
      Serial.print("Clock ");
      Serial.print(p + 1);
      Serial.print(p + 1 < 10 ? "  " : " ");
      Serial.print(layer == 0 ? "top    (hand " : "bottom (hand ");
      Serial.print(m + 1);
      Serial.print("): ");
      uint8_t n = zoneCount[layer][p];
      if (n == 0) Serial.print("never");
      for (int z = 0; z < n; z++) {
        if (z) Serial.print(", ");
        Serial.print(zoneStart[layer][p][z]);
        Serial.print("-");
        Serial.print(zoneEnd[layer][p][z]);
      }
      if (calibrated) {
        Serial.print("   (calibrated trigger ");
        float trig = 360.0 - homeDegrees[m];
        if (trig >= 360.0) trig -= 360.0;
        Serial.print(trig, 0);
        Serial.print(")");
      }
      if (n > 1) Serial.print("   <- EXTRA ZONE");
      Serial.println();
    }
  }
}

// ================= CROSSTALK TEST =================
// Turns each hand one full turn on its own while every other hand stays still,
// and reports any OTHER clock's sensor that fires meanwhile. With correct
// wiring only the hand's own sensor ever fires. About 35 seconds.
void crosstalkTest() {
  Serial.println("Crosstalk test - each hand turns once on its own. Keep your hands clear of the clock.");
  if (homed) moveHandsTo(UP_DEGREES);       // park everything away from the sensors

  bool startDet[NUM_PAIRS];
  for (int p = 0; p < NUM_PAIRS; p++) {
    startDet[p] = sensorDetected(p);
    if (startDet[p]) {
      Serial.print("Clock ");
      Serial.print(p + 1);
      Serial.println(" sensor already detected at the start - ignored for crosstalk");
    }
  }

  int problems = 0;
  unsigned long startMs = millis();
  for (int m = 0; m < NUM_HANDS; m++) {
    int own = m / 2;
    bool ownSeen = false;
    bool other[NUM_PAIRS] = { false };

    // Steps just this one hand (much faster than the all-hands move routine),
    // checking every sensor after each step. A full turn, so handPos is unchanged.
    enableSteppers();
    setDirection(m, true);
    delayMicroseconds(10);
    unsigned long lastTick = micros();
    for (long i = 0; i < STEPS_PER_REV; i++) {
      while (micros() - lastTick < WIRING_CHECK_TICK_US) {}
      lastTick = micros();
      stepHigh(m);
      delayMicroseconds(3);
      stepLow(m);
      for (int p = 0; p < NUM_PAIRS; p++) {
        if (!sensorDetected(p)) continue;
        if (p == own) ownSeen = true;
        else if (!startDet[p]) other[p] = true;
      }
    }

    printHand(m);
    Serial.print(ownSeen ? ": own sensor OK" : ": own sensor NOT seen");
    bool cross = false;
    for (int p = 0; p < NUM_PAIRS; p++) {
      if (!other[p]) continue;
      if (!cross) Serial.print("   also fired clock(s):");
      Serial.print(" ");
      Serial.print(p + 1);
      cross = true;
    }
    if (cross) Serial.print("   <- CROSSTALK");
    if (cross || !ownSeen) problems++;
    Serial.println();
  }
  Serial.println(problems ? "Problems found - see above." : "No crosstalk - every sensor only sees its own hands.");
  Serial.print("Took ");
  Serial.print((millis() - startMs) / 1000.0, 1);
  Serial.println(" s");
  if (!HOLD_POSITION) disableSteppers();
}

// ================= SENSOR MONITOR =================
// Prints a line only when a sensor changes between detected and clear (the
// digital reading homing uses), showing which clock changed and every clock
// that is detected at that moment, with its voltage.
void printDetectedList() {
  bool any = false;
  for (int p = 0; p < NUM_PAIRS; p++) {
    if (!sensorDetected(p)) continue;
    Serial.print("  ");
    Serial.print(p + 1);
    Serial.print("(");
    Serial.print(readSensorVolts(p), 1);
    Serial.print("V)");
    any = true;
  }
  if (!any) Serial.print("  none");
  Serial.println();
}

void monitorSensors() {
  Serial.println("Sensor monitor - prints only when a sensor changes. Send any key to stop.");
  while (Serial.available()) Serial.read();

  bool last[NUM_PAIRS];
  for (int p = 0; p < NUM_PAIRS; p++) last[p] = sensorDetected(p);
  Serial.print("Detected now:");
  printDetectedList();

  while (!Serial.available()) {
    for (int p = 0; p < NUM_PAIRS; p++) {
      bool det = sensorDetected(p);
      if (det == last[p]) continue;
      last[p] = det;

      Serial.print(millis());
      Serial.print(" ms  clock ");
      Serial.print(p + 1);
      Serial.print(" (A");
      Serial.print(SENSOR_PINS[p] - A0);
      Serial.print(det ? ") ON " : ") off");
      Serial.print("   detected now:");
      printDetectedList();
    }
    delay(2);
  }
  while (Serial.available()) Serial.read();
  Serial.println("Sensor monitor stopped");
}

// ================= PATTERN =================
bool paused = true;                 // start paused - type p to run the pattern
bool inPattern = false;             // true once the hands have been put in the box

void patternStep() {
  if (!inPattern) {
    moveHandsTo(BOX_DEGREES);
    inPattern = true;
  }
  moveHandsBy(SPIN_MOVE);
}

// ================= COMMANDS =================
// Reads "x", "x 5" or "x 5 -1.5" style commands
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  char c = line.charAt(0);
  String args = line.substring(1);
  args.trim();
  int space = args.indexOf(' ');
  int hand = args.length() ? args.toInt() : 0;                     // first number (1-24)
  float value = space > 0 ? args.substring(space + 1).toFloat() : 0;   // second number

  if (c != 'p') inPattern = false;   // other commands restart the pattern from the box

  if (c >= '0' && c <= '9') {
    int d = c - '0';
    paused = true;
    if (!DIGIT_DEFINED[d]) { Serial.print("Digit "); Serial.print(d); Serial.println(" not defined yet"); return; }
    Serial.print("Digit ");
    Serial.println(d);
    showDigit(d);
    return;
  }

  switch (c) {
    case 'h': homeAll(); break;
    case 'c': paused = true; calibrateAll(); break;
    case 'd': printCalibration(); break;
    case 's': paused = true; monitorSensors(); break;
    case 'b': paused = true; Serial.println("Box"); moveHandsTo(BOX_DEGREES); break;
    case 'u': paused = true; Serial.println("All hands up"); moveHandsTo(UP_DEGREES); break;

    case 't':
      paused = true;
      if (hand >= 1 && hand <= NUM_HANDS) testHands(hand - 1);
      else testHands(-1);
      break;

    case 'w':
      paused = true;
      homed = false;                // hands end up 90° on, so positions are no longer known
      wiringCheck(hand >= 1 && hand <= NUM_HANDS ? hand - 1 : 0, value > 0 ? value : 90);
      break;

    case 'x':
      paused = true;
      crosstalkTest();              // full turns, so hands end where they started
      break;

    case 'v':
      paused = true;
      sensorSweep();                // hands end where they started (full turns)
      break;

    case 'j':
      paused = true;
      if (hand < 1 || hand > NUM_HANDS || space < 0) { Serial.println("Usage: j <hand 1-24> <degrees, - = CCW>"); break; }
      printHand(hand - 1);
      Serial.print(": jog ");
      Serial.println(value, 1);
      jogHand(hand - 1, value, PATTERN_TICK_US);
      break;

    case 'n':
      if (!calibrated) { Serial.println("Not calibrated yet - type c"); break; }
      if (hand < 1 || hand > NUM_HANDS || space < 0) { Serial.println("Usage: n <hand 1-24> <degrees, + = further CW>"); break; }
      homeDegrees[hand - 1] += value;
      if (homeDegrees[hand - 1] < 0) homeDegrees[hand - 1] += 360;
      if (homeDegrees[hand - 1] >= 360) homeDegrees[hand - 1] -= 360;
      saveCalibration();
      printHand(hand - 1);
      Serial.print(": home now ");
      Serial.print(homeDegrees[hand - 1], 2);
      Serial.println(" (saved) - type h to check");
      break;

    case 'p':
      if (!homed) { Serial.println("Not homed - type h first"); break; }
      paused = !paused;
      Serial.println(paused ? "Pattern paused" : "Pattern running");
      break;

    case 'f':
      paused = true;
      disableSteppers();
      homed = false;                // hands may be moved by hand, so positions are unknown
      Serial.println("Motors off - hands can be turned by hand. Type h to home again.");
      break;

    default:
      Serial.println("Commands: h c d n j w v x t s b u p f 0-9 (see the top of the sketch)");
  }
}

// ================= SETUP =================
void setup() {
  pinMode(EN_PIN, OUTPUT);
  disableSteppers();                // motors off until a run starts
  setupPins();

  Serial.begin(9600);
  Serial.setTimeout(100);
  delay(500);

  loadCalibration();
  if (!calibrated) {
    Serial.println("Not calibrated yet. Type f to free the motors (or use j to jog),");
    Serial.println("set top (odd) hands UP and bottom (even) hands DOWN, then type c.");
    return;
  }

  if (homeAll()) Serial.println("Type p to start the pattern, or t to test the hands");
}

// ================= LOOP =================
void loop() {
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));

  if (paused || !homed) return;
  patternStep();
}
