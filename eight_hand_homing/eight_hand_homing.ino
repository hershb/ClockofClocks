// 8-hand homing test - 4 pairs of hands, one IR sensor per pair.
//
// CALIBRATE (once): set CALIBRATION_MODE = true and upload. The motors start
// off - turn each hand to its CAL_START_DEGREES position (odd hands up, even
// hands down, so each clock's pair forms one straight line), then type c.
// Each hand runs CW to its sensor, counting steps, then finishes a full turn
// back to its start. The Serial Monitor prints a HOME_DEGREES line to paste
// below. Then set CALIBRATION_MODE = false and upload again.
//
// HOME (normal): each hand runs CW to its sensor, then moves its own
// HOME_DEGREES, so every hand ends up pointing up.
//
// Serial Monitor (9600 baud, "Newline"): h = home, c = calibrate (hands at start first),
// s = sensor monitor (prints each sensor's voltage; send any key to stop),
// b = make a box, u = all hands up, f = free the motors (so hands turn by hand),
// t = test each hand, p = pause / resume the box <-> star loop.
// Angles: 0 = up, 90 = right, 180 = down, 270 = left.

// ================= MODE =================
const bool CALIBRATION_MODE = false;   // true = wait for c with motors off, false = home at power-on

// ================= PINS =================
const int EN_PIN = 53;              // shared by all drivers - LOW = on, HIGH = off

const int NUM_HANDS = 8;
const int NUM_PAIRS = NUM_HANDS / 2;

//                             hand:  1   2   3   4   5   6   7   8
// Odd hands are the top (front) hand on each shaft
const int STEP_PINS[NUM_HANDS]   = { 5, 3, 9, 7,  18,  11,  14,  16 };
const int DIR_PINS[NUM_HANDS]    = { 4, 2, 8, 6,  19,  10,  15,  17 };

// DIR level that turns the hands clockwise as seen from the front
const int CW = LOW;

// Pair p uses hands 2p+1 and 2p+2 (hands 1+2 on A0, 3+4 on A1, ...)
const int SENSOR_PINS[NUM_PAIRS] = { A0, A1, A2, A3 };
const int DETECTED = HIGH;          // sensor reads 1 when a hand reflects

// ================= MOTOR SETUP =================
const long STEPS_PER_REV = 180L * 64;    // steps per turn of the hand (11,520)

// Angle each hand moves past its sensor trigger point to end up pointing up.
// Paste the line printed by calibration here.
//                                    hand:  1      2      3      4      5      6      7      8
// float HOME_DEGREES[NUM_HANDS] = { 300.0,  28.0, 300.0,  28.0, 300.0,  28.0, 300.0,  28.0 };
float HOME_DEGREES[NUM_HANDS] = { 211.13, 208.09, 210.47, 207.41, 209.13, 206.56, 212.47, 208.06 };   // 5-8 +2° by eye
// Where each hand is set by hand before calibrating (0 = up, 180 = down).
// Odd hands up and even hands down puts each clock's pair in one straight line.
//                                      hand:  1    2    3    4    5    6    7    8
const float CAL_START_DEGREES[NUM_HANDS] = {   0, 180,   0, 180,   0, 180,   0, 180 };

// ================= SHAPES =================
// Added to each hand's shape angle so every hand reads 0 = up, 90 = right,
// 180 = down, 270 = left. Leave at 0 when calibrated from CAL_START_DEGREES.
//                                hand:  1    2    3    4    5    6    7    8
const float HAND_OFFSET[NUM_HANDS] = {   0,   0,   0,   0,   0,   0,   0,   0 };

// Clocks are in a 2 x 2 square:
//   hands 3+4 top-left     hands 1+2 top-right
//   hands 7+8 bottom-left  hands 5+6 bottom-right
// Each clock draws one corner of the box.
//                                hand:  1    2    3    4    5    6    7    8
const float BOX_DEGREES[NUM_HANDS] = { 270, 180,  90, 180, 270,   0,  90,   0 };
const float UP_DEGREES[NUM_HANDS]  = {   0,   0,   0,   0,   0,   0,   0,   0 };

// Star: from the box, each hand turns 45° (+ = CW, - = CCW) so both hands of
// every clock point diagonally at the centre, making a big X.
//                                hand:  1    2    3    4    5    6    7    8
const float STAR_MOVE[NUM_HANDS]   = { -45,  45,  45, -45,  45, -45, -45,  45 };

const unsigned long STAR_PAUSE_MS = 0;   // brief hold each time the hands point at the centre

// ================= TIMING =================
// Homing and calibration must use the same speed, or the sensor triggers at a
// slightly different point and the calibrated HOME_DEGREES no longer match.
const unsigned int STEP_DELAY_US = 80;     // homing + calibration speed (smaller = faster)
const unsigned long TICK_US = STEP_DELAY_US * 2UL;   // time between steps

const unsigned int PATTERN_STEP_DELAY_US = 200;   // box, star and other shape moves
const unsigned long PATTERN_TICK_US = PATTERN_STEP_DELAY_US * 2UL;

const long HOME_MAX_STEPS = STEPS_PER_REV * 2;   // give up after 2 turns
const long CLEAR_MARGIN   = 100;                 // extra steps past the sensor when clearing
const long CHECK_STEPS    = STEPS_PER_REV / 4;   // 90° - if still blocked after this, it's the other hand

// ================= STATE =================
long remaining[NUM_HANDS];          // steps left in each hand's current move
long calSteps[NUM_HANDS];           // calibration: steps from "up" to the sensor trigger
long handPos[NUM_HANDS];            // steps CW from "up" - valid once homed
bool homed = false;

enum PairState : uint8_t {
  // homing
  H_CLEAR_A,    // moving the first hand off the sensor
  H_CLEAR_B,    // still blocked, so moving the second hand off
  H_SEEK_A,     // first hand running to the sensor
  H_LEAVE_B,    // making sure the second hand starts clear of the sensor
  H_SEEK_B,     // second hand running to the sensor
  // calibration
  C_LEAVE_A,    // first hand moving off the sensor (if it starts on it)
  C_SEEK_A,     // first hand running to the sensor, counting steps
  C_LEAVE_B,
  C_SEEK_B,
  // both
  S_FINISH,     // waiting for the last move to end
  S_DONE,
  S_ERROR
};
PairState pairState[NUM_PAIRS];
long pairCount[NUM_PAIRS];

// ================= ENABLE =================
// true = keep the motors powered after homing and moves so the hands can't
// drift (with power off each rotor settles to its nearest full step, ~2° at
// the hand). false = switch off after every run. Errors always switch off.
const bool HOLD_POSITION = true;

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

// ================= PAIR STATE MACHINE =================
void nextState(int p, PairState s) {
  pairState[p] = s;
  pairCount[p] = 0;
}

void printPair(int p) {
  Serial.print("Hands ");
  Serial.print(p * 2 + 1);
  Serial.print("+");
  Serial.print(p * 2 + 2);
}

void fail(int p, const char *msg) {
  pairState[p] = S_ERROR;
  printPair(p);
  Serial.print(" ERROR: ");
  Serial.println(msg);
}

// Advances one pair by at most one step
void pairTick(int p) {
  int a = p * 2;                    // first hand of the pair
  int b = p * 2 + 1;                // second hand of the pair
  PairState st = pairState[p];

  if (st == S_DONE || st == S_ERROR) return;
  if (remaining[a] > 0 || remaining[b] > 0) return;   // let the current move finish

  bool det = digitalRead(SENSOR_PINS[p]) == DETECTED;
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
      else fail(p, "sensor still blocked - check for obstruction");
      break;

    case H_SEEK_A:
      if (det) { remaining[a] = degreesToSteps(HOME_DEGREES[a]); nextState(p, H_LEAVE_B); }
      else if (++cnt <= HOME_MAX_STEPS) remaining[a] = 1;
      else fail(p, "first hand didn't reach the sensor");
      break;

    case H_LEAVE_B:
      if (!det) nextState(p, H_SEEK_B);
      else if (++cnt <= HOME_MAX_STEPS) remaining[b] = 1;
      else fail(p, "second hand stuck on the sensor");
      break;

    case H_SEEK_B:
      if (det) { remaining[b] = degreesToSteps(HOME_DEGREES[b]); nextState(p, S_FINISH); }
      else if (++cnt <= HOME_MAX_STEPS) remaining[b] = 1;
      else fail(p, "second hand didn't reach the sensor");
      break;

    // ---------- calibration (hands start pointing up) ----------
    // calSteps counts every step from "up" until the sensor triggers.
    case C_LEAVE_A:
      if (!det) nextState(p, C_SEEK_A);
      else if (++calSteps[a] <= STEPS_PER_REV) remaining[a] = 1;
      else fail(p, "first hand stuck on the sensor");
      break;

    case C_SEEK_A:
      if (det) {
        remaining[a] = STEPS_PER_REV - calSteps[a];   // finish the turn, back to up
        nextState(p, C_LEAVE_B);
      }
      else if (++calSteps[a] < STEPS_PER_REV) remaining[a] = 1;
      else fail(p, "first hand didn't reach the sensor in one turn");
      break;

    case C_LEAVE_B:
      if (!det) nextState(p, C_SEEK_B);
      else if (++calSteps[b] <= STEPS_PER_REV) remaining[b] = 1;
      else fail(p, "second hand stuck on the sensor");
      break;

    case C_SEEK_B:
      if (det) {
        remaining[b] = STEPS_PER_REV - calSteps[b];
        nextState(p, S_FINISH);
      }
      else if (++calSteps[b] < STEPS_PER_REV) remaining[b] = 1;
      else fail(p, "second hand didn't reach the sensor in one turn");
      break;

    case S_FINISH:
      nextState(p, S_DONE);
      printPair(p);
      Serial.println(" done");
      break;

    default:
      break;
  }
}

// Steps every hand that has a move in progress, all at the same time
void stepTick() {
  bool stepped[NUM_HANDS];
  for (int m = 0; m < NUM_HANDS; m++) {
    stepped[m] = remaining[m] > 0;
    if (stepped[m]) {
      digitalWrite(STEP_PINS[m], HIGH);
      remaining[m]--;
    }
  }
  delayMicroseconds(5);
  for (int m = 0; m < NUM_HANDS; m++) {
    if (stepped[m]) digitalWrite(STEP_PINS[m], LOW);
  }
}

// Runs all pairs from firstState until they're all done or failed.
// Returns true if every pair finished without an error.
bool runAllPairs(PairState firstState) {
  enableSteppers();

  // Always turn clockwise so each hand approaches its sensor from the same side
  for (int m = 0; m < NUM_HANDS; m++) {
    digitalWrite(DIR_PINS[m], CW);
    remaining[m] = 0;
    calSteps[m] = 0;
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
// Homes all hands from wherever they are, ending pointing up. Safe to call any time.
bool homeAll() {
  Serial.println("Homing all hands...");
  bool ok = runAllPairs(H_CLEAR_A);
  Serial.println(ok ? "Homing done" : "Homing finished with errors - motors off");
  setHomed(ok);
  return ok;
}

// After a successful home every hand is pointing up (0°)
void setHomed(bool ok) {
  homed = ok;
  if (ok) {
    for (int m = 0; m < NUM_HANDS; m++) handPos[m] = 0;
  }
}

// ================= MOVING =================
// Moves every hand CW to its target angle (0 = up). All hands start and
// finish together: the longest move runs at full speed, the rest are spread
// evenly across the same time.
void moveHandsTo(const float targetDegrees[]) {
  if (!homed) {
    Serial.println("Not homed - type h first");
    return;
  }

  long delta[NUM_HANDS];
  long acc[NUM_HANDS];
  long total = 0;
  for (int m = 0; m < NUM_HANDS; m++) {
    long target = degreesToSteps(targetDegrees[m] + HAND_OFFSET[m]) % STEPS_PER_REV;
    delta[m] = (target - handPos[m] + STEPS_PER_REV) % STEPS_PER_REV;
    acc[m] = 0;
    if (delta[m] > total) total = delta[m];
    digitalWrite(DIR_PINS[m], CW);
  }
  if (total == 0) return;

  enableSteppers();
  delayMicroseconds(10);

  unsigned long lastTick = micros();
  for (long i = 0; i < total; i++) {
    while (micros() - lastTick < PATTERN_TICK_US) {}
    lastTick = micros();

    bool stepped[NUM_HANDS];
    for (int m = 0; m < NUM_HANDS; m++) {
      acc[m] += delta[m];
      stepped[m] = acc[m] >= total;
      if (stepped[m]) {
        acc[m] -= total;
        digitalWrite(STEP_PINS[m], HIGH);
      }
    }
    delayMicroseconds(5);
    for (int m = 0; m < NUM_HANDS; m++) {
      if (stepped[m]) digitalWrite(STEP_PINS[m], LOW);
    }
  }

  for (int m = 0; m < NUM_HANDS; m++) {
    handPos[m] = (handPos[m] + delta[m]) % STEPS_PER_REV;
  }
  if (!HOLD_POSITION) disableSteppers();
}

// Moves each hand by its own amount: positive = CW, negative = CCW.
// scale = 1 does the move, -1 undoes it. All hands start and finish together.
void moveHandsBy(const float moveDegrees[], int scale) {
  if (!homed) {
    Serial.println("Not homed - type h first");
    return;
  }

  long steps[NUM_HANDS];
  long acc[NUM_HANDS];
  long total = 0;
  for (int m = 0; m < NUM_HANDS; m++) {
    long signedSteps = degreesToSteps(abs(moveDegrees[m])) * (moveDegrees[m] < 0 ? -1 : 1) * scale;
    bool cw = signedSteps >= 0;
    steps[m] = abs(signedSteps);
    acc[m] = 0;
    if (steps[m] > total) total = steps[m];
    digitalWrite(DIR_PINS[m], cw ? CW : !CW);

    handPos[m] = ((handPos[m] + signedSteps) % STEPS_PER_REV + STEPS_PER_REV) % STEPS_PER_REV;
  }
  if (total == 0) return;

  enableSteppers();
  delayMicroseconds(10);

  unsigned long lastTick = micros();
  for (long i = 0; i < total; i++) {
    while (micros() - lastTick < PATTERN_TICK_US) {}
    lastTick = micros();

    bool stepped[NUM_HANDS];
    for (int m = 0; m < NUM_HANDS; m++) {
      acc[m] += steps[m];
      stepped[m] = acc[m] >= total;
      if (stepped[m]) {
        acc[m] -= total;
        digitalWrite(STEP_PINS[m], HIGH);
      }
    }
    delayMicroseconds(5);
    for (int m = 0; m < NUM_HANDS; m++) {
      if (stepped[m]) digitalWrite(STEP_PINS[m], LOW);
    }
  }

  if (!HOLD_POSITION) disableSteppers();
}

// ================= SHAPES =================
// Box -> star (call from the box position)
void starShape() {
  moveHandsBy(STAR_MOVE, 1);
}

// Star -> box, retracing the star move in the opposite directions
void starToBox() {
  Serial.println("Box");
  moveHandsBy(STAR_MOVE, -1);
}

// ================= CALIBRATION =================
// Hands must be at their CAL_START_DEGREES positions before calling this.
bool calibrateAll() {
  Serial.println("Calibrating - hands must be at their start positions...");
  bool ok = runAllPairs(C_LEAVE_A);

  if (!ok) {
    Serial.println("Calibration failed - motors off");
    setHomed(false);
    return false;
  }

  // The trigger point is calSteps CW from the start position, so the angle
  // past the trigger that brings the hand to up (0°) is 360 - start - trigger
  for (int m = 0; m < NUM_HANDS; m++) {
    long startSteps = degreesToSteps(CAL_START_DEGREES[m]);
    long homeSteps = (2 * STEPS_PER_REV - startSteps - calSteps[m]) % STEPS_PER_REV;
    HOME_DEGREES[m] = homeSteps * 360.0 / STEPS_PER_REV;
  }

  Serial.println("Calibration done. Paste this line into the sketch:");
  Serial.print("float HOME_DEGREES[NUM_HANDS] = { ");
  for (int m = 0; m < NUM_HANDS; m++) {
    Serial.print(HOME_DEGREES[m], 2);
    if (m < NUM_HANDS - 1) Serial.print(", ");
  }
  Serial.println(" };");
  Serial.println("Type h to test homing with these values.");

  // Calibration finishes with each hand back at its start position
  homed = true;
  for (int m = 0; m < NUM_HANDS; m++) handPos[m] = degreesToSteps(CAL_START_DEGREES[m]) % STEPS_PER_REV;
  return true;
}

// ================= HAND TEST =================
// Moves one hand at a time a quarter turn to 90° (right) and back to 0° (up),
// so each hand's number, direction and position can be checked.
void waitForKey() {
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {}
  while (Serial.available()) Serial.read();
}

void testHands() {
  if (!homeAll()) return;

  float targets[NUM_HANDS];
  for (int m = 0; m < NUM_HANDS; m++) {
    for (int i = 0; i < NUM_HANDS; i++) targets[i] = 0;
    targets[m] = 90;

    Serial.print("Hand ");
    Serial.print(m + 1);
    Serial.println(": should turn CW a quarter turn, from UP to RIGHT. Send any key for next hand.");
    moveHandsTo(targets);
    waitForKey();

    targets[m] = 0;
    moveHandsTo(targets);         // carries on CW back to up
  }
  Serial.println("Hand test done");
}

// ================= SENSOR MONITOR =================
// Prints each sensor's voltage and digital reading 5 times a second.
// Motors are off, so turn the hands over the sensors by hand.
// A Mega digital input reads HIGH reliably only above about 3.0 V.
void monitorSensors() {
  Serial.println("Sensor monitor - turn hands over the sensors. Send any key to stop.");
  disableSteppers();
  while (Serial.available()) Serial.read();      // clear the 's' and line ending

  float peak[NUM_PAIRS] = {0};
  while (!Serial.available()) {
    for (int p = 0; p < NUM_PAIRS; p++) {
      float volts = analogRead(SENSOR_PINS[p]) * 5.0 / 1023.0;
      if (volts > peak[p]) peak[p] = volts;

      Serial.print("A");
      Serial.print(p);
      Serial.print(": ");
      Serial.print(volts, 2);
      Serial.print("V ");
      Serial.print(digitalRead(SENSOR_PINS[p]) == DETECTED ? "HIGH" : "low ");
      Serial.print(" (peak ");
      Serial.print(peak[p], 2);
      Serial.print("V)   ");
    }
    Serial.println();
    delay(200);
  }
  while (Serial.available()) Serial.read();
  Serial.println("Sensor monitor stopped");
}

// ================= SETUP =================
void setup() {
  pinMode(EN_PIN, OUTPUT);
  disableSteppers();                // motors off until a run starts

  for (int m = 0; m < NUM_HANDS; m++) {
    pinMode(STEP_PINS[m], OUTPUT);
    pinMode(DIR_PINS[m], OUTPUT);
  }
  for (int p = 0; p < NUM_PAIRS; p++) pinMode(SENSOR_PINS[p], INPUT);

  Serial.begin(9600);
  delay(500);

  if (CALIBRATION_MODE) {
    // Motors stay off so the hands can be turned by hand
    Serial.println("Calibration mode - motors off.");
    Serial.println("Turn odd hands (1,3,5,7) up and even hands (2,4,6,8) down, then type c");
    return;
  }

  homeAll();
}

// ================= LOOP =================
bool paused = false;                // p toggles; f, s, c and t also pause
bool inPattern = false;             // true once the hands have been put in the box
// True when every hand is at its inward star angle (box + STAR_MOVE), i.e.
// both hands of every clock point at the centre
bool atInwardStar() {
  for (int m = 0; m < NUM_HANDS; m++) {
    float deg = BOX_DEGREES[m] + STAR_MOVE[m] + HAND_OFFSET[m];
    long target = (degreesToSteps(deg + 360.0) % STEPS_PER_REV);
    if (handPos[m] != target) return false;
  }
  return true;
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c != 'p') inPattern = false;   // other commands restart the pattern from the box
    if (c == 'h') homeAll();
    else if (c == 'c') { paused = true; calibrateAll(); }
    else if (c == 's') { paused = true; monitorSensors(); }
    else if (c == 'b') { Serial.println("Making a box"); moveHandsTo(BOX_DEGREES); }
    else if (c == 'u') { paused = true; Serial.println("All hands up - loop paused"); moveHandsTo(UP_DEGREES); }
    else if (c == 't') { paused = true; testHands(); }
    else if (c == 'f') { paused = true; disableSteppers(); Serial.println("Motors off, loop paused - hands can be turned by hand"); }
    else if (c == 'p') { paused = !paused; Serial.println(paused ? "Loop paused" : "Loop running"); }
  }

  if (paused || !homed) return;

  // Start from the box once, then keep turning every hand 45° at a time in
  // its star direction, so the motion passes through
  // box -> star -> box (each clock's hands swapped) -> star -> ...
  // It only holds briefly when every hand points at the centre (once per turn).
  if (!inPattern) {
    moveHandsTo(BOX_DEGREES);
    inPattern = true;
  }
  starShape();
  if (atInwardStar()) delay(STAR_PAUSE_MS);
}
