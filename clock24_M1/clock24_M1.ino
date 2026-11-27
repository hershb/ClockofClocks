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
//   w 1 360 on Mega 1 runs hands 1-24, then signals Mega 2 (pin 18) to run 25-48
//   w 1 -360     same but CCW - a hand that goes CW instead has a DIR fault
//   v            fast sensor sweep (~3 s): all top hands turn once together, then all
//                bottom hands, with background and peak voltage for every sensor
//   x            crosstalk test: each hand turns once alone; reports any other
//                clock's sensor that fires (~35 s)
//   r 80         set the speed of the w, v and x tests (us per step, smaller =
//                faster; homing uses STEP_DELAY_US). Not saved.
//   t            test every hand in turn: should turn CW from UP to RIGHT
//   t 5          test hand 5 only
//   s            sensor monitor (any key to stop)
//   y            direction test: all hands turn 90° CW together, pause, then 90° CCW back
//   y 360        same with a full turn each way
//   a            alternate test: odd hands (tops) 90° CCW while even hands (bottoms) go
//                90° CW, pause, then each goes back the other way
//   a 360        same with a full turn
//   k 9          DIR finder for hand 9: steps it 20° 24 times, each time with only ONE
//                DIR pin set to CCW. The pin printed when the hand goes CCW is the
//                DIR pin its driver is really wired to
//   b            box around the edge of the grid
//   u            all hands up
//   0 - 9        show that digit (on Mega 1 this also tells Mega 2 to draw the bottom half)
//                h f u b c p o on Mega 1 are also passed on to Mega 2
//   m            show: 0-5 (5 s each), pattern 20 s, 6-9 (5 s each), 3 times (m 1 = once)
//   z            count 0-9, 2 s between digits (z 500 = 0.5 s); any key stops
//   o            pattern start position: six boxes (corners + middle pair)
//   p            start / pause the continuous pattern (starts from the six boxes)
//   f            free the motors so hands can be turned by hand
// Angles: 0 = up, 90 = right, 180 = down, 270 = left.

#include <EEPROM.h>
#include <Wire.h>

// ================= PINS =================
// ================= BOARD =================
// The display is two Megas: Mega 1 drives the top half (hands 1-24, clocks
// 1-12), Mega 2 the bottom half (hands 25-48, clocks 13-24). Set this per board.
const int BOARD_ID = 1;
const int HAND_BASE = (BOARD_ID - 1) * 24;   // added to every hand number printed
const int CLOCK_BASE = (BOARD_ID - 1) * 12;  // added to every clock number printed
// Link wire: Mega 1 pin 18 -> Mega 2 pin 18, plus a GND wire between the boards.
// Mega 1 sends Mega 2 one HIGH pulse; its length says what to do:
//   20, 30 ... 110 ms   show digit 0 ... 9 (sent before Mega 1 moves, so both
//                       halves draw together)
//   130 h  140 f  150 u  160 b  170 c  180 p  190 o   (Mega 2 runs the same command)
//   250 - 1750 ms       wiring check on hands 25-48, (length - 1000) degrees
const int LINK_PIN = 18;

// I2C from the ESP32 (Mega pins 20 SDA / 21 SCL, through the level shifter).
// Only Mega 1 listens; it passes everything on to Mega 2 over the link.
// Messages:  'D', digit 0-9   -> show that digit (the minute units)
//            'C', letter      -> run that command, e.g. 'C','h' = home
const uint8_t I2C_ADDRESS = 0x10;  // minute-units board
volatile int i2cDigit = -1;        // waiting to be shown (set in the I2C interrupt)
volatile char i2cCommand = 0;

// EN is ONE wire shared by all 48 drivers and BOTH Megas' pin 19. LOW = motors on.
// So the two Megas never fight, neither ever drives it HIGH: "on" = drive LOW,
// "off" = let go (input with pull-up). The motors are on while EITHER Mega wants
// them on, and off only once both have let go.
const int EN_PIN = 19;

const int NUM_PAIRS = 12;  // clocks
const int NUM_HANDS = NUM_PAIRS * 2;

// Odd hands are the top (front) hand on each shaft
//                    clock:   1       2       3       4
//                             5       6       7       8
//                             9      10      11      12
// Test wiring: clocks 5<->6 and 7<->8 are swapped on the cable, and on clocks
// 5-8 the top and bottom hands are swapped, so their pins are rearranged here
//                     HAND NUMBER        1   2    3   4    5   6    7   8   9   10   11   12   13  14   15  16   17  18   19  20   21  22   23  24
const uint8_t STEP_PINS[NUM_HANDS] = { 3, 5, 7, 9, 11, 13, 15, 17, 31, 33, 35, 37, 23, 25, 27, 29, 53, 51, 49, 47, 45, 43, 41, 39 };
const uint8_t DIR_PINS[NUM_HANDS] = { 2, 4, 6, 8, 10, 12, 14, 16, 30, 32, 34, 36, 22, 24, 26, 28, 52, 50, 48, 46, 44, 42, 40, 38 };
const uint8_t SENSOR_PINS[NUM_PAIRS] = { A0, A1, A2, A3, A4, A5, A8, A9, A10, A11, A12, A13 };

// For Mega 2
// //                     HAND NUMBER        1   2     3   4   5   6    7   8   9   10   11   12   13  14   15  16   17  18   19  20   21  22   23  24
// const uint8_t STEP_PINS[NUM_HANDS]   = {  9,  11,  13,  3,  5,  7,  15, 17, 31,  33,  35,  37,  23, 25,  27, 29,  53, 51,  49, 47,  39, 41,  43, 45 };
// const uint8_t DIR_PINS[NUM_HANDS]    = {  8,  10,  12,  2,  4,  6,  14, 16, 30,  32,  34,  36,  22, 24,  26, 28,  52, 50,  48, 46,  38, 40,  42, 44 };
// const uint8_t SENSOR_PINS[NUM_PAIRS] = { A0, A1,  A2, A3,  A4, A5,  A8, A9, A10, A11, A12, A13 };

// Change pin assignment when using the shield
//                        HAND NUMBER        1   2    3   4    5   6    7   8    9  10   11   12   13  14   15  16   17  18   19  20   21  22   23  24
// const uint8_t STEP_PINS[NUM_HANDS]   = {  3,  5,   7,  9,  11, 13,  15, 17,  23, 25,  27,  29,  31, 33,  35, 37,  39, 41,  43, 45,  47, 49,  51, 53 };
// const uint8_t DIR_PINS[NUM_HANDS]    = {  2,  4,   6,  8,  10, 12,  14, 16,  22, 24,  26,  28,  30, 32,  34, 36,  38  40,  42, 44,  46, 48,  50, 52 };
// const uint8_t SENSOR_PINS[NUM_PAIRS] = { A0, A1,  A2, A3,  A4, A5,  A6, A7,  A8, A9,  A10, A11 };

const int DETECTED = HIGH;  // sensor reads 1 when a hand reflects
const int CW = LOW;         // DIR level that turns the hands clockwise from the front

// ================= MOTOR SETUP =================
const long STEPS_PER_REV = 180L * 64;  // steps per turn of the hand (11,520)

// Where each hand is set by hand before calibrating: top hands up, bottom
// hands down, so each clock's pair forms one straight vertical line.
const float CAL_START_TOP = 0;
const float CAL_START_BOTTOM = 180;

// ================= SHAPES =================
// Box around the edge of the grid. Top hands draw the top and bottom edges'
// joins on one layer and bottom hands on the other, so every line meets a
// hand at the same height. Clocks 6 and 7 are inside the box and park at
// 7:30 (225°).
//                            clock:      1        2        3        4        5        6        7        8        9       10       11       12
const float BOX_DEGREES[NUM_HANDS] = { 90, 180, 270, 90, 90, 270, 270, 180, 180, 0, 225, 225, 225, 225, 180, 0, 0, 90, 90, 270, 270, 90, 0, 270 };
const float UP_DEGREES[NUM_HANDS] = { 0 };

// Digits: this Mega's 12 clocks draw the top half of a digit (the other Mega
// draws the bottom half). One row per digit, two angles (top, bottom hand) per clock.
//                           clock:    1        2        3        4        5        6        7        8        9       10       11       12
const int16_t DIGITS[10][NUM_HANDS] PROGMEM = {
  // stored in flash, not RAM
  /* 0 */ { 90, 180, 270, 90, 90, 270, 270, 180, 180, 0, 90, 180, 270, 180, 180, 0, 0, 180, 180, 0, 180, 0, 0, 180 },
  /* 1 */ { 225, 225, 180, 90, 90, 270, 270, 180, 225, 225, 0, 90, 180, 270, 180, 0, 225, 225, 225, 225, 0, 180, 0, 180 },
  /* 2 */ { 90, 180, 270, 90, 90, 270, 270, 180, 90, 0, 270, 90, 180, 270, 180, 0, 90, 180, 270, 90, 0, 270, 0, 180 },
  /* 3 */ { 90, 180, 270, 90, 90, 270, 270, 180, 90, 0, 270, 90, 180, 270, 180, 0, 90, 180, 270, 90, 0, 270, 0, 180 },  // top half same as 2
  /* 4 */ { 90, 180, 270, 180, 90, 180, 270, 180, 180, 0, 180, 0, 180, 0, 180, 0, 0, 180, 0, 90, 0, 270, 0, 180 },
  /* 5 */ { 90, 180, 270, 90, 90, 270, 270, 180, 180, 0, 180, 90, 90, 270, 270, 0, 0, 180, 0, 90, 90, 270, 270, 180 },    // top bar, left column
  /* 6 */ { 90, 180, 270, 90, 90, 270, 270, 180, 180, 0, 180, 90, 90, 270, 270, 0, 0, 180, 0, 90, 90, 270, 270, 180 },    // top half same as 5
  /* 7 */ { 90, 180, 270, 90, 90, 270, 270, 180, 90, 0, 270, 90, 180, 270, 180, 0, 225, 225, 225, 225, 0, 180, 0, 180 },  // top half same as 1
  /* 8 */ { 90, 180, 270, 90, 90, 270, 270, 180, 180, 0, 90, 180, 270, 180, 180, 0, 0, 135, 90, 0, 270, 0, 0, 225 },      // two holes, waist hands 18/24 angled in
  /* 9 */ { 90, 180, 270, 90, 90, 270, 270, 180, 180, 0, 90, 180, 270, 180, 180, 0, 0, 180, 90, 0, 270, 0, 0, 180 },      // top half same as 8
};
const bool DIGIT_DEFINED[10] = { true, true, true, true, true, true, true, true, true, true };

// Pattern start: six equal 2 x 2 boxes over the whole 4 x 6 display - one in
// each corner and two in the middle (the middle pair straddles the two Megas).
// Both hands of every clock point at the centre of its box, so the four
// clocks of a box meet in the middle (135 / 225 / 45 / 315).
//   Mega 1 rows:  top of boxes / bottom of boxes / top of middle boxes
//   Mega 2 rows:  bottom of middle boxes / top of boxes / bottom of boxes
//                             clock:       1        2        3        4        5        6        7        8        9       10       11       12
const float PATTERN_START[NUM_HANDS] = { 135, 135, 225, 225, 135, 135, 225, 225, 45, 45, 315, 315, 45, 45, 315, 315, 135, 135, 225, 225, 135, 135, 225, 225 };

// Continuous pattern: every step turns each hand 45° (+ = CW, - = CCW).
// Top hands CW, bottom hands CCW, so each clock's hands scissor open and shut.
//                        clock:          1        2        3        4        5        6        7        8        9       10       11       12
const float SPIN_MOVE[NUM_HANDS] = { 45, -45, 45, -45, 45, -45, 45, -45, 45, -45, 45, -45, 45, -45, 45, -45, 45, -45, 45, -45, 45, -45, 45, -45 };

// ================= TIMING =================
// Homing and calibration must use the same speed, or the sensor triggers at a
// slightly different point and the stored calibration no longer matches.
const unsigned int STEP_DELAY_US = 200;  // homing + calibration - quietest speed (smaller = faster)
const unsigned long TICK_US = STEP_DELAY_US * 2UL;

const unsigned int PATTERN_STEP_DELAY_US = 200;  // shapes, jog and test
const unsigned int SPIN_STEP_DELAY_US = 300;     // the running pattern (p) - bigger = slower
const unsigned long PATTERN_TICK_US = PATTERN_STEP_DELAY_US * 2UL;

const long HOME_MAX_STEPS = STEPS_PER_REV * 2;  // give up after 2 turns
const long CLEAR_MARGIN = 100;                  // extra steps past the sensor when clearing
const long CHECK_STEPS = STEPS_PER_REV / 4;     // 90° - if still blocked after this, it's the other hand

// true = keep the motors powered after moves so hands can't drift
const bool HOLD_POSITION = true;

// ================= STATE =================
long remaining[NUM_HANDS];  // steps left in each hand's current homing move
long calSteps[NUM_HANDS];   // calibration: steps from the start position to the trigger
long handPos[NUM_HANDS];    // steps CW from "up" - valid once homed
bool homed = false;
long moved[NUM_HANDS];      // homing: steps each hand has turned so far
long triggerAt[NUM_HANDS];  // homing: value of moved[] when the sensor saw the hand

enum PairState : uint8_t {
  // homing
  H_CLEAR_A,  // moving the top hand off the sensor
  H_CLEAR_B,  // still blocked, so moving the bottom hand off
  H_SEEK_A,   // top hand running to the sensor
  H_LEAVE_B,  // making sure the bottom hand starts clear of the sensor
  H_SEEK_B,   // bottom hand running to the sensor
  // calibration
  C_LEAVE_A,  // top hand moving off the sensor (if it starts on it)
  C_SEEK_A,   // top hand running to the sensor, counting steps
  C_LEAVE_B,
  C_SEEK_B,
  // both
  S_FINISH,  // waiting for the last move to end
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
    dirPort[m] = portOutputRegister(digitalPinToPort(DIR_PINS[m]));
    dirMask[m] = digitalPinToBitMask(DIR_PINS[m]);
  }
  for (int p = 0; p < NUM_PAIRS; p++) {
    pinMode(SENSOR_PINS[p], INPUT);
    sensorPort[p] = portInputRegister(digitalPinToPort(SENSOR_PINS[p]));
    sensorMask[p] = digitalPinToBitMask(SENSOR_PINS[p]);
  }
}

inline void stepHigh(int m) {
  *stepPort[m] |= stepMask[m];
}
inline void stepLow(int m) {
  *stepPort[m] &= ~stepMask[m];
}

inline void setDirection(int m, bool cw) {
  bool high = cw ? (CW == HIGH) : (CW != HIGH);
  if (high) *dirPort[m] |= dirMask[m];
  else *dirPort[m] &= ~dirMask[m];
}

inline bool sensorDetected(int p) {
  bool high = (*sensorPort[p] & sensorMask[p]) != 0;
  return high == (DETECTED == HIGH);
}

// ================= ENABLE =================
bool steppersOn = false;

void enableSteppers() {
  if (steppersOn) return;     // already on - no delay, so back-to-back moves run smoothly
  digitalWrite(EN_PIN, LOW);  // pull-up off first, so the pin goes straight to LOW...
  pinMode(EN_PIN, OUTPUT);    // ...when it becomes an output
  delay(5);                   // let the drivers power up the coils
  steppersOn = true;
}

void disableSteppers() {
  pinMode(EN_PIN, INPUT_PULLUP);  // let go - never drive HIGH (the other Mega may be driving LOW)
  steppersOn = false;
}

long degreesToSteps(float degrees) {
  return (long)(STEPS_PER_REV * degrees / 360.0 + 0.5);
}

bool isTopHand(int m) {
  return m % 2 == 0;
}

void printHand(int m) {
  Serial.print("Hand ");
  Serial.print(m + 1 + HAND_BASE);
  Serial.print(" (clock ");
  Serial.print(m / 2 + 1 + CLOCK_BASE);
  Serial.print(isTopHand(m) ? ", top)" : ", bottom)");
}

// ================= CALIBRATION STORAGE (EEPROM) =================
// Angle each hand moves past its sensor trigger point to end up pointing up.
float homeDegrees[NUM_HANDS];
bool calibrated = false;

const uint32_t CAL_MAGIC = 0xC10C2401;  // changes if the stored layout changes
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
  Serial.print(p + 1 + CLOCK_BASE);
  Serial.print(" ERROR: ");
  Serial.println(msg);
}

// Advances one clock by at most one step
void pairTick(int p) {
  int a = p * 2;      // top hand
  int b = p * 2 + 1;  // bottom hand
  PairState st = pairState[p];

  if (st == S_DONE || st == S_ERROR) return;
  if (remaining[a] > 0 || remaining[b] > 0) return;  // let the current move finish

  bool det = sensorDetected(p);
  long &cnt = pairCount[p];

  switch (st) {
    // ---------- homing ----------
    case H_CLEAR_A:
      if (!det) {
        remaining[a] = CLEAR_MARGIN;
        nextState(p, H_SEEK_A);
      } else if (++cnt <= CHECK_STEPS) remaining[a] = 1;
      else nextState(p, H_CLEAR_B);
      break;

    case H_CLEAR_B:
      if (!det) {
        remaining[b] = CLEAR_MARGIN;
        nextState(p, H_SEEK_A);
      } else if (++cnt <= HOME_MAX_STEPS) remaining[b] = 1;
      else fail(p, "sensor still blocked - check for obstruction or bright light");
      break;

    case H_SEEK_A:
      if (det) {
        triggerAt[a] = moved[a];
        remaining[a] = degreesToSteps(homeDegrees[a]);
        nextState(p, H_LEAVE_B);
      } else if (++cnt <= HOME_MAX_STEPS) remaining[a] = 1;
      else fail(p, "top hand didn't reach the sensor");
      break;

    case H_LEAVE_B:
      if (!det) nextState(p, H_SEEK_B);
      else if (++cnt <= HOME_MAX_STEPS) remaining[b] = 1;
      else fail(p, "bottom hand stuck on the sensor");
      break;

    case H_SEEK_B:
      if (det) {
        triggerAt[b] = moved[b];
        remaining[b] = degreesToSteps(homeDegrees[b]);
        nextState(p, S_FINISH);
      } else if (++cnt <= HOME_MAX_STEPS) remaining[b] = 1;
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
        remaining[a] = STEPS_PER_REV - calSteps[a];  // finish the turn, back to the start
        nextState(p, C_LEAVE_B);
      } else if (++calSteps[a] < STEPS_PER_REV) remaining[a] = 1;
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
      } else if (++calSteps[b] < STEPS_PER_REV) remaining[b] = 1;
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
    Serial.print(m + 1 + HAND_BASE < 10 ? "  " : " ");
    Serial.print(m + 1 + HAND_BASE);
    Serial.print(isTopHand(m) ? "T" : "B");
    Serial.print("   ");
    Serial.print(m / 2 + 1 + CLOCK_BASE < 10 ? " " : "");
    Serial.print(m / 2 + 1 + CLOCK_BASE);
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
    if (fabs(diffDeg) > 2.0) {
      Serial.print("   <- WRONG");
      bad++;
    }
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
void showDigit(int d, bool motorsOff = true) {
  float targets[NUM_HANDS];
  for (int m = 0; m < NUM_HANDS; m++) targets[m] = (int16_t)pgm_read_word(&DIGITS[d][m]);
  moveHandsTo(targets);
  if (motorsOff) disableSteppers();  // EN released once the digit is drawn - motors off
}

// Counts 0 to 9 with a pause between digits (any key stops). On Mega 1 each
// digit is also sent to Mega 2. Mega 1 keeps EN on (LOW) for the whole count,
// so the shared EN line only goes off at the end.
void countUp(unsigned long pauseMs) {
  Serial.println("Counting 0-9 - send any key to stop");
  while (Serial.available()) Serial.read();
  for (int d = 0; d <= 9; d++) {
    if (!DIGIT_DEFINED[d]) continue;
    Serial.print("Digit ");
    Serial.println(d);
    signalMega2Digit(d);
    showDigit(d, false);
    unsigned long start = millis();
    while (millis() - start < pauseMs && !Serial.available()) {}
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println("Count stopped");
      break;
    }
  }
  disableSteppers();
  Serial.println("Count done");
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
  moveStepsAt(s, SPIN_STEP_DELAY_US * 2UL);  // only the pattern uses this - at pattern speed
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
// Speed for the w, v and x tests. Faster than homing so the tests are quick; change it while
// running with "r <us>" to find the quietest speed (not saved).
unsigned int testStepDelayUs = 50;  // smaller = faster
#define WIRING_CHECK_TICK_US (testStepDelayUs * 2UL)
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
    if (sensorMax[p] < 0) continue;  // clock not tested
    Serial.print(p + 1 + CLOCK_BASE < 10 ? "  " : " ");
    Serial.print(p + 1 + CLOCK_BASE);
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

bool wiringCheck(int first, float degrees) {  // false if stopped by a key
  Serial.print("Wiring check - each hand turns ");
  Serial.print(fabs(degrees), 0);
  Serial.println(degrees < 0 ? " CCW in turn. Send any key to stop." : " CW in turn. Send any key to stop.");
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
    long total = degreesToSteps(fabs(degrees));
    long s[NUM_HANDS] = { 0 };
    for (long done = 0; done < total; done += WIRING_SAMPLE_STEPS) {
      s[m] = min(WIRING_SAMPLE_STEPS, total - done) * (degrees < 0 ? -1 : 1);
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
  return !stopped;
}

// ================= LINK BETWEEN THE MEGAS =================
// Mega 1: one pulse on LINK_PIN, (1000 + degrees) ms long, tells Mega 2 to
// run its wiring check (hands 25-48) with that many degrees.
void sendLinkPulse(unsigned long ms) {
  digitalWrite(LINK_PIN, HIGH);
  delay(ms);
  digitalWrite(LINK_PIN, LOW);
}

void signalMega2Digit(int d) {
  if (BOARD_ID != 1) return;
  sendLinkPulse(20 + d * 10UL);
}

void signalMega2(float degrees) {
  if (BOARD_ID != 1) return;
  long d = constrain((long)(degrees + (degrees < 0 ? -0.5 : 0.5)), -720L, 720L);
  Serial.print("Telling Mega 2 to continue with hands 25-48 (");
  Serial.print(d);
  Serial.println(" deg) - see Mega 2's Serial Monitor");
  sendLinkPulse(1000 + d);
}

// Mega 2: pin 18 is an interrupt pin, so the pulse from Mega 1 is timed in
// the background - even while hands are moving - and acted on once the
// current move has finished. Lengths that mean nothing (noise) are ignored.
volatile unsigned long linkRiseUs = 0;
volatile long linkPendingMs = -1;  // length of the last pulse not yet acted on

void linkISR() {
  if (digitalRead(LINK_PIN) == HIGH) linkRiseUs = micros();
  else linkPendingMs = (long)((micros() - linkRiseUs + 500UL) / 1000UL);
}

void checkLink() {
  if (BOARD_ID != 2) return;
  noInterrupts();
  long ms = linkPendingMs;
  linkPendingMs = -1;
  interrupts();
  if (ms >= 0) {
    if (ms >= 125 && ms <= 195) {  // h f u b c p o
      char cmd = "hfubcpo"[(ms - 125) / 10];
      Serial.print("Mega 1 says: ");
      Serial.println(cmd);
      handleCommand(String(cmd));
    } else if (ms >= 15 && ms < 125) {  // digit
      int d = (ms - 15) / 10;
      Serial.print("Mega 1 says: digit ");
      Serial.println(d);
      handleCommand(String((char)('0' + d)));  // same as typing it: also stops a running pattern
    } else if (ms >= 250 && ms <= 1750 && ms != 1000) {
      float degrees = ms - 1000;
      Serial.print("Mega 1 says: wiring check on hands 25-48, ");
      Serial.print(degrees, 0);
      Serial.println(" deg");
      homed = false;  // positions no longer known (pattern stays off until h)
      wiringCheck(0, degrees);
    }
  }
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
  static bool everClear[2][NUM_PAIRS];  // false = sensor stayed ON for the whole turn
  bool inZone[NUM_PAIRS];

  for (int layer = 0; layer < 2; layer++) {  // 0 = top hands, 1 = bottom hands
    long s[NUM_HANDS] = { 0 };
    for (int p = 0; p < NUM_PAIRS; p++) {
      zoneCount[layer][p] = 0;
      inZone[p] = false;
      everClear[layer][p] = false;
    }

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
        if (!det) everClear[layer][p] = true;
        uint8_t &n = zoneCount[layer][p];
        if (det && !inZone[p] && n < MAX_ZONES) {
          zoneStart[layer][p][n] = angle;
          zoneEnd[layer][p][n] = angle;
          n++;
        }
        if (det && n > 0) zoneEnd[layer][p][n - 1] = angle;
        inZone[p] = det;
      }
    }
  }

  Serial.println();
  Serial.println("Clock  Pin   Min    Top max  Bottom max");
  for (int p = 0; p < NUM_PAIRS; p++) {
    Serial.print(p + 1 + CLOCK_BASE < 10 ? "  " : " ");
    Serial.print(p + 1 + CLOCK_BASE);
    Serial.print("    A");
    Serial.print(SENSOR_PINS[p] - A0);
    Serial.print(SENSOR_PINS[p] - A0 < 10 ? "   " : "  ");
    Serial.print(minV[p], 2);
    Serial.print("   ");
    Serial.print(topMax[p], 2);
    Serial.print("     ");
    Serial.print(bottomMax[p], 2);
    if (topMax[p] < 3.0) Serial.print("   <- top hand not seen");
    if (bottomMax[p] < 3.0) Serial.print("   <- bottom hand not seen");
    if (minV[p] > 1.0) Serial.print("   <- high background: stray light or wiring?");
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
      Serial.print(p + 1 + CLOCK_BASE);
      Serial.print(p + 1 + CLOCK_BASE < 10 ? "  " : " ");
      Serial.print(layer == 0 ? "top    (hand " : "bottom (hand ");
      Serial.print(m + 1 + HAND_BASE);
      Serial.print("): ");
      uint8_t n = zoneCount[layer][p];
      if (n == 0) Serial.print("never");
      if (n > 0 && !everClear[layer][p]) {
        Serial.print("ON the whole turn - sensor never cleared (stuck high: light or wiring)");
        n = 0;
      }
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

// ================= DIRECTION TEST =================
// All hands turn CW together, pause 1 s, then CCW back to where they started.
// Any hand going the wrong way (or the same way both times = DIR not reaching
// the driver) stands out. Works before homing; positions are unchanged after.
void directionTest(float degrees) {
  long s[NUM_HANDS];
  long n = degreesToSteps(degrees);
  Serial.print("Direction test - all hands ");
  Serial.print(degrees, 0);
  Serial.println(" deg CW...");
  for (int m = 0; m < NUM_HANDS; m++) s[m] = n;
  moveStepsAt(s, PATTERN_TICK_US);
  delay(1000);
  Serial.println("...now CCW back");
  for (int m = 0; m < NUM_HANDS; m++) s[m] = -n;
  moveStepsAt(s, PATTERN_TICK_US);
  Serial.println("Direction test done");
  if (!HOLD_POSITION) disableSteppers();
}

// Odd hands (1, 3, 5... = top hands) turn CCW while even hands turn CW, then
// both reverse back. Half the DIR pins are high and half low, so any DIR line
// that is joined to another (or driven by the wrong pin) shows up as a hand
// going the wrong way. Works before homing; positions are unchanged after.
void alternateTest(float degrees) {
  long s[NUM_HANDS];
  long n = degreesToSteps(degrees);
  Serial.print("Alternate test - odd hands (tops) ");
  Serial.print(degrees, 0);
  Serial.println(" deg CCW, even hands (bottoms) CW...");
  for (int m = 0; m < NUM_HANDS; m++) s[m] = (m % 2 == 0) ? -n : n;  // m 0 = hand 1 (odd)
  moveStepsAt(s, PATTERN_TICK_US);
  delay(1000);
  Serial.println("...now back: odd hands CW, even hands CCW");
  for (int m = 0; m < NUM_HANDS; m++) s[m] = -s[m];
  moveStepsAt(s, PATTERN_TICK_US);
  Serial.println("Alternate test done");
  if (!HOLD_POSITION) disableSteppers();
}

// DIR finder: which Mega pin really sets this hand's direction? For each of
// the 24 DIR pins in turn, that pin alone is set to CCW (all others CW) and
// the hand steps 20°. It turns CW every time except for the pin its driver is
// actually wired to. Hand positions are lost afterwards (home again).
void dirFinder(int m) {
  printHand(m);
  Serial.print(": DIR finder. Code expects DIR pin ");
  Serial.print(DIR_PINS[m]);
  Serial.println(". Watch the hand - note the pin printed when it goes CCW. Any key stops.");
  while (Serial.available()) Serial.read();
  homed = false;
  enableSteppers();
  long n = degreesToSteps(20);
  for (int k = 0; k < NUM_HANDS; k++) {
    for (int j = 0; j < NUM_HANDS; j++) setDirection(j, j != k);  // only DIR k = CCW
    delayMicroseconds(20);
    Serial.print("  only DIR pin ");
    Serial.print(DIR_PINS[k]);
    Serial.print(" (hand ");
    Serial.print(k + 1 + HAND_BASE);
    Serial.println(") set CCW");
    unsigned long lastTick = micros();
    for (long i = 0; i < n; i++) {
      while (micros() - lastTick < PATTERN_TICK_US) {}
      lastTick = micros();
      stepHigh(m);
      delayMicroseconds(3);
      stepLow(m);
    }
    delay(1500);
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println("DIR finder stopped");
      break;
    }
  }
  Serial.println("DIR finder done - home again (h) before using shapes");
  if (!HOLD_POSITION) disableSteppers();
}

// ================= CROSSTALK TEST =================
// Turns each hand one full turn on its own while every other hand stays still,
// and reports any OTHER clock's sensor that fires meanwhile. With correct
// wiring only the hand's own sensor ever fires. About 35 seconds.
void crosstalkTest() {
  Serial.println("Crosstalk test - each hand turns once on its own. Keep your hands clear of the clock.");
  if (homed) moveHandsTo(UP_DEGREES);  // park everything away from the sensors

  bool startDet[NUM_PAIRS];
  for (int p = 0; p < NUM_PAIRS; p++) {
    startDet[p] = sensorDetected(p);
    if (startDet[p]) {
      Serial.print("Clock ");
      Serial.print(p + 1 + CLOCK_BASE);
      Serial.print(" sensor already detected at the start (");
      Serial.print(readSensorVolts(p), 2);
      Serial.println(" V) - ignored for crosstalk");
    }
  }

  int problems = 0;
  unsigned long startMs = millis();
  for (int m = 0; m < NUM_HANDS; m++) {
    int own = m / 2;
    bool ownSeen = false, ownCleared = false;
    bool other[NUM_PAIRS] = { false };
    float ownMin = 99, ownMax = 0;  // own sensor voltage range during the turn

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
      if ((i & 15) == 0) {  // analog read is slow - sample every 16 steps (0.5 deg)
        float v = readSensorVolts(own);
        if (v < ownMin) ownMin = v;
        if (v > ownMax) ownMax = v;
      }
      for (int p = 0; p < NUM_PAIRS; p++) {
        if (!sensorDetected(p)) {
          if (p == own) ownCleared = true;
          continue;
        }
        if (p == own) ownSeen = true;
        else if (!startDet[p]) other[p] = true;
      }
    }

    printHand(m);
    bool ownOK = ownSeen && ownCleared;
    if (!ownSeen) Serial.print(": own sensor NOT seen  ");
    else if (!ownCleared) Serial.print(": own sensor NEVER CLEARED (stuck high)");
    else Serial.print(": own sensor OK        ");
    Serial.print("   ");
    Serial.print(ownMin, 2);
    Serial.print(" - ");
    Serial.print(ownMax, 2);
    Serial.print(" V");
    if (ownMin > 1.0) Serial.print("  <- high background");
    bool cross = false;
    for (int p = 0; p < NUM_PAIRS; p++) {
      if (!other[p]) continue;
      if (!cross) Serial.print("   also fired clock(s):");
      Serial.print(" ");
      Serial.print(p + 1 + CLOCK_BASE);
      cross = true;
    }
    if (cross) Serial.print("   <- CROSSTALK");
    if (cross || !ownOK) problems++;
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
    Serial.print(p + 1 + CLOCK_BASE);
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
      Serial.print(p + 1 + CLOCK_BASE);
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
bool paused = true;      // start paused - type p to run the pattern
bool inPattern = false;  // true once the hands have been put in the box

void patternStep() {
  if (!inPattern) {
    moveHandsTo(PATTERN_START);
    inPattern = true;
  }
  moveHandsBy(SPIN_MOVE);
}

// ================= SHOW MODE (m) =================
// Run on Mega 1: count 0-5 (5 s each), run the pattern for 20 s, count 6-9
// (5 s each), and repeat - 3 times by default. Mega 2 follows over the link
// (digits, and p to start its half of the pattern). Any key stops.
const unsigned long SHOW_DIGIT_MS = 5000;
const unsigned long SHOW_PATTERN_MS = 20000;

bool keyPressed() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

bool showWait(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms)
    if (keyPressed()) return false;
  return true;
}

bool showDigits(int from, int to) {
  for (int d = from; d <= to; d++) {
    if (!DIGIT_DEFINED[d]) continue;
    Serial.print("  digit ");
    Serial.println(d);
    signalMega2Digit(d);
    showDigit(d, false);  // keep the motors on for the whole show
    if (!showWait(SHOW_DIGIT_MS)) return false;
  }
  return true;
}

bool showPattern() {
  Serial.println("  pattern");
  if (BOARD_ID == 1) sendLinkPulse(180);  // Mega 2: p (its digit left it paused, so this starts it)
  inPattern = false;                      // start from the six boxes
  unsigned long start = millis();
  while (millis() - start < SHOW_PATTERN_MS) {
    patternStep();
    if (keyPressed()) {
      if (BOARD_ID == 1) sendLinkPulse(180);  // p again pauses Mega 2's pattern
      return false;
    }
  }
  return true;  // the next digit stops Mega 2's pattern
}

void showMode(int cycles) {
  if (!homed) {
    Serial.println("Not homed - type h first");
    return;
  }
  Serial.print("Show mode - ");
  Serial.print(cycles);
  Serial.println(" times: 0-5, pattern, 6-9. Send any key to stop.");
  while (Serial.available()) Serial.read();
  bool ok = true;
  for (int c = 1; c <= cycles && ok; c++) {
    Serial.print("Cycle ");
    Serial.println(c);
    ok = showDigits(0, 5) && showPattern() && showDigits(6, 9);
  }
  Serial.println(ok ? "Show done" : "Show stopped");
  paused = true;
  disableSteppers();
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
  int hand = args.length() ? args.toInt() : 0;                        // first number (1-24)
  float value = space > 0 ? args.substring(space + 1).toFloat() : 0;  // second number

  // On Mega 2, hand numbers 25-48 mean its local hands 1-24
  if (HAND_BASE && strchr("jntkw", c) && hand > HAND_BASE) hand -= HAND_BASE;

  if (c != 'p') inPattern = false;  // other commands restart the pattern from the box

  if (c >= '0' && c <= '9') {
    int d = c - '0';
    paused = true;
    if (!DIGIT_DEFINED[d]) {
      Serial.print("Digit ");
      Serial.print(d);
      Serial.println(" not defined yet");
      return;
    }
    Serial.print("Digit ");
    Serial.println(d);
    signalMega2Digit(d);  // Mega 1: tell Mega 2 to draw the bottom half
    showDigit(d);
    return;
  }

  // Mega 1 passes these straight on to Mega 2 (before doing them itself)
  if (BOARD_ID == 1) {
    const char *forward = "hfubcpo";
    const char *q = strchr(forward, c);
    if (q) sendLinkPulse(130 + (q - forward) * 10UL);
  }

  switch (c) {
    case 'h': homeAll(); break;
    case 'c':
      paused = true;
      calibrateAll();
      break;
    case 'd': printCalibration(); break;
    case 's':
      paused = true;
      monitorSensors();
      break;
    case 'b':
      paused = true;
      Serial.println("Box");
      moveHandsTo(BOX_DEGREES);
      break;
    case 'u':
      paused = true;
      Serial.println("All hands up");
      moveHandsTo(UP_DEGREES);
      break;
    case 'o':
      paused = true;
      inPattern = false;
      Serial.println("Pattern start - six boxes");
      moveHandsTo(PATTERN_START);
      break;

    case 'm':
      paused = true;
      showMode(hand > 0 ? hand : 3);
      break;

    case 'z':
      paused = true;
      countUp(hand > 0 ? hand : 2000);
      break;

    case 'k':
      paused = true;
      if (hand < 1 || hand > NUM_HANDS) {
        Serial.println("Usage: k <hand 1-24>");
        break;
      }
      dirFinder(hand - 1);
      break;

    case 'a':
      paused = true;
      alternateTest(hand > 0 ? hand : 90);
      break;

    case 'y':
      paused = true;
      directionTest(hand > 0 ? hand : 90);
      break;

    case 't':
      paused = true;
      if (hand >= 1 && hand <= NUM_HANDS) testHands(hand - 1);
      else testHands(-1);
      break;

    case 'w':
      {
        paused = true;
        homed = false;  // hands end up 90° on, so positions are no longer known
        float degrees = value != 0 ? value : 90;
        bool finished = true;
        if (BOARD_ID == 1 && hand > NUM_HANDS) Serial.println("Hands 25-48 are on Mega 2");
        else finished = wiringCheck(hand >= 1 && hand <= NUM_HANDS ? hand - 1 : 0, degrees);
        if (finished) signalMega2(degrees);  // Mega 1 only: Mega 2 carries on with 25-48
        break;
      }

    case 'x':
      paused = true;
      crosstalkTest();  // full turns, so hands end where they started
      break;

    case 'v':
      paused = true;
      sensorSweep();  // hands end where they started (full turns)
      break;

    case 'j':
      paused = true;
      if (hand < 1 || hand > NUM_HANDS || space < 0) {
        Serial.println("Usage: j <hand 1-24> <degrees, - = CCW>");
        break;
      }
      printHand(hand - 1);
      Serial.print(": jog ");
      Serial.println(value, 1);
      jogHand(hand - 1, value, PATTERN_TICK_US);
      break;

    case 'n':
      if (!calibrated) {
        Serial.println("Not calibrated yet - type c");
        break;
      }
      if (hand < 1 || hand > NUM_HANDS || space < 0) {
        Serial.println("Usage: n <hand 1-24> <degrees, + = further CW>");
        break;
      }
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
      if (!homed) {
        Serial.println("Not homed - type h first");
        break;
      }
      paused = !paused;
      Serial.println(paused ? "Pattern paused" : "Pattern running");
      break;

    case 'r':
      if (hand >= 20 && hand <= 2000) testStepDelayUs = hand;
      else Serial.println("Usage: r <microseconds, 20-2000> - e.g. r 80");
      Serial.print("Test speed (w, v, x): ");
      Serial.print(testStepDelayUs);
      Serial.print(" us per step = ");
      Serial.print(1000000.0 / (testStepDelayUs * 2.0) * 360.0 / STEPS_PER_REV, 0);
      Serial.println(" deg/s");
      break;

    case 'f':
      paused = true;
      disableSteppers();
      homed = false;  // hands may be moved by hand, so positions are unknown
      Serial.println("Motors off - hands can be turned by hand. Type h to home again.");
      break;

    default:
      Serial.println("Commands: h c d n j w v x r t s y a k z m b u o p f 0-9 (see the top of the sketch)");
  }
}

// ================= SETUP =================
void setup() {
  pinMode(EN_PIN, INPUT_PULLUP);  // EN released (see EN_PIN)
  if (BOARD_ID == 1) {
    pinMode(LINK_PIN, OUTPUT);
    digitalWrite(LINK_PIN, LOW);
  } else {
    pinMode(LINK_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LINK_PIN), linkISR, CHANGE);
  }
  if (BOARD_ID == 1) {
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(onI2CReceive);
  }
  disableSteppers();  // motors off until a run starts
  setupPins();

  Serial.begin(9600);
  Serial.setTimeout(100);
  delay(500);
  Serial.println(BOARD_ID == 1 ? "=== MEGA 1 - top half, hands 1-24 ===" : "=== MEGA 2 - bottom half, hands 25-48 ===");

  loadCalibration();
  if (!calibrated) {
    Serial.println("Not calibrated yet. Type f to free the motors (or use j to jog),");
    Serial.println("set top (odd) hands UP and bottom (even) hands DOWN, then type c.");
    return;
  }

  if (homeAll()) Serial.println("Type p to start the pattern, or t to test the hands");
}

// ================= LOOP =================
// I2C interrupt: just stores the message for checkI2C()
void onI2CReceive(int count) {
  if (count < 2) {
    while (Wire.available()) Wire.read();
    return;
  }
  char type = Wire.read();
  char value = Wire.read();
  while (Wire.available()) Wire.read();
  if (type == 'D' && value >= 0 && value <= 9) i2cDigit = value;
  else if (type == 'C') i2cCommand = value;
}

// Acts on the last message from the ESP32 (outside the interrupt, as the
// moves take seconds). A newer digit replaces one not yet shown.
void checkI2C() {
  if (BOARD_ID != 1) return;
  noInterrupts();
  int d = i2cDigit;
  i2cDigit = -1;
  char c = i2cCommand;
  i2cCommand = 0;
  interrupts();
  if (c) {
    Serial.print("ESP32 says: ");
    Serial.println(c);
    handleCommand(String(c));
  }
  if (d >= 0) {
    Serial.print("ESP32 says: digit ");
    Serial.println(d);
    handleCommand(String((char)('0' + d)));  // shows it here and on Mega 2
  }
}

void loop() {
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));

  checkLink();  // Mega 2: start when Mega 1 says so
  checkI2C();   // Mega 1: digit or command from the ESP32

  if (paused || !homed) return;
  patternStep();
}
