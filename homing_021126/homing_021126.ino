// ================= PINS =================
// Motor 1
const int STEP1_PIN = 16;
const int DIR1_PIN  = 17;

// Motor 2
const int STEP2_PIN = 14;
const int DIR2_PIN  = 15;

// Enable (shared by both TMC2208s) - LOW = motors on, HIGH = motors off
const int EN_PIN = 53;

// Sensor
const int IR_PIN   = A3;
const int DETECTED = HIGH;          // sensor reads 1 when blocked

// ================= MOTOR SETUP =================
const long STEPS_PER_REV = 180L * 64;    // steps per turn of the hand (11,520)

// Real angles past the sensor trigger point
const float M1_DEGREES = 300.0;
const float M2_DEGREES =  28.0;

const long M1_STEPS = (long)(STEPS_PER_REV * M1_DEGREES / 360.0 + 0.5);
const long M2_STEPS = (long)(STEPS_PER_REV * M2_DEGREES / 360.0 + 0.5);

// ================= TIMING =================
const unsigned long HOME_INTERVAL_MS = 12UL * 60 * 60 * 1000;   // re-home every 12 hours

const unsigned int STEP_DELAY_US =  60;    // running speed (smaller = faster)
const unsigned int HOME_DELAY_US = 200;   // clearing speed

const long HOME_MAX_STEPS = STEPS_PER_REV * 2;   // give up after 2 turns
const long CLEAR_MARGIN   = 100;                 // extra steps past the sensor when clearing
const long CHECK_STEPS    = STEPS_PER_REV / 4;   // 90° - if still blocked after this, it's the other hand

// ================= MOVEMENT FUNCTIONS =================
void stepOnce(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

void stepSlow(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(HOME_DELAY_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(HOME_DELAY_US);
}

void moveSteps(int stepPin, long steps) {
  for (long i = 0; i < steps; i++) {
    stepOnce(stepPin);
  }
}

// Steps both motors at the same time so they start and finish together.
// The motor with more steps runs at full speed; the other has its steps
// spread evenly across the move, so it runs proportionally slower.
void moveBoth(long steps1, long steps2) {
  long total = max(steps1, steps2);
  long acc1 = 0, acc2 = 0;
  for (long i = 0; i < total; i++) {
    acc1 += steps1;
    acc2 += steps2;
    if (acc1 >= total) { acc1 -= total; digitalWrite(STEP1_PIN, HIGH); }
    if (acc2 >= total) { acc2 -= total; digitalWrite(STEP2_PIN, HIGH); }
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP1_PIN, LOW);
    digitalWrite(STEP2_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}


// Runs a motor until the sensor triggers, always approaching from clear.
// Returns false if the sensor isn't found within 2 turns.
bool runToSensor(int stepPin) {
  long count = 0;
  while (digitalRead(IR_PIN) == DETECTED) {
    stepOnce(stepPin);
    if (++count > HOME_MAX_STEPS) return false;
  }
  count = 0;
  while (digitalRead(IR_PIN) != DETECTED) {
    stepOnce(stepPin);
    if (++count > HOME_MAX_STEPS) return false;
  }
  return true;
}

// ================= SENSOR CLEARING =================
// Makes sure neither hand is covering the sensor.
// Returns true if the sensor ends up clear.
bool clearSensor() {
  if (digitalRead(IR_PIN) != DETECTED) {
    Serial.println("Sensor clear");
    return true;
  }

  // Try moving hand 1 off the sensor (up to 90°)
  Serial.println("Sensor blocked - moving hand 1");
  for (long i = 0; i < CHECK_STEPS; i++) {
    if (digitalRead(IR_PIN) != DETECTED) {
      for (long j = 0; j < CLEAR_MARGIN; j++) stepSlow(STEP1_PIN);
      Serial.println("Hand 1 was blocking - now clear");
      return true;
    }
    stepSlow(STEP1_PIN);
  }

  // Still blocked after hand 1 moved 90°, so hand 2 is covering it
  Serial.println("Still blocked - moving hand 2");
  for (long i = 0; i < HOME_MAX_STEPS; i++) {
    if (digitalRead(IR_PIN) != DETECTED) {
      for (long j = 0; j < CLEAR_MARGIN; j++) stepSlow(STEP2_PIN);
      Serial.println("Hand 2 was blocking - now clear");
      return true;
    }
    stepSlow(STEP2_PIN);
  }

  return false;   // something else is blocking the sensor
}

// ================= ENABLE =================
void enableSteppers() {
  digitalWrite(EN_PIN, LOW);
  delay(5);                         // let the drivers power up the coils
}

void disableSteppers() {
  digitalWrite(EN_PIN, HIGH);
}

// ================= HOMING =================
// Homes both hands from wherever they are and leaves them at their
// M1/M2 positions. Safe to call any time. Returns false if homing failed.
bool homeHands() {
  Serial.println("Homing...");
  enableSteppers();

  // Always home clockwise so each hand approaches the sensor from the same side
  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);
  delayMicroseconds(10);

  // 1. Make sure neither hand is over the sensor
  if (!clearSensor()) {
    Serial.println("ERROR: sensor still blocked - check for obstruction");
    return false;
  }

  // 2. Hand 1: run to sensor, then move to its position
  Serial.println("Homing hand 1...");
  if (!runToSensor(STEP1_PIN)) {
    Serial.println("ERROR: hand 1 didn't reach the sensor");
    return false;
  }
  moveSteps(STEP1_PIN, M1_STEPS);

  // 3. Hand 2: run to sensor, then move to its position
  Serial.println("Homing hand 2...");
  if (!runToSensor(STEP2_PIN)) {
    Serial.println("ERROR: hand 2 didn't reach the sensor");
    return false;
  }
  moveSteps(STEP2_PIN, M2_STEPS);

  Serial.println("Homing done");
  return true;
}

// Stops everything after a homing failure
void haltWithError() {
  disableSteppers();
  while (true);
}

// ================= SETUP (runs once) =================
void setup() {
  pinMode(EN_PIN, OUTPUT);
  enableSteppers();

  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(IR_PIN, INPUT);

  Serial.begin(9600);
  delay(500);

  if (!homeHands()) haltWithError();
  delay(1000);
}

// ================= LOOP =================
void loop() {
  // Homing is done - turn the motors off so they don't hold or get hot
  // disableSteppers();

  // Re-home every HOME_INTERVAL_MS to correct any drift
  static unsigned long lastHomed = millis();
  if (millis() - lastHomed >= HOME_INTERVAL_MS) {
    if (!homeHands()) haltWithError();
    lastHomed = millis();
    delay(1000);
  }

  // Both hands do one full turn together, pause, then reverse
  static bool forward = true;

  digitalWrite(DIR1_PIN, forward ? HIGH : LOW);
  digitalWrite(DIR2_PIN, forward ? HIGH : LOW);
  delayMicroseconds(10);
  moveBoth(STEPS_PER_REV, STEPS_PER_REV);

  delay(1000);
  forward = !forward;
}