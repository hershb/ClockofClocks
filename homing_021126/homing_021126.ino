// ================= PINS =================
// Motor 1
const int STEP1_PIN = 9;
const int DIR1_PIN  = 8;

// Motor 2
const int STEP2_PIN = 11;
const int DIR2_PIN  = 10;

// Enable (one per TMC2208) - LOW = motor on, HIGH = motor off
const int EN_PIN_1 = 6;
const int EN_PIN_2 = 7;

// Sensor
const int IR_PIN   = A0;
const int DETECTED = HIGH;          // sensor reads 1 when blocked

// ================= MOTOR SETUP =================
const long STEPS_PER_REV = 180L * 128;   // steps per turn of the hand

const float M1_DEGREES = 92.0;
const float M2_DEGREES = 48.0;

const long M1_STEPS = (long)(STEPS_PER_REV * M1_DEGREES / 360.0 + 0.5);
const long M2_STEPS = (long)(STEPS_PER_REV * M2_DEGREES / 360.0 + 0.5);

// ================= TIMING =================
const unsigned int STEP_DELAY_US = 100;    // running speed (smaller = faster)
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

// ================= STARTUP CLEARING =================
// Makes sure neither hand is covering the sensor.
// Returns true if the sensor ends up clear.
bool clearSensor() {
  if (digitalRead(IR_PIN) != DETECTED) {
    Serial.println("Sensor clear at startup");
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
  digitalWrite(EN_PIN_1, LOW);
  digitalWrite(EN_PIN_2, LOW);
  delay(5);                         // let the drivers power up the coils
}

void disableSteppers() {
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);
}

// ================= SETUP (runs once) =================
void setup() {
  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);
  enableSteppers();

  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(IR_PIN, INPUT);

  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);

  Serial.begin(9600);
  delay(500);

  // 1. Make sure neither hand is over the sensor
  if (!clearSensor()) {
    Serial.println("ERROR: sensor still blocked - check for obstruction");
    disableSteppers();
    while (true);
  }

  // 2. Hand 1: run to sensor, then move to its position
  Serial.println("Homing hand 1...");
  if (!runToSensor(STEP1_PIN)) {
    Serial.println("ERROR: hand 1 didn't reach the sensor");
    disableSteppers();
    while (true);
  }
  moveSteps(STEP1_PIN, M1_STEPS);

  // 3. Hand 2: run to sensor, then move to its position
  Serial.println("Homing hand 2...");
  if (!runToSensor(STEP2_PIN)) {
    Serial.println("ERROR: hand 2 didn't reach the sensor");
    disableSteppers();
    while (true);
  }
  moveSteps(STEP2_PIN, M2_STEPS);

  Serial.println("Homing done");
}

// ================= LOOP =================
void loop() {
  // Homing is done - turn the motors off so they don't hold or get hot
  disableSteppers();
}