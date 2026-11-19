// Single stepper test - homes one hand to the IR sensor, then swings it
// back and forth.

// ================= PINS =================

const int DIR_PIN  = 2;
const int STEP_PIN = 3;
const int EN_PIN   = 53;             // LOW = motor on, HIGH = motor off

const int IR_PIN   = A0;
const int DETECTED = HIGH;          // sensor reads 1 when the hand reflects

// ================= MOTOR SETUP =================
const long STEPS_PER_REV = 180L * 64;    // steps per turn of the hand (11,520)

// Doubled from 123 to keep the same movement now that STEPS_PER_REV is correct
const float HOME_DEGREES  = 246.0;  // position after homing (from the sensor trigger point)
const float SWING_DEGREES = 246.0;  // how far it swings back and forth in loop()

const long HOME_STEPS  = (long)(STEPS_PER_REV * HOME_DEGREES / 360.0 + 0.5);
const long SWING_STEPS = (long)(STEPS_PER_REV * SWING_DEGREES / 360.0 + 0.5);

// ================= TIMING =================
const unsigned int STEP_DELAY_US = 50;    // running speed (smaller = faster)
const unsigned int HOME_DELAY_US = 200;   // clearing speed
const unsigned long PAUSE_MS     = 500;   // pause between swings

const long HOME_MAX_STEPS = STEPS_PER_REV * 2;   // give up after 2 turns
const long CLEAR_MARGIN   = 100;                 // extra steps past the sensor when clearing

// ================= MOVEMENT FUNCTIONS =================
void stepOnce(unsigned int delayUs) {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delayUs);
}

void moveSteps(long steps) {
  for (long i = 0; i < steps; i++) stepOnce(STEP_DELAY_US);
}

void setDirection(bool cw) {
  digitalWrite(DIR_PIN, cw ? HIGH : LOW);
  delayMicroseconds(10);
}

// ================= ENABLE =================
void enableStepper() {
  digitalWrite(EN_PIN, LOW);
  delay(5);                         // let the driver power up the coils
}

void disableStepper() {
  digitalWrite(EN_PIN, HIGH);
}

void halt(const char *msg) {
  Serial.print("ERROR: ");
  Serial.println(msg);
  disableStepper();
  while (true);
}

// ================= HOMING =================
// Moves the hand off the sensor if needed, then runs CW until the sensor
// triggers, so it always approaches from the same side.
void home() {
  setDirection(true);
  long count = 0;

  if (digitalRead(IR_PIN) == DETECTED) {
    Serial.println("Hand on sensor - moving off");
    while (digitalRead(IR_PIN) == DETECTED) {
      stepOnce(HOME_DELAY_US);
      if (++count > HOME_MAX_STEPS) halt("sensor still blocked - check for obstruction");
    }
    for (long i = 0; i < CLEAR_MARGIN; i++) stepOnce(HOME_DELAY_US);
  }

  Serial.println("Homing...");
  count = 0;
  while (digitalRead(IR_PIN) != DETECTED) {
    stepOnce(STEP_DELAY_US);
    if (++count > HOME_MAX_STEPS) halt("hand didn't reach the sensor");
  }

  moveSteps(HOME_STEPS);
  Serial.println("Homing done");
}

// ================= SETUP (runs once) =================
void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(IR_PIN, INPUT);

  Serial.begin(9600);
  delay(500);

  enableStepper();
  home();
}

// ================= LOOP =================
void loop() {
  setDirection(false);              // back
  moveSteps(SWING_STEPS);
  delay(PAUSE_MS);

  setDirection(true);               // forward
  moveSteps(SWING_STEPS);
  delay(PAUSE_MS);
}
