// dir_test - standalone DIR test for one Mega (24 hands).
// Odd hands (1, 3, 5... = top hands) turn one turn CCW while even hands
// (2, 4, 6... = bottom hands) turn one turn CW. Pause 2 s, then swap, and repeat.
// Any hand going the same way as its partner on the same clock has a DIR problem.
// No homing, no sensors, no EEPROM - only STEP, DIR and EN.
// Serial Monitor 9600: prints each DIR pin and the level it reads back.

const int NUM_HANDS = 24;
const uint8_t STEP_PINS[NUM_HANDS] = {  9, 11, 13,  3,  5,  7, 15, 17, 31, 33, 35, 37, 23, 25, 27, 29, 53, 51, 49, 47, 39, 41, 43, 45 };
const uint8_t DIR_PINS[NUM_HANDS]  = {  8, 10, 12,  2,  4,  6, 14, 16, 30, 32, 34, 36, 22, 24, 26, 28, 52, 50, 48, 46, 38, 40, 42, 44 };
const int EN_PIN = 19;                 // LOW = drivers on
const int CW = LOW;                    // DIR level that turns a hand CW (same as clock24)
const long STEPS_PER_REV = 11520;
const unsigned int STEP_DELAY_US = 200;

void setDirs(bool oddCCW) {
  for (int m = 0; m < NUM_HANDS; m++) {
    bool odd = (m % 2 == 0);           // index 0 = hand 1
    bool ccw = odd ? oddCCW : !oddCCW;
    digitalWrite(DIR_PINS[m], ccw ? !CW : CW);
  }
  delayMicroseconds(20);
  for (int m = 0; m < NUM_HANDS; m++) {
    bool odd = (m % 2 == 0);
    bool ccw = odd ? oddCCW : !oddCCW;
    Serial.print("Hand ");
    Serial.print(m + 1);
    Serial.print(m + 1 < 10 ? "  " : " ");
    Serial.print(ccw ? "CCW" : "CW ");
    Serial.print("  DIR pin ");
    Serial.print(DIR_PINS[m]);
    Serial.print(" set ");
    Serial.print((ccw ? !CW : CW) ? "HIGH" : "LOW ");
    Serial.print("  reads ");
    Serial.println(digitalRead(DIR_PINS[m]) ? "HIGH" : "LOW");   // LOW/HIGH mismatch = pin pulled by something
  }
}

void turnAll() {
  for (long i = 0; i < STEPS_PER_REV; i++) {
    for (int m = 0; m < NUM_HANDS; m++) digitalWrite(STEP_PINS[m], HIGH);
    delayMicroseconds(3);
    for (int m = 0; m < NUM_HANDS; m++) digitalWrite(STEP_PINS[m], LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  for (int m = 0; m < NUM_HANDS; m++) {
    pinMode(STEP_PINS[m], OUTPUT);
    pinMode(DIR_PINS[m], OUTPUT);
    digitalWrite(STEP_PINS[m], LOW);
  }
  digitalWrite(EN_PIN, LOW);
  Serial.println("dir_test: odd hands CCW / even hands CW, then swap. Repeats.");
}

void loop() {
  Serial.println("\n--- odd hands CCW, even hands CW ---");
  setDirs(true);
  turnAll();
  delay(2000);
  Serial.println("\n--- odd hands CW, even hands CCW ---");
  setDirs(false);
  turnAll();
  delay(2000);
}
