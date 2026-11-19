// ESP32 I2C master - sends commands to the two clock Megas.
// Test it by typing commands in the Serial Monitor (115200 baud, "Newline" line ending):
//   h                      home all clocks on both Megas
//   e 1  /  e 0            motors on / off
//   m <mega> <motor> <angle> [ccw]
//                          move a hand, e.g. "m 0 3 180" or "m 1 0 90.5 1"
//                          mega 0-1, motor 0-5, angle 0-359.9, ccw 1 = counter-clockwise
//   s                      status of both Megas

#include <Wire.h>

// ================= PINS =================
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// ================= MEGAS =================
const int NUM_MEGAS = 2;
const uint8_t MEGA_ADDR[NUM_MEGAS] = {0x10, 0x11};

// Must match mega_clock.ino
const uint8_t CMD_HOME   = 0x01;
const uint8_t CMD_MOVE   = 0x02;
const uint8_t CMD_ENABLE = 0x03;

// ================= I2C HELPERS =================
bool sendCmd(uint8_t addr, const uint8_t *data, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(data, len);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.print("Mega 0x");
    Serial.print(addr, HEX);
    Serial.print(" didn't respond (error ");
    Serial.print(err);
    Serial.println(")");
  }
  return err == 0;
}

void homeAll() {
  uint8_t data[] = {CMD_HOME};
  for (int i = 0; i < NUM_MEGAS; i++) sendCmd(MEGA_ADDR[i], data, sizeof(data));
}

void setEnabled(bool on) {
  uint8_t data[] = {CMD_ENABLE, (uint8_t)(on ? 1 : 0)};
  for (int i = 0; i < NUM_MEGAS; i++) sendCmd(MEGA_ADDR[i], data, sizeof(data));
}

void moveHand(int mega, int motor, float angle, bool ccw) {
  if (mega < 0 || mega >= NUM_MEGAS || motor < 0 || motor > 5) {
    Serial.println("Bad mega or motor number");
    return;
  }
  while (angle < 0) angle += 360.0;
  uint16_t tenths = (uint16_t)(angle * 10.0 + 0.5) % 3600;

  uint8_t data[] = {CMD_MOVE, (uint8_t)motor,
                    (uint8_t)(tenths >> 8), (uint8_t)(tenths & 0xFF),
                    (uint8_t)(ccw ? 1 : 0)};
  sendCmd(MEGA_ADDR[mega], data, sizeof(data));
}

// Reads [busy, homed mask, error mask] from a Mega
bool readStatus(uint8_t addr, uint8_t &busy, uint8_t &homed, uint8_t &errors) {
  if (Wire.requestFrom((int)addr, 3) != 3) return false;
  busy   = Wire.read();
  homed  = Wire.read();
  errors = Wire.read();
  return true;
}

void printStatus() {
  for (int i = 0; i < NUM_MEGAS; i++) {
    uint8_t busy, homed, errors;
    Serial.print("Mega ");
    Serial.print(i);
    Serial.print(" (0x");
    Serial.print(MEGA_ADDR[i], HEX);
    Serial.print("): ");

    if (!readStatus(MEGA_ADDR[i], busy, homed, errors)) {
      Serial.println("no response");
      continue;
    }
    Serial.print(busy ? "busy" : "idle");
    for (int c = 0; c < 3; c++) {
      Serial.print(" | clock ");
      Serial.print(c);
      if (errors & (1 << c))     Serial.print(" ERROR");
      else if (homed & (1 << c)) Serial.print(" homed");
      else                       Serial.print(" not homed");
    }
    Serial.println();
  }
}

// ================= SERIAL COMMANDS =================
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  String args = line.substring(1);
  args.trim();

  if (cmd == 'h') {
    Serial.println("Homing...");
    homeAll();
  }
  else if (cmd == 'e') {
    setEnabled(args.toInt() != 0);
  }
  else if (cmd == 'm') {
    int mega = 0, motor = 0, ccw = 0;
    float angle = 0;
    int n = sscanf(args.c_str(), "%d %d %f %d", &mega, &motor, &angle, &ccw);
    if (n < 3) {
      Serial.println("Usage: m <mega> <motor> <angle> [ccw]");
      return;
    }
    moveHand(mega, motor, angle, ccw != 0);
  }
  else if (cmd == 's') {
    printStatus();
  }
  else {
    Serial.println("Commands: h, e 1/0, m <mega> <motor> <angle> [ccw], s");
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(1000);                      // give the Megas time to boot

  Serial.println("ESP32 master ready");
  printStatus();
}

// ================= LOOP =================
void loop() {
  if (Serial.available()) {
    handleLine(Serial.readStringUntil('\n'));
  }
}
