// ESP32 time master for the Clock of Clocks.
// Gets the time from the internet (NTP), and every time the minute changes
// sends the minute-units digit to the minute-units board (Mega 1, I2C 0x10),
// which draws it on both of its Megas. Also blinks a colon LED once a second,
// locked to the real second, like the flashing colon on a digital clock.
//
// WiFi name and password go in secrets.h (copy secrets_example.h to secrets.h
// - secrets.h is not committed to GitHub).
//
// Serial Monitor 115200 baud, "Newline". Commands:
//   t          print the time and what was last sent
//   d 5        send digit 5 now (test)
//   c h        send a command letter to the Mega (h = home, f = free ...)
//   s          send the current minute digit again
//
// Wiring: SDA 21, SCL 22 -> BSS138 level shifter -> Mega 1 pins 20 (SDA), 21 (SCL),
//         plus GND. Colon LED: LED_PIN -> 330 ohm -> LED -> GND.

#include <WiFi.h>
#include <Wire.h>
#include <time.h>
#include <sys/time.h>
#include "secrets.h"                // const char *WIFI_SSID, *WIFI_PASSWORD

// ================= SETTINGS =================
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int LED_PIN = 2;              // colon LED (2 = the on-board LED on most ESP32 DevKits)
const uint8_t MINUTE_UNITS_ADDR = 0x10;

// Time zone (POSIX format, handles daylight saving). Melbourne / Sydney:
const char *TIME_ZONE = "AEST-10AEDT,M10.1.0,M4.1.0/3";
const char *NTP_1 = "pool.ntp.org";
const char *NTP_2 = "time.google.com";

const unsigned int LED_ON_MS = 500; // colon on for the first half of every second

// ================= I2C =================
bool sendToMega(uint8_t addr, char type, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)type);
  Wire.write(value);
  uint8_t err = Wire.endTransmission();
  if (err) {
    Serial.printf("Mega 0x%02X didn't answer (error %u)\n", addr, err);
  }
  return err == 0;
}

// ================= TIME =================
bool timeValid = false;
int lastMinute = -1;                // minute whose digit has been sent
int sentDigit = -1;

bool getTime(struct tm &t) {
  if (!getLocalTime(&t, 0)) return false;
  return t.tm_year > (2024 - 1900); // not synced yet = 1970
}

void printTime() {
  struct tm t;
  if (!getTime(t)) { Serial.println("Time not synced yet"); return; }
  char buf[40];
  strftime(buf, sizeof(buf), "%a %d %b %Y %H:%M:%S %Z", &t);
  Serial.print(buf);
  Serial.print("   last digit sent: ");
  Serial.println(sentDigit);
}

// Called often: sends the minute-units digit when the minute changes. If the
// Mega didn't answer, lastMinute is not updated, so it tries again next time.
void updateClock() {
  struct tm t;
  if (!getTime(t)) return;
  if (!timeValid) { timeValid = true; Serial.print("Time synced: "); printTime(); }
  if (t.tm_min == lastMinute) return;

  static unsigned long lastTry = 0;
  if (millis() - lastTry < 1000) return;   // retry at most once a second
  lastTry = millis();

  int digit = t.tm_min % 10;
  if (sendToMega(MINUTE_UNITS_ADDR, 'D', digit)) {
    lastMinute = t.tm_min;
    sentDigit = digit;
    Serial.printf("%02d:%02d  sent minute digit %d\n", t.tm_hour, t.tm_min, digit);
  }
}

// Colon LED locked to the real second (uses the sub-second part of the time)
void updateLed() {
  if (!timeValid) {                  // not synced: short blink every 2 s
    digitalWrite(LED_PIN, (millis() % 2000) < 100);
    return;
  }
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  digitalWrite(LED_PIN, (tv.tv_usec / 1000) < LED_ON_MS);
}

// ================= WIFI =================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  static unsigned long lastTry = 0;
  if (lastTry && millis() - lastTry < 10000) return;   // keep trying every 10 s
  lastTry = millis();
  Serial.print("Connecting to WiFi ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ================= SERIAL =================
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  char cmd = line.charAt(0);
  String arg = line.substring(1);
  arg.trim();

  if (cmd == 't') printTime();
  else if (cmd == 'd' && arg.length()) {
    int d = arg.toInt();
    if (d >= 0 && d <= 9 && sendToMega(MINUTE_UNITS_ADDR, 'D', d)) Serial.printf("Sent digit %d\n", d);
  }
  else if (cmd == 'c' && arg.length()) {
    if (sendToMega(MINUTE_UNITS_ADDR, 'C', arg.charAt(0))) Serial.printf("Sent command %c\n", arg.charAt(0));
  }
  else if (cmd == 's') lastMinute = -1;             // updateClock sends it again
  else Serial.println("Commands: t, d <0-9>, c <letter>, s");
}

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("ESP32 time master");
  configTzTime(TIME_ZONE, NTP_1, NTP_2);   // SNTP keeps the clock synced in the background
  connectWiFi();
}

void loop() {
  connectWiFi();
  updateClock();
  updateLed();
  if (Serial.available()) handleLine(Serial.readStringUntil('\n'));
}
