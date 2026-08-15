#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <U8g2lib.h>

#define BUTTON_PIN 4

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

unsigned long lastUpdate = 0;
const unsigned long UPDATE_TIME = 1000;
// Timing tracking variables
unsigned long totalActiveTime = 0;  // Cumulative time spent running (ms)
unsigned long lastStateChange =
    0;  // Timestamp when state last transitioned (ms)

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

enum WatchState { IDLE, RUNNING, PAUSED };

enum ButtonEvent { NONE, SHORT_PRESS, LONG_PRESS };

WatchState watchState = IDLE;

int counter = 0;

void displayInfo() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  switch (watchState) {
    case IDLE:
      u8g2.drawStr(0, 10, "IDLE MODE");
      break;

    case RUNNING: {
      u8g2.drawStr(0, 10, "--GPS COORDINATES--");
      unsigned long currentRunDuration =
          totalActiveTime + (millis() - lastStateChange);

      // Convert milliseconds to MM:SS format
      unsigned long totalSeconds = currentRunDuration / 1000;
      unsigned int mins = totalSeconds / 60;
      unsigned int secs = totalSeconds % 60;

      char timeBuffer[16];
      snprintf(timeBuffer, sizeof(timeBuffer), "TIME: %02u:%02u", mins, secs);
      u8g2.drawStr(0, 56, timeBuffer);

      char latBuffer[20];
      char lngBuffer[20];

      if (gps.location.isValid()) {
        char* latValue = dtostrf(gps.location.lat(), 10, 6, latBuffer);

        char* lngValue = dtostrf(gps.location.lng(), 10, 6, lngBuffer);

        u8g2.drawStr(0, 22, latValue);
        u8g2.drawStr(0, 44, lngValue);
      } else {
        u8g2.drawStr(0, 32, "Locating..");
      }
      break;
    }

    case PAUSED: {
      u8g2.drawStr(0, 10, "RUN PAUSED");

      unsigned long totalSecondsPaused = totalActiveTime / 1000;
      unsigned int minsPaused = totalSecondsPaused / 60;
      unsigned int secsPaused = totalSecondsPaused % 60;

      char timeBuffer[16];
      snprintf(timeBuffer, sizeof(timeBuffer), "PAUSED: %02u:%02u", minsPaused, secsPaused);
      u8g2.drawStr(0, 32, timeBuffer);
      break;
    }
  }

  u8g2.sendBuffer();
}

ButtonEvent checkButton() {
  static bool lastState = HIGH;
  static unsigned long lastChangeTime = 0;
  static unsigned long pressStartTime = 0;
  const unsigned long DEBOUNCE_MS = 50;

  bool currentState = digitalRead(BUTTON_PIN);

  if (currentState != lastState && (millis() - lastChangeTime) >= DEBOUNCE_MS) {
    lastState = currentState;
    lastChangeTime = millis();

    if (currentState == LOW) {
      pressStartTime = millis();
    } else {
      unsigned long duration = millis() - pressStartTime;

      if (duration >= 2000) {
        return LONG_PRESS;
      } else if (duration >= DEBOUNCE_MS) {
        return SHORT_PRESS;
      }
    }
  }

  return NONE;
}

void checkWatchState(ButtonEvent isPressed) {
  if (isPressed != NONE) {
    Serial.print("Current watch state: ");
    Serial.println(watchState);

    switch (watchState) {
      case IDLE:
        if (isPressed == SHORT_PRESS) {
          totalActiveTime = 0;         // Reset timer for a fresh run
          lastStateChange = millis();  // Mark the start time
          watchState = RUNNING;
        }
        break;

      case RUNNING:
        if (isPressed == SHORT_PRESS) {
          totalActiveTime +=
              millis() - lastStateChange;  // Lock in active time spent
          watchState = PAUSED;
        }
        break;

      case PAUSED:
        if (isPressed == SHORT_PRESS) {
          lastStateChange = millis();  // Reset segment start time for resume
          watchState = RUNNING;
        } else if (isPressed == LONG_PRESS) {
          totalActiveTime = 0;  // Reset timer when quitting
          watchState = IDLE;
        }
        break;
    }

    Serial.print("Watch state changed to: ");
    Serial.println(watchState);

    displayInfo();
    lastUpdate = millis();
  }
}

void setup() {
  Serial.begin(115200);
  u8g2.begin();
  GPSSerial.begin(115200, SERIAL_8N1, 2, 1);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Listening for GPS...");
}

void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }
  ButtonEvent isPressed = checkButton();
  checkWatchState(isPressed);
  if (millis() - lastUpdate >= UPDATE_TIME) {
    lastUpdate = millis();
    displayInfo();
  }
}
