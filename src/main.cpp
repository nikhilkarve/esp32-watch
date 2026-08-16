#include <Arduino.h>
#include <LittleFS.h>
#include <TinyGPSPlus.h>
#include <U8g2lib.h>

// --------------------
// Hardware
// --------------------

#define BUTTON_PIN 4

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset = */ U8X8_PIN_NONE);

// --------------------
// Timing
// --------------------

const unsigned long DISPLAY_UPDATE_MS = 1000;
const unsigned long DEBOUNCE_MS = 50;
const unsigned long LONG_PRESS_MS = 2000;
const unsigned long MIN_LOG_INTERVAL_MS =
    3000;  // Throttles point writes to 1 per 3s

// --------------------
// Watch state
// --------------------

enum WatchState { IDLE, RUNNING, PAUSED };
enum ButtonEvent { NONE, SHORT_PRESS, LONG_PRESS };

WatchState watchState = IDLE;

// --------------------
// Run timing & distance
// --------------------

unsigned long activeTime = 0;
unsigned long activeSegmentStart = 0;
unsigned long lastDisplayUpdate = 0;

double totalDistanceMeters = 0.0;
double lastLat = 0.0;
double lastLng = 0.0;
bool hasFirstFix = false;

// --------------------
// Storage & GPX State
// --------------------

File gpxFile;
bool isGpxActive = false;
unsigned long lastGpxFlush = 0;
unsigned long lastLogTime = 0;

// ============================================================
// GPX LOGGING HELPERS
// ============================================================

String generateGpxFilename() {
  if (gps.date.isValid() && gps.time.isValid()) {
    char filename[32];
    snprintf(filename, sizeof(filename), "/run_%04d%02d%02d_%02d%02d%02d.gpx",
             gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(),
             gps.time.minute(), gps.time.second());
    return String(filename);
  }
  return "/run_" + String(millis()) + ".gpx";
}

void startGpxFile() {
  if (isGpxActive) return;

  String path = generateGpxFilename();
  Serial.print("Creating GPX file: ");
  Serial.println(path);

  gpxFile = LittleFS.open(path, "w");
  if (!gpxFile) {
    Serial.println("Failed to open GPX file for writing!");
    return;
  }

  // GPX 1.1 Header
  gpxFile.println(F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"));
  gpxFile.println(F("<gpx version=\"1.1\" creator=\"ESP32-S3 Running Watch\""));
  gpxFile.println(F("     xmlns=\"http://www.topografix.com/GPX/1/1\">"));
  gpxFile.println(F("  <metadata>"));
  gpxFile.print(F("    <time>"));

  if (gps.date.isValid() && gps.time.isValid()) {
    char timeBuf[30];
    snprintf(timeBuf, sizeof(timeBuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(),
             gps.time.minute(), gps.time.second());
    gpxFile.print(timeBuf);
  } else {
    gpxFile.print(F("1970-01-01T00:00:00Z"));
  }

  gpxFile.println(F("</time>"));
  gpxFile.println(F("  </metadata>"));
  gpxFile.println(F("  <trk>"));
  gpxFile.println(F("    <name>Running Activity</name>"));
  gpxFile.println(F("    <trkseg>"));

  gpxFile.flush();
  isGpxActive = true;
}

void logGpxTrackpoint(double lat, double lng, double ele, double speedKmph) {
  if (!isGpxActive || !gpxFile) return;

  gpxFile.printf("      <trkpt lat=\"%.6f\" lon=\"%.6f\">\n", lat, lng);

  if (gps.altitude.isValid()) {
    gpxFile.printf("        <ele>%.2f</ele>\n", ele);
  }

  if (gps.time.isValid() && gps.date.isValid()) {
    char timeBuf[30];
    snprintf(timeBuf, sizeof(timeBuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(),
             gps.time.minute(), gps.time.second());
    gpxFile.printf("        <time>%s</time>\n", timeBuf);
  }

  gpxFile.println(F("      </trkpt>"));

  // Periodically flush buffer to guard against unexpected power loss
  if (millis() - lastGpxFlush > 10000) {
    gpxFile.flush();
    lastGpxFlush = millis();
  }
}

void closeGpxFile() {
  if (!isGpxActive || !gpxFile) return;

  if (watchState == RUNNING) {
    gpxFile.println(F("    </trkseg>"));
  }

  gpxFile.println(F("  </trk>"));
  gpxFile.println(F("</gpx>"));

  gpxFile.flush();
  gpxFile.close();
  isGpxActive = false;
  Serial.println("GPX file successfully written and closed.");
}

// ============================================================
// DISPLAY HELPERS
// ============================================================

void displayRunTime(unsigned long timeMs) {
  unsigned long totalSeconds = timeMs / 1000;
  unsigned int minutes = totalSeconds / 60;
  unsigned int seconds = totalSeconds % 60;

  char timeBuffer[20];
  snprintf(timeBuffer, sizeof(timeBuffer), "TIME: %02u:%02u", minutes, seconds);
  u8g2.drawStr(0, 56, timeBuffer);
}

void displayDistance() {
  char distBuffer[20];
  if (totalDistanceMeters >= 1000.0) {
    double km = totalDistanceMeters / 1000.0;
    snprintf(distBuffer, sizeof(distBuffer), "DIST: %.2f km", km);
  } else {
    snprintf(distBuffer, sizeof(distBuffer), "DIST: %.0f m",
             totalDistanceMeters);
  }
  u8g2.drawStr(0, 42, distBuffer);
}

void displayCoordinates() {
  char latBuffer[20];
  char lngBuffer[20];

  if (gps.location.isValid()) {
    dtostrf(gps.location.lat(), 10, 6, latBuffer);
    dtostrf(gps.location.lng(), 10, 6, lngBuffer);

    u8g2.drawStr(0, 20, latBuffer);
    u8g2.drawStr(0, 31, lngBuffer);
  } else {
    u8g2.drawStr(0, 26, "   sg..");
  }
}

// ============================================================
// DISPLAY
// ============================================================

void displayInfo() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  switch (watchState) {
    case IDLE:
      u8g2.drawStr(0, 10, "IDLE MODE");
      u8g2.drawStr(0, 30, "Press button");
      u8g2.drawStr(0, 42, "to start run");
      break;

    case RUNNING: {
      u8g2.drawStr(0, 10, "-- RUNNING --");

      unsigned long currentRunTime =
          activeTime + (millis() - activeSegmentStart);
      displayRunTime(currentRunTime);
      displayDistance();
      displayCoordinates();
      break;
    }

    case PAUSED: {
      u8g2.drawStr(0, 10, "-- PAUSED --");
      displayRunTime(activeTime);
      displayDistance();
      u8g2.drawStr(0, 26, "Hold to stop");
      break;
    }
  }

  u8g2.sendBuffer();
}

// ============================================================
// DISTANCE & LOGGING LOGIC
// ============================================================

void updateDistanceAndLog() {
  if (watchState == RUNNING && gps.location.isUpdated() &&
      gps.location.isValid()) {
    double currentHdop = gps.hdop.hdop();

    // Reject fixes too imprecise to trust
    if (currentHdop > 2.5) {
      return;
    }

    double currentLat = gps.location.lat();
    double currentLng = gps.location.lng();
    double currentEle = gps.altitude.meters();
    double currentSpeed = gps.speed.kmph();

    if (!hasFirstFix) {
      lastLat = currentLat;
      lastLng = currentLng;
      hasFirstFix = true;

      logGpxTrackpoint(currentLat, currentLng, currentEle, currentSpeed);
      lastLogTime = millis();
    } else {
      double deltaMeters = TinyGPSPlus::distanceBetween(lastLat, lastLng,
                                                        currentLat, currentLng);

      if (deltaMeters > 2.5 && currentSpeed > 1.2) {
        totalDistanceMeters += deltaMeters;
        lastLat = currentLat;
        lastLng = currentLng;

        // Decoupled logging: only write point every 3 seconds
        if (millis() - lastLogTime >= MIN_LOG_INTERVAL_MS) {
          logGpxTrackpoint(currentLat, currentLng, currentEle, currentSpeed);
          lastLogTime = millis();
        }
      }
    }
  }
}

// ============================================================
// BUTTON
// ============================================================

ButtonEvent checkButton() {
  static bool lastState = HIGH;
  static unsigned long lastChangeTime = 0;
  static unsigned long pressStartTime = 0;

  bool currentState = digitalRead(BUTTON_PIN);

  if (currentState != lastState) {
    if (millis() - lastChangeTime < DEBOUNCE_MS) {
      return NONE;
    }

    lastState = currentState;
    lastChangeTime = millis();

    if (currentState == LOW) {
      pressStartTime = millis();
      return NONE;
    }

    unsigned long pressDuration = millis() - pressStartTime;
    if (pressDuration >= LONG_PRESS_MS) {
      return LONG_PRESS;
    }

    return SHORT_PRESS;
  }

  return NONE;
}

void finishRun() {
  closeGpxFile();

  u8g2.clearBuffer();
  u8g2.drawStr(0, 30, "Run Saved!");
  u8g2.sendBuffer();
  delay(1200);
}

// ============================================================
// WATCH STATE MACHINE
// ============================================================

void handleButtonEvent(ButtonEvent event) {
  if (event == NONE) {
    return;
  }

  switch (watchState) {
    case IDLE:
      if (event == SHORT_PRESS) {
        activeTime = 0;
        activeSegmentStart = millis();
        totalDistanceMeters = 0.0;
        hasFirstFix = false;

        startGpxFile();
        watchState = RUNNING;
      }
      break;

    case RUNNING:
      if (event == SHORT_PRESS) {
        activeTime += millis() - activeSegmentStart;

        // Close active GPX segment
        if (isGpxActive && gpxFile) {
          gpxFile.println(F("    </trkseg>"));
          gpxFile.flush();
        }

        watchState = PAUSED;
      }
      break;

    case PAUSED:
      if (event == SHORT_PRESS) {
        activeSegmentStart = millis();
        hasFirstFix = false;

        // Open new GPX segment for resumed run
        if (isGpxActive && gpxFile) {
          gpxFile.println(F("    <trkseg>"));
          gpxFile.flush();
        }

        watchState = RUNNING;
      } else if (event == LONG_PRESS) {
        finishRun();
        activeTime = 0;
        totalDistanceMeters = 0.0;
        watchState = IDLE;
      }
      break;
  }

  displayInfo();
  lastDisplayUpdate = millis();
}

// ============================================================
// SERIAL COMMAND INSPECTOR
// ============================================================

void handleSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "LIST") {
    Serial.println("--- LittleFS Files ---");
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
      Serial.print("  ");
      Serial.print(file.name());
      Serial.print(" \t (");
      Serial.print(file.size());
      Serial.println(" bytes)");
      file = root.openNextFile();
    }
    Serial.println("----------------------");
    Serial.printf("Used: %u / %u bytes (%.1f%%)\n", LittleFS.usedBytes(),
                  LittleFS.totalBytes(),
                  100.0 * LittleFS.usedBytes() / LittleFS.totalBytes());

  } else if (cmd.startsWith("DUMP ")) {
    String filename = cmd.substring(5);
    filename.trim();
    if (!filename.startsWith("/")) filename = "/" + filename;

    Serial.print("Dumping file: ");
    Serial.println(filename);

    File f = LittleFS.open(filename, "r");
    if (!f) {
      Serial.println("ERROR: File not found or failed to open.");
      return;
    }

    Serial.println("=== FILE START ===");
    while (f.available()) {
      Serial.write(f.read());
    }
    Serial.println("\n=== FILE END ===");
    f.close();

  } else if (cmd.startsWith("DELETE ")) {
    String filename = cmd.substring(7);
    filename.trim();
    if (!filename.startsWith("/")) filename = "/" + filename;

    if (LittleFS.remove(filename)) {
      Serial.print("Deleted: ");
      Serial.println(filename);
    } else {
      Serial.println("ERROR: delete failed.");
    }
  } else if (cmd.startsWith("DOWNLOAD ")) {
    String filename = cmd.substring(9);
    filename.trim();
    if (!filename.startsWith("/")) filename = "/" + filename;

    File f = LittleFS.open(filename, "r");
    if (!f) {
      Serial.println("ERROR");
      return;
    }

    Serial.println("BEGIN_FILE");
    while (f.available()) {
      Serial.write(f.read());
    }
    Serial.println("\nEND_FILE");
    f.close();
  }
}

// ============================================================
// SETUP & LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  u8g2.begin();

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed!");
  } else {
    Serial.println("LittleFS Mounted Successfully.");
  }

  GPSSerial.begin(115200, SERIAL_8N1, 2, 1);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("GPS Watch started.");
}

void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  updateDistanceAndLog();
  ButtonEvent event = checkButton();
  handleButtonEvent(event);

  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    lastDisplayUpdate = millis();
    displayInfo();
  }

  handleSerialCommands();
}