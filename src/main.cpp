#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <U8g2lib.h>

HardwareSerial GPSSerial(1); // grab UART controller 1 — UART0 is already the USB/debug channel
TinyGPSPlus gps;
unsigned long lastUpdate = 0;
const unsigned long UPDATE_TIME = 1000;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

void displayInfo() {
  u8g2.clearBuffer();                 // wipe the in-memory image
  u8g2.setFont(u8g2_font_ncenB08_tr); // pick a font
  u8g2.drawStr(0, 10, "--GPS COORDINATES--");
  char latBuffer[12];
  char lngBuffer[12];
  if(gps.location.isValid()) {
    char* latValue = dtostrf(gps.location.lat(), 10, 6, latBuffer);
    char* lngValue = dtostrf(gps.location.lng(), 10, 6, lngBuffer);
    u8g2.drawStr(0, 22, latValue);
    u8g2.drawStr(0, 44, lngValue);
  } else {
    u8g2.drawStr(0, 32, "Locating..");
    String totalSats = String(gps.satellites.value());
    u8g2.drawStr(0, 44, totalSats.c_str());
  }

  u8g2.sendBuffer();
}

void setup() {
  Serial.begin(115200);                      // USB <-> your laptop
  u8g2.begin();
  GPSSerial.begin(115200, SERIAL_8N1, 2, 1); // GPS <-> ESP32 (rx=GPIO2, tx=GPIO1)
  Serial.println("Listening for GPS...");
}

void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  if (millis() - lastUpdate >= UPDATE_TIME) {
    lastUpdate = millis();
    displayInfo();
  }
}