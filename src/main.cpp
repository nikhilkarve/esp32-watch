#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <U8g2lib.h>

// --------------------
// Hardware
// --------------------

#define BUTTON_PIN 4

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0,
    /* reset = */ U8X8_PIN_NONE
);

// --------------------
// Timing
// --------------------

const unsigned long DISPLAY_UPDATE_MS = 1000;
const unsigned long DEBOUNCE_MS = 50;
const unsigned long LONG_PRESS_MS = 2000;

// --------------------
// Watch state
// --------------------

enum WatchState {
    IDLE,
    RUNNING,
    PAUSED
};

enum ButtonEvent {
    NONE,
    SHORT_PRESS,
    LONG_PRESS
};

WatchState watchState = IDLE;

// --------------------
// Run timing
// --------------------

// Total time accumulated from previous RUNNING segments.
unsigned long activeTime = 0;

// When the current RUNNING segment started.
unsigned long activeSegmentStart = 0;

// Used to control display refresh rate.
unsigned long lastDisplayUpdate = 0;


// ============================================================
// DISPLAY
// ============================================================

void displayInfo() {

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    switch (watchState) {

        case IDLE:
            u8g2.drawStr(0, 10, "IDLE MODE");
            break;


        case RUNNING: {

            u8g2.drawStr(0, 10, "--GPS COORDINATES--");

            // Time from previous running segments
            // + time spent in the current running segment.
            unsigned long currentRunTime =
                activeTime + (millis() - activeSegmentStart);

            displayRunTime(currentRunTime);

            displayCoordinates();

            break;
        }


        case PAUSED: {

            u8g2.drawStr(0, 10, "RUN PAUSED");

            displayRunTime(activeTime);

            break;
        }
    }

    u8g2.sendBuffer();
}


// ============================================================
// DISPLAY HELPERS
// ============================================================

void displayRunTime(unsigned long timeMs) {

    unsigned long totalSeconds = timeMs / 1000;

    unsigned int minutes = totalSeconds / 60;
    unsigned int seconds = totalSeconds % 60;

    char timeBuffer[20];

    snprintf(
        timeBuffer,
        sizeof(timeBuffer),
        "TIME: %02u:%02u",
        minutes,
        seconds
    );

    u8g2.drawStr(0, 56, timeBuffer);
}


void displayCoordinates() {

    char latBuffer[20];
    char lngBuffer[20];

    if (gps.location.isValid()) {

        char* latValue = dtostrf(
            gps.location.lat(),
            10,
            6,
            latBuffer
        );

        char* lngValue = dtostrf(
            gps.location.lng(),
            10,
            6,
            lngBuffer
        );

        u8g2.drawStr(0, 22, latValue);
        u8g2.drawStr(0, 44, lngValue);

    } else {

        u8g2.drawStr(0, 32, "Locating..");
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

    // Has the electrical state changed?
    if (currentState != lastState) {

        // Ignore changes happening too quickly.
        if (millis() - lastChangeTime < DEBOUNCE_MS) {
            return NONE;
        }

        lastState = currentState;
        lastChangeTime = millis();


        // Button was pressed.
        if (currentState == LOW) {

            pressStartTime = millis();

            return NONE;
        }


        // Button was released.
        unsigned long pressDuration =
            millis() - pressStartTime;

        if (pressDuration >= LONG_PRESS_MS) {
            return LONG_PRESS;
        }

        return SHORT_PRESS;
    }

    return NONE;
}


// ============================================================
// WATCH STATE MACHINE
// ============================================================

void handleButtonEvent(ButtonEvent event) {

    if (event == NONE) {
        return;
    }

    Serial.print("Current state: ");
    Serial.println(watchState);


    switch (watchState) {

        case IDLE:

            if (event == SHORT_PRESS) {

                // Start a brand-new run.
                activeTime = 0;
                activeSegmentStart = millis();

                watchState = RUNNING;
            }

            break;


        case RUNNING:

            if (event == SHORT_PRESS) {

                // Save the time spent in this
                // RUNNING segment.
                activeTime +=
                    millis() - activeSegmentStart;

                watchState = PAUSED;
            }

            break;


        case PAUSED:

            if (event == SHORT_PRESS) {

                // Start a new running segment.
                activeSegmentStart = millis();

                watchState = RUNNING;

            } else if (event == LONG_PRESS) {

                // Throw away the current run.
                activeTime = 0;

                watchState = IDLE;
            }

            break;
    }


    Serial.print("New state: ");
    Serial.println(watchState);

    displayInfo();

    lastDisplayUpdate = millis();
}


// ============================================================
// SETUP
// ============================================================

void setup() {

    Serial.begin(115200);

    u8g2.begin();

    GPSSerial.begin(
        115200,
        SERIAL_8N1,
        2,
        1
    );

    pinMode(
        BUTTON_PIN,
        INPUT_PULLUP
    );

    Serial.println("GPS Watch started.");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

    // Continuously process incoming GPS data.
    while (GPSSerial.available()) {
        gps.encode(GPSSerial.read());
    }


    // Check whether the user pressed the button.
    ButtonEvent event = checkButton();

    // Apply the button event to the watch state machine.
    handleButtonEvent(event);


    // Refresh the OLED once per second.
    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {

        lastDisplayUpdate = millis();

        displayInfo();
    }
}