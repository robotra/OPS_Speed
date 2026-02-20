// Particle Boron - Omnipresense Speed Sensor UART Receiver
// Receives speed data via Serial1 (hardware UART)

#include "Particle.h"

#define SENSOR_BAUD 9600
#define BUFFER_SIZE 64

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

// Connection/sleep behavior copied from fogsensor state-machine approach
const std::chrono::milliseconds connectMaxTime = 6min;
const std::chrono::milliseconds cloudMinTime = 10s;
const std::chrono::seconds sleepTime = 5min;
const std::chrono::milliseconds firmwareUpdateMaxTime = 2min;

enum State {
    STATE_WAIT_CONNECTED = 0,
    STATE_PUBLISH,
    STATE_PRE_SLEEP,
    STATE_SLEEP,
    STATE_FIRMWARE_UPDATE
};

State state = STATE_WAIT_CONNECTED;
unsigned long stateTime = 0;
bool firmwareUpdateInProgress = false;

char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;
float lastSpeedMph = 0.0f;
bool speedValid = false;

void readSensorAndPublish();
void processSpeedData(const char* data);
void firmwareUpdateHandler(system_event_t event, int param);

void setup() {
    Serial.begin(9600);
    Serial1.begin(SENSOR_BAUD);

    waitFor(Serial.isConnected, 10000);
    Serial.println("Boron Speed Sensor Receiver Started");

    System.on(firmware_update, firmwareUpdateHandler);

    Cellular.on();
    Particle.connect();
    stateTime = millis();
}

void loop() {
    switch (state) {
    case STATE_WAIT_CONNECTED:
        if (Particle.connected()) {
            Log.info("connected to the cloud in %lu ms", millis() - stateTime);
            state = STATE_PUBLISH;
            stateTime = millis();
        }
        else if (millis() - stateTime >= connectMaxTime.count()) {
            Log.info("failed to connect, going to sleep");
            state = STATE_SLEEP;
        }
        break;

    case STATE_PUBLISH:
        readSensorAndPublish();

        if (millis() - stateTime < cloudMinTime.count()) {
            Log.info("waiting %lu ms before sleeping", cloudMinTime.count() - (millis() - stateTime));
            state = STATE_PRE_SLEEP;
        }
        else {
            state = STATE_SLEEP;
        }
        break;

    case STATE_PRE_SLEEP:
        if (millis() - stateTime >= cloudMinTime.count()) {
            state = STATE_SLEEP;
        }
        break;

    case STATE_SLEEP:
        if (firmwareUpdateInProgress) {
            Log.info("firmware update detected");
            state = STATE_FIRMWARE_UPDATE;
            stateTime = millis();
            break;
        }

        Log.info("going to sleep for %ld seconds", (long)sleepTime.count());

#if HAL_PLATFORM_NRF52840
        System.sleep(WKP, RISING, sleepTime);
        System.reset();
#else
        System.sleep(SLEEP_MODE_DEEP, sleepTime);
#endif
        break;

    case STATE_FIRMWARE_UPDATE:
        if (!firmwareUpdateInProgress) {
            Log.info("firmware update completed");
            state = STATE_SLEEP;
        }
        else if (millis() - stateTime >= firmwareUpdateMaxTime.count()) {
            Log.info("firmware update timed out");
            state = STATE_SLEEP;
        }
        break;
    }
}

void readSensorAndPublish() {
    // Read UART for a short window and process one or more complete lines.
    const unsigned long readWindowMs = 2000;
    unsigned long readStart = millis();

    while (millis() - readStart < readWindowMs) {
        while (Serial1.available()) {
            char inChar = Serial1.read();

            if (inChar == '\n') {
                rxBuffer[bufferIndex] = '\0';
                processSpeedData(rxBuffer);
                bufferIndex = 0;
            }
            else if (inChar != '\r') {
                if (bufferIndex < BUFFER_SIZE - 1) {
                    rxBuffer[bufferIndex++] = inChar;
                }
            }
        }
        delay(10);
    }

    if (speedValid && Particle.connected()) {
        Particle.publish("speed_reading", String(lastSpeedMph), PRIVATE);
    }
}

void processSpeedData(const char* data) {
    Serial.print("Received: ");
    Serial.println(data);

    lastSpeedMph = atof(data);
    speedValid = true;

    Serial.print("Speed: ");
    Serial.print(lastSpeedMph);
    Serial.println(" mph");
}

void firmwareUpdateHandler(system_event_t event, int param) {
    switch (param) {
    case firmware_update_begin:
        firmwareUpdateInProgress = true;
        break;

    case firmware_update_complete:
    case firmware_update_failed:
        firmwareUpdateInProgress = false;
        break;
    }
}
