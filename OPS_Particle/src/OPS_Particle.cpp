// Particle Boron - Omnipresense Speed Sensor UART Receiver
// Receives speed data via Serial1 (hardware UART)

#include "Particle.h"
#include <stdio.h>

#define SENSOR_BAUD_PRIMARY 19200
#define SENSOR_BAUD_FALLBACK 9600
#define BUFFER_SIZE 64
#define MAX_EVENTS 20

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

// Connection/sleep behavior copied from fogsensor state-machine approach
const std::chrono::milliseconds connectMaxTime = 6min;
const std::chrono::milliseconds collectDuration = 1min;
const std::chrono::seconds sleepTime = 5min;
const std::chrono::milliseconds firmwareUpdateMaxTime = 2min;

enum State {
    STATE_WAIT_CONNECTED = 0,
    STATE_COLLECT,
    STATE_PUBLISH,
    STATE_SLEEP,
    STATE_FIRMWARE_UPDATE
};

struct SpeedEvent {
    unsigned long tickMs;
    time_t unixTime;
    float speedMph;
};

State state = STATE_WAIT_CONNECTED;
unsigned long stateTime = 0;
bool firmwareUpdateInProgress = false;

char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;
SpeedEvent events[MAX_EVENTS];
size_t eventCount = 0;
size_t droppedEventCount = 0;
unsigned long collectStartMs = 0;
int sensorBaud = SENSOR_BAUD_PRIMARY;

void initSensorUart();
bool trySensorBaud(int baud);
void clearSerial1Buffer();
void collectSensorData();
bool parseSpeedData(const char* data, float &speedOut);
String buildBatchJson();
void resetCollectionWindow();
void firmwareUpdateHandler(system_event_t event, int param);

void setup() {
    Serial.begin(9600);

    waitFor(Serial.isConnected, 10000);
    Serial.println("Boron Speed Sensor Receiver Started");

    initSensorUart();

    System.on(firmware_update, firmwareUpdateHandler);

    Cellular.on();
    Particle.connect();
    stateTime = millis();
}

void initSensorUart() {
    if (trySensorBaud(SENSOR_BAUD_PRIMARY)) {
        sensorBaud = SENSOR_BAUD_PRIMARY;
        return;
    }

    if (trySensorBaud(SENSOR_BAUD_FALLBACK)) {
        sensorBaud = SENSOR_BAUD_FALLBACK;
        return;
    }

    // Fallback if no response was detected.
    sensorBaud = SENSOR_BAUD_PRIMARY;
    Serial1.begin(sensorBaud);
    Serial.println("Sensor baud auto-detect failed; defaulting to 19200");
}

bool trySensorBaud(int baud) {
    Serial1.end();
    delay(50);
    Serial1.begin(baud);
    delay(150);
    clearSerial1Buffer();

    // Query current interface settings (AN-014-C: I? command).
    Serial1.print("I?\r\n");

    unsigned long start = millis();
    char line[BUFFER_SIZE];
    int idx = 0;

    while (millis() - start < 900) {
        while (Serial1.available()) {
            char c = (char)Serial1.read();
            if (c == '\n') {
                line[idx] = '\0';
                if (idx > 0) {
                    Serial.print("Sensor init response @");
                    Serial.print(baud);
                    Serial.print(": ");
                    Serial.println(line);
                    return true;
                }
                idx = 0;
            }
            else if (c != '\r') {
                if (idx < BUFFER_SIZE - 1) {
                    line[idx++] = c;
                }
            }
        }
        delay(10);
    }

    // Some configs stream data without an I? response; treat any bytes as valid.
    if (Serial1.available()) {
        Serial.print("Sensor data detected @");
        Serial.println(baud);
        return true;
    }

    return false;
}

void clearSerial1Buffer() {
    while (Serial1.available()) {
        (void)Serial1.read();
    }
}

void loop() {
    switch (state) {
    case STATE_WAIT_CONNECTED:
        if (Particle.connected()) {
            Log.info("connected to the cloud in %lu ms", millis() - stateTime);
            resetCollectionWindow();
            state = STATE_COLLECT;
            stateTime = millis();
        }
        else if (millis() - stateTime >= connectMaxTime.count()) {
            Log.info("failed to connect, going to sleep");
            state = STATE_SLEEP;
        }
        break;

    case STATE_COLLECT:
        collectSensorData();
        if (millis() - collectStartMs >= collectDuration.count()) {
            state = STATE_PUBLISH;
        }
        break;

    case STATE_PUBLISH:
        if (Particle.connected()) {
            String payload = buildBatchJson();
            Particle.publish("speed_batch_json", payload, PRIVATE);
            Log.info("published %u events (%u dropped)", (unsigned)eventCount, (unsigned)droppedEventCount);
            state = STATE_SLEEP;
        }
        else {
            Log.info("cloud disconnected before publish, skipping upload");
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

void collectSensorData() {
    while (Serial1.available()) {
        char inChar = Serial1.read();

        if (inChar == '\n') {
            rxBuffer[bufferIndex] = '\0';
            float speed = 0.0f;
            if (parseSpeedData(rxBuffer, speed)) {
                if (eventCount < MAX_EVENTS) {
                    SpeedEvent &evt = events[eventCount++];
                    evt.tickMs = millis();
                    evt.unixTime = Time.isValid() ? Time.now() : 0;
                    evt.speedMph = speed;
                }
                else {
                    droppedEventCount++;
                }
            }
            bufferIndex = 0;
        }
        else if (inChar != '\r') {
            if (bufferIndex < BUFFER_SIZE - 1) {
                rxBuffer[bufferIndex++] = inChar;
            }
        }
    }
}

bool parseSpeedData(const char* data, float &speedOut) {
    Serial.print("Received: ");
    Serial.println(data);

    char *endPtr = nullptr;
    float parsed = strtof(data, &endPtr);
    if (endPtr == data) {
        // Non-numeric line (for example: config/command response), ignore.
        return false;
    }

    speedOut = parsed;
    return true;
}

String buildBatchJson() {
    String json = "{";
    json += "\"window_start_tick_ms\":";
    json += String(collectStartMs);
    json += ",\"window_duration_ms\":";
    json += String((unsigned long)collectDuration.count());
    json += ",\"event_count\":";
    json += String((unsigned)eventCount);
    json += ",\"dropped_event_count\":";
    json += String((unsigned)droppedEventCount);
    json += ",\"events\":[";

    for (size_t i = 0; i < eventCount; i++) {
        if (i > 0) {
            json += ",";
        }
        json += "{";
        json += "\"tick_ms\":";
        json += String(events[i].tickMs);
        json += ",\"ts_unix\":";
        json += String((long)events[i].unixTime);
        json += ",\"speed_mph\":";
        json += String(events[i].speedMph, 2);
        json += "}";
    }
    json += "]}";
    return json;
}

void resetCollectionWindow() {
    eventCount = 0;
    droppedEventCount = 0;
    collectStartMs = millis();
    bufferIndex = 0;
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
