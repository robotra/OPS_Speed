# Omnipresence UART Radar -> Particle Boron -> Node-RED Dashboard

This guide shows a practical, end-to-end path for taking UART radar data from an omnipresence sensor, forwarding it through a Particle Boron, and visualizing it in a Node-RED dashboard.

## 1) Architecture

1. **Radar sensor** streams frames over UART (`TX -> Boron RX`, optional `RX -> Boron TX` for configuration).
2. **Particle Boron firmware** parses each UART frame into values (for example: motion flag, distance, confidence).
3. Boron **publishes JSON payloads** using `Particle.publish()` (or optionally sends directly over HTTP if your plan allows it).
4. **Node-RED** receives events from Particle Cloud (SSE or webhook), extracts fields, then writes values to dashboard widgets.

## 2) Hardware wiring

Typical 3.3V UART wiring:

- Sensor `VCC` -> Boron `3V3` (or external regulator per sensor current requirements)
- Sensor `GND` -> Boron `GND`
- Sensor `TX` -> Boron UART `RX` (for Boron: `Serial1 RX`)
- Sensor `RX` -> Boron UART `TX` (optional, only if sensor needs host commands)

> Verify your sensor voltage levels. If the sensor UART is 5V logic, use a level shifter before the Boron.

## 3) Particle Boron firmware example

This example assumes incoming sensor lines are ASCII ending in `\n` such as:

```text
MOTION=1,DIST=142,CONF=87
```

Adapt the parser if your sensor emits binary frames.

```cpp
#include "Particle.h"

SYSTEM_THREAD(ENABLED);

// Publish no faster than your Particle plan allows.
const unsigned long PUBLISH_MS = 2000;
unsigned long lastPublish = 0;

String uartLine;

struct RadarData {
    int motion = 0;
    int distance = -1;
    int confidence = -1;
    bool valid = false;
};

RadarData parseRadarLine(const String& line) {
    RadarData d;

    int m = -1, dist = -1, conf = -1;
    int parsed = sscanf(line.c_str(), "MOTION=%d,DIST=%d,CONF=%d", &m, &dist, &conf);

    if (parsed == 3) {
        d.motion = m;
        d.distance = dist;
        d.confidence = conf;
        d.valid = true;
    }

    return d;
}

void setup() {
    Serial.begin(115200);   // USB debug
    Serial1.begin(115200);  // Sensor UART baud; set to your sensor's baud
}

void loop() {
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == '\n') {
            uartLine.trim();

            RadarData d = parseRadarLine(uartLine);
            if (d.valid && millis() - lastPublish >= PUBLISH_MS) {
                char payload[96];
                snprintf(payload, sizeof(payload),
                         "{\"motion\":%d,\"distance\":%d,\"confidence\":%d}",
                         d.motion, d.distance, d.confidence);

                // Event name used by Node-RED subscription.
                Particle.publish("radar_data", payload, PRIVATE);
                lastPublish = millis();

                Serial.println(payload);
            }

            uartLine = "";
        } else {
            uartLine += c;
            if (uartLine.length() > 200) {
                uartLine = "";  // guard against malformed frames
            }
        }
    }
}
```

## 4) Particle Cloud -> Node-RED integration options

### Option A (recommended): Particle webhook to Node-RED HTTP endpoint

1. In Node-RED, create an `http in` node:
   - Method: `POST`
   - URL: `/particle/radar`
2. Add a `json` node to parse body.
3. Add function/change nodes to map payload fields.
4. Add dashboard widgets (`ui_text`, `ui_gauge`, `ui_chart`) for `motion`, `distance`, `confidence`.
5. In Particle Console, create integration webhook:
   - Trigger event: `radar_data`
   - URL: `https://<your-nodered-host>/particle/radar`
   - Request type: `JSON`

### Option B: Node-RED subscribes to Particle event stream (SSE)

1. In Node-RED, use an `http request` or dedicated Particle node to subscribe to:
   - `https://api.particle.io/v1/devices/events/radar_data?access_token=<TOKEN>`
2. Parse event `data` as JSON.
3. Route to dashboard widgets.

Option A is usually easier to secure and scale because Node-RED just handles regular HTTPS webhooks.

## 5) Node-RED flow skeleton (webhook path)

- `http in (POST /particle/radar)`
- `json`
- `function` (normalize payload)
- `ui_text` (motion state)
- `ui_gauge` (distance)
- `ui_chart` (distance over time)
- `http response (200 OK)`

Example function node:

```js
// Particle webhook body usually includes event metadata plus data string.
// If payload arrives as msg.payload.data JSON string, parse it.
let p = msg.payload;

if (typeof p.data === "string") {
    try {
        p = JSON.parse(p.data);
    } catch (e) {
        // leave as-is if already parsed differently
    }
}

msg.motion = p.motion;
msg.distance = p.distance;
msg.confidence = p.confidence;
msg.payload = p.distance; // for gauge/chart default wiring

return msg;
```

## 6) Reliability + data-shaping tips

- **Rate-limit publishes** on Boron; avoid sending every UART byte burst.
- Include a **timestamp** in payload if your dashboard needs device-side timing.
- Add a **device ID** field if you will deploy multiple sensors.
- For noisy sensors, do a moving average (for distance) before publishing.
- Add stale-data handling in Node-RED (if no update in X seconds, set status = offline).

## 7) Security checklist

- Use **Particle PRIVATE events** only.
- Keep access tokens in Node-RED environment variables, not hardcoded in flows.
- If using webhook endpoint exposure, protect with HTTPS and (if possible) shared secret validation in Node-RED.

## 8) Bring-up checklist

1. Confirm raw sensor UART data using USB serial logs on Boron.
2. Confirm parsed values print correctly in Boron logs.
3. Confirm `radar_data` appears in Particle Console events.
4. Confirm Node-RED receives payload and updates widgets.
5. Confirm dashboard behavior when sensor is disconnected.

---

If you share your exact radar sensor model and one sample raw UART frame, you can swap in a parser that exactly matches your protocol (especially if your module outputs binary packets).
