// Particle Boron - Omnipresense Speed Sensor UART Receiver
// Receives speed data via Serial1 (hardware UART)

// Define Serial1 pins for Boron
// RX: A3, TX: A2

#define SENSOR_BAUD 9600
#define BUFFER_SIZE 64

char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
    Serial.begin(9600);  // Debug serial
    Serial1.begin(SENSOR_BAUD);  // Sensor UART
    
    waitFor(Serial.isConnected, 10000);
    Serial.println("Boron Speed Sensor Receiver Started");
}

void loop() {
    // Check for incoming UART data
    while (Serial1.available()) {
        char inChar = Serial1.read();
        
        // Accumulate data until newline
        if (inChar == '\n') {
            rxBuffer[bufferIndex] = '\0';  // Null terminate
            processSpeedData(rxBuffer);
            bufferIndex = 0;  // Reset buffer
        } 
        else if (inChar != '\r') {
            // Add to buffer if not carriage return
            if (bufferIndex < BUFFER_SIZE - 1) {
                rxBuffer[bufferIndex++] = inChar;
            }
        }
    }
    
    delay(10);
}

void processSpeedData(char* data) {
    // Parse and process speed sensor data
    Serial.print("Received: ");
    Serial.println(data);
    
    // Extract speed value (adjust parsing based on sensor format)
    float speed = atof(data);
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.println(" mph");
    
    // Publish to Particle Cloud (optional)
    Particle.publish("speed_reading", String(speed), PRIVATE);
}