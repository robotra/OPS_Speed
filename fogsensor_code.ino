// This #include statement was automatically added by the Particle IDE.
#include <fog-statistic.h>
#include <Wire.h> // Standard I2C Library
#include <SPI.h>
#include "Particle.h"

Statistic VCNL1proxStats;
Statistic VCNL1ambStats;

// General variables ////////////////////////////////////////////
int bytes;
int reg;
char data;
unsigned long longPreviousMillis = 0;
unsigned long longIntervalMillis = 60000; // Interval for averaging data if millis() is used (in milliseconds)
// VCNL4200 Proximity variables (1 Hz) ////////////////////////////////////////////
char VCNLaddress = 0x51; // VCNL4200 address

int VCNLcount = 0;

uint16_t VCNLprox[8] = {0, 0, 0, 0, 0, 0, 0, 0};

unsigned long VCNLshortPreviousMillis = 0;
unsigned long VCNLshortInterval = 1000;

// VCNL4200 Ambient Light variables (1 Hz)  //////////////////////////////////

uint16_t VCNLamb[8] = {0, 0, 0, 0, 0, 0, 0, 0};

///////////////////////////////////////////////////////////////////

// This example uses threading enabled and SEMI_AUTOMATIC mode
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

// If you are using a product, uncomment these lines and set the correct product ID and version
// for your product
PRODUCT_ID(11852);
PRODUCT_VERSION(1);

const int switchPin = 13;

// This is the maximum amount of time to wait for the cloud to be connected in
// milliseconds. This should be at least 5 minutes. If you set this limit shorter,
// on Gen 2 devices the modem may not get power cycled which may help with reconnection.
const std::chrono::milliseconds connectMaxTime = 6min;

// This is the minimum amount of time to stay connected to the cloud. You can set this
// to zero and the device will sleep as fast as possible, however you may not get
// firmware updates and device diagnostics won't go out all of the time. Setting this
// to 10 seconds is typically a good value to use for getting updates.
const std::chrono::milliseconds cloudMinTime = 10s;

// How long to sleep
const std::chrono::seconds sleepTime = 5min;

// Maximum time to wait for publish to complete. It normally takes 20 seconds for Particle.publish
// to succeed or time out, but if cellular needs to reconnect, it could take longer, typically
// 80 seconds. This timeout should be longer than that and is just a safety net in case something
// goes wrong.
const std::chrono::milliseconds publishMaxTime = 1min;

// Maximum amount of time to wait for a user firmware download in milliseconds
// before giving up and just going back to sleep
const std::chrono::milliseconds firmwareUpdateMaxTime = 2min;

// These are the states in the finite state machine, handled in loop()
enum State
{
    STATE_WAIT_CONNECTED = 0,
    STATE_PUBLISH,
    STATE_PRE_SLEEP,
    STATE_SLEEP,
    STATE_FIRMWARE_UPDATE
};

State state = STATE_WAIT_CONNECTED;

unsigned long stateTime;
bool firmwareUpdateInProgress = false;

// forward declarations
void readSensorAndPublish();
void firmwareUpdateHandler(system_event_t event, int param);
byte i2c_read(char address, int reg, int bytes);
void i2c_write(char address, int reg, char data);

int16_t tfDist = 0; // Distance to object in centimeters
int16_t tfFlux = 0; // Strength or quality of return signal
int16_t tfTemp = 0; // In
String dString = "";
int fluxVal = 0;

//INITIALIZATION
void setup()
{
    System.on(firmware_update, firmwareUpdateHandler);

    // It's only necessary to turn cellular on and connect to the cloud. Stepping up
    // one layer at a time with Cellular.connect() and wait for Cellular.ready() can
    // be done but there's little advantage to doing so.
    Cellular.on();
    Particle.connect();
    stateTime = millis();

    pinMode(switchPin, OUTPUT);

    Wire.begin();       // Starts the I2C Connection
    pinMode(2, OUTPUT); // use LED on pin 5 to indicate errors (blink) or success (steady)

    VCNL1proxStats.clear();
    VCNL1ambStats.clear();

    //********************************************
    // VCNL4200 Setup ////////////////////////////////////////////////////////////////////////////////////////////////
    // Check each VCNL4200 ID and print it to serial monitor

    Wire.beginTransmission(VCNLaddress);
    Wire.write(0x0E); // Register #0E(low) (ID_L) Product ID Revision Register, Should be 58(hex)
    Wire.endTransmission(false);
    Wire.requestFrom(VCNLaddress, 2);

    // /Read VCNL4200 Proximity Data
    byte VCNLdevIDL = Wire.read();
    byte VCNLdevIDH = Wire.read();

    Serial.print(F("VCNL Device ID: "));
    Serial.println(VCNLdevIDL, HEX);

    // configure VCNL4200 proximity sensor
    // configure VCNL4200 register 3 (PS_CONF1 and PS_CONF2)
    Wire.beginTransmission(VCNLaddress);
    Wire.write(0x03);       // low byte = PS_CONF1, high byte = PS_CONF2
    Wire.write(0b11001010); //PS_CONF1  [7:6] PS_DUTY (1/1280 = 1:1)   [5:4] PS_PERS   [3:1] PS_IT pulse width (1:0:1 = 9T)   [0] PS_SD (0 = power on)
    Wire.write(0b00001000); //PS_CONF2    [7:4] reserved     [3] PS_HD (1 = 16 bit)     [2] reserved      [1:0]  PS_INT  (0:0 interupt disable)
    Wire.endTransmission();

    // configure VCNL4200 register 4 (PS_CONF3 and PS_MS)   set PS_AF to active force mode
    Wire.beginTransmission(VCNLaddress);
    Wire.write(0x04);       // low byte = PS_CONF3, high byte = PS_MS
    Wire.write(0b01101001); //PS_CONF3  [7] reserved [6:5] PS_MPS (1:1 8 pulses) [4] PS_SMART_PERS (0 disable) [3] PS_AF (0 active force mode disable) [2] PS-TRIG (0) [1] PS_SC_ADV (1 2Xsunlight immunity) [0] PS_SC_EN (1 sunlight cancellation enable)
    Wire.write(0b00000111); //PS_MS    [7:6] reserved [5] PS_MS (0 prox normal operation with interrupt) [4] PS_SP (1 1.5X sun protect) [3] PS_SPO (1 sun protect mode output FFh) [2:0] LED_I (1:1:1 200 mA)
    Wire.endTransmission();

    // configure VCNL4200 ambient light sensor
    // configure VCNL4200 register 0 (ALS_CONF)
    Wire.beginTransmission(VCNLaddress);
    Wire.write(0x00);       // low byte = ALS_CONF, high byte = reserved
    Wire.write(0b00000000); //ALS_CONF  [7:6] ALS_IT time integration (0:0 50ms) [5] ALS_INT_SWITCH (0) [4] reserved [3:2] ALS_PERS (0) [1] ALS_INT_EN (0 interupt disable) [0] ALS_SC (0 ALS power on)
    Wire.endTransmission();

    Serial.println(F("VCNL1,VCNL2,VCNL3,VCNL4,VCNL5,VCNL6,AMB1,AMB2,AMB3,AMB4,AMB5,AMB6,VCNLcount,OneHzCount")); //data file header
    //logfile.println(F("Date/Time,VCNL1,VCNL2,VCNL3,VCNL4,VCNL5,VCNL6,AMB1,AMB2,AMB3,AMB4,AMB5,AMB6,VCNLcount,VCNLstdev,VCNL2stdev,VCNL3stdev,VCNL4stdev,VCNL5stdev,VCNL6stdev"));  //data file header
    //Serial.println(F("VCNL1prox,VCNL2prox,VCNL1amb,VCNL2amb,VCNLcount,OneHzCount"));  //data file header
    //logfile.println(F("Date/Time,VCNL1prox,VCNL2prox,VCNL1amb,VCNL2amb,VCNLcount,OneHzCount"));  //data file header
    delay(1000);
}

// MAIN
void loop()
{
    switch (state)
    {
    case STATE_WAIT_CONNECTED:
        // Wait for the connection to the Particle cloud to complete
        if (Particle.connected())
        {
            Log.info("connected to the cloud in %lu ms", millis() - stateTime);
            state = STATE_PUBLISH;
            stateTime = millis();
        }
        else if (millis() - stateTime >= connectMaxTime.count())
        {
            // Took too long to connect, go to sleep
            Log.info("failed to connect, going to sleep");
            state = STATE_SLEEP;
        }
        break;

    case STATE_PUBLISH:
        readSensorAndPublish();

        if (millis() - stateTime < cloudMinTime.count())
        {
            Log.info("waiting %lu ms before sleeping", cloudMinTime.count() - (millis() - stateTime));
            state = STATE_PRE_SLEEP;
        }
        else
        {
            state = STATE_SLEEP;
        }
        break;

    case STATE_PRE_SLEEP:
        // This delay is used to make sure firmware updates can start and diagnostics go out
        // It can be eliminated by setting cloudMinTime to 0 and sleep will occur as quickly
        // as possible.
        if (millis() - stateTime >= cloudMinTime.count())
        {
            state = STATE_SLEEP;
        }
        break;

    case STATE_SLEEP:
        if (firmwareUpdateInProgress)
        {
            Log.info("firmware update detected");
            state = STATE_FIRMWARE_UPDATE;
            stateTime = millis();
            break;
        }

        Log.info("going to sleep for %ld seconds", (long)sleepTime.count());

#if HAL_PLATFORM_NRF52840
        // Gen 3 (nRF52840) does not suppport SLEEP_MODE_DEEP with a time in seconds
        // to wake up. This code uses stop mode sleep instead.
        System.sleep(WKP, RISING, sleepTime);
        System.reset();
#else
        System.sleep(SLEEP_MODE_DEEP, sleepTime);
        // This is never reached; when the device wakes from sleep it will start over
        // with setup()
#endif
        break;

    case STATE_FIRMWARE_UPDATE:
        if (!firmwareUpdateInProgress)
        {
            Log.info("firmware update completed");
            state = STATE_SLEEP;
        }
        else if (millis() - stateTime >= firmwareUpdateMaxTime.count())
        {
            Log.info("firmware update timed out");
            state = STATE_SLEEP;
        }
        break;
    }
}

void readSensorAndPublish()
{

    // OneHzCount=0;
    VCNL1ambStats.clear();
    VCNL1proxStats.clear();
    for (int i = 0; i<10; i++){
    Wire.beginTransmission(VCNLaddress);
    Wire.write(0x04);       // low byte = PS_CONF3, high byte = PS_MS
    Wire.write(0b01101101); //PS_CONF3  [7] reserved [6:5] PS_MPS (1:1 8 pulses) [4] PS_SMART_PERS (0 disable) [3] PS_AF (1 active force mode enable) [2] PS-TRIG (1) [1] PS_SC_ADV (1 2Xsunlight immunity) [0] PS_SC_EN (1 sunlight cancellation enable)
    Wire.write(0b00000111); //PS_MS    [7:6] reserved [5] PS_MS (0 prox normal operation with interrupt) [4] PS_SP (1 1.5X sun protect) [3] PS_SPO (1 sun protect mode output FFh) [2:0] LED_I (1:1:1 200 mA)
    Wire.endTransmission();

    delay(30);

    Wire.beginTransmission(VCNLaddress);
    Wire.write(0x08);
    Wire.endTransmission(false);
    Wire.requestFrom(VCNLaddress, 2);

    // /Read VCNL4200 Proximity Data
    uint8_t LSB = Wire.read();
    uint16_t MSB = Wire.read();

    VCNLprox[2] = (MSB <<= 8) + LSB;
    delay(30);

    VCNL1proxStats.add(VCNLprox[2]);

    // Read VCNL4200 ambient data /////////////////////////////////////////////////////////////////////
    Wire.beginTransmission(VCNLaddress);
    Wire.write(0x09);
    Wire.endTransmission(false);
    Wire.requestFrom(VCNLaddress, 2);
    delay(30);

    // /Read VCNL4200 ALS Data
    LSB = Wire.read();
    MSB = Wire.read();

    VCNLamb[2] = (MSB <<= 8) + LSB;

    delay(30);

    VCNL1ambStats.add(VCNLamb[2]);
    }
    Particle.publish("VCNL1ambStats", String(VCNL1ambStats.average()));
    Particle.publish("VCNL1proxStats", String(VCNL1proxStats.average()));
    Particle.publish("BatterySoC",String(System.batteryCharge()));
    Particle.publish("BatteryState",String(System.batteryState()));
}

byte i2c_read(char address, int reg, int bytes)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(address, bytes);
    return data = Wire.read();
}

void i2c_write(char address, int reg, char data)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void firmwareUpdateHandler(system_event_t event, int param)
{
    switch (param)
    {
    case firmware_update_begin:
        firmwareUpdateInProgress = true;
        break;

    case firmware_update_complete:
    case firmware_update_failed:
        firmwareUpdateInProgress = false;
        break;
    }
}