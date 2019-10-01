/* ************************************************************** 
 * Arduino sketch with low power
 * Author: Martijn Quaedvlieg / Jan de Laet (january 2017)
 * Generated with Generate script by Jan de Laet
 * 
 * *************************************************************/
#include <SPI.h>
// define the activation method ABP or OTAA
#define ACT_METHOD_OTAA

#include "euis.h"

/* **************************************************************
 * radio
 * *************************************************************/
// Uses LMIC libary by Thomas Telkamp and Matthijs Kooijman (https://github.com/matthijskooijman/arduino-lmic)
#include <lmic.h>
#include <hal/hal.h>

// Declare the job control structures
static osjob_t sendjob;

#define SLEEP_MINUTES 10

// These callbacks are only used in over-the-air activation, so they are
// left empty when ABP (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

/* ************************************************************** 
 * Pin mapping
 * *************************************************************/

const lmic_pinmap lmic_pins = { 
   .nss = 8, 
   .rxtx = LMIC_UNUSED_PIN, 
   .rst = 4, 
   .dio = {3, 6, 5}, 
};



#include <RTCZero.h>
RTCZero rtc;

// show debug statements; comment next line to disable debug statements
#define DEBUG

#define VBATPIN A7

#define ANALOGPIN A1 // read from analog pin 1 (not 0 as it is the DAC for M0)

// data to send
static uint8_t dataTX[52]; // max packet size
enum payload_types 
{
  PAYLOAD_NONE,
  PAYLOAD_BATT_ONLY,
  PAYLOAD_BATT_SOIL,
  PAYLOAD_BATT_AIR,
  PAYLOAD_BATT_SOIL_AIR,
  PAYLOAD_BATT_ANALOG,
  PAYLOAD_BATT_ANALOG_SOIL,
  PAYLOAD_BATT_ANALOG_AIR,
  PAYLOAD_BATT_ANALOG_SOIL_AIR
};


/* **************************************************************
 * user settings
 * *************************************************************/
#define cycle_length (1 * 60 * 1000UL) // cycle * mins_or_secs * 1000;
#define sense_every 1
#define send_every 1

unsigned long cycle = -1;  //  init at -1, so first cycle starts as cycle 0 for 1st sense/send
unsigned long prevSleep = 0; 
size_t datasize;
payload_types payload_type = PAYLOAD_NONE;

/* **************************************************************
 * sensor settings
 * *************************************************************/

#define MAX_TRIES 5
#define BME_MAX_TEMP 85
#define BME_MIN_TEMP -40
#define BME_MAX_HUM 100
#define BME_MIN_HUM 0
#define BME_MIN_PRES 300
#define BME_MAX_PRES 1100

#include <Wire.h>

/* **************************************************************
 * I2C soil moisture sensor
 * https://www.tindie.com/products/miceuz/i2c-soil-moisture-sensor/
 * https://github.com/Apollon77/I2CSoilMoistureSensor
 * *************************************************************/
#include <I2CSoilMoistureSensor.h>
I2CSoilMoistureSensor sensor;

/* **************************************************************
 * BME280 I2C library, Tyler Glenn <finitespaceghb2@junk.yoglenn.com>
 * https://www.github.com/finitespace/BME280
 * *************************************************************/
#include <BME280I2C.h>
BME280I2C bme;


bool isValidBME(float temperature, float humidity, float pressure){
  if (isnan(temperature) || isnan(humidity) || isnan(pressure)) return false;
  if (temperature < BME_MIN_TEMP || temperature > BME_MAX_TEMP) return false;
  if (humidity < BME_MIN_HUM || humidity > BME_MAX_HUM) return false;
  if (pressure < BME_MIN_PRES || pressure > BME_MAX_PRES) return false;
  return true;
}

void init_sensor() {
  Wire.begin();
  sensor.begin(); // reset sensor
  delay(1000); // give some time to boot up
}

void do_sense() {
  memset(dataTX, 0, sizeof(dataTX)); // set dataTX to 0
  size_t n = 1; // leave 0 for payload_type
  
  dataTX[0] = PAYLOAD_BATT_ANALOG; // set payload type to battery and analog to begin with
  analogReadResolution(12); // set the adc resolution even if the hardware doesnt support it

  float analogBatt = analogRead(VBATPIN);
  analogBatt *= 2; // voltage divider for battery pin
  analogBatt *= 3300; // aref mV
  analogBatt /= 4096; // max resolution of adc @ 12bits
  uint16_t batt_mV = (uint16_t)analogBatt;
  dataTX[n++] = lowByte(batt_mV);
  dataTX[n++] = highByte(batt_mV);

  float analogMeasurement = analogRead(ANALOGPIN);
  analogMeasurement *= 3300; // aref mV
  analogMeasurement /= 4096; // max resolution of adc @ 12bits
  uint16_t analog_mV = (uint16_t)(analogMeasurement);
  dataTX[n++] = lowByte(analog_mV);
  dataTX[n++] = highByte(analog_mV);
  
  size_t tries = 0;

  if (bme.begin()){ // bme returns true when is successful and sensor exists.
    float air_tempC, air_relativeHumidity;
    uint16_t air_pressurehPa;
    do {
      // bme.refresh();
      float pres_read;
      bme.read(pres_read, air_tempC, air_relativeHumidity);
      air_pressurehPa = (uint16_t)pres_read;
      // air_tempC = bme.temperature;
      // air_relativeHumidity = bme.humidity;
      // air_pressurehPa = (int)(bme.pressure/100.0F);
      delay(100);
    } while (tries++ < MAX_TRIES && !isValidBME(air_tempC, air_relativeHumidity, air_pressurehPa));
    
    if (tries < MAX_TRIES) {
      uint16_t payload_airTemp = LMIC_f2sflt16(air_tempC/100.0); // adjust for the f2sflt16 range (-1 to 1)
      dataTX[n++] = lowByte(payload_airTemp);
      dataTX[n++] = highByte(payload_airTemp);
      uint16_t payload_airRh = LMIC_f2sflt16(air_relativeHumidity/1000.0); // adjust for the f2sflt16 range (-1 to 1)
      dataTX[n++] = lowByte(payload_airRh);
      dataTX[n++] = highByte(payload_airRh);
      uint16_t payload_airPres = LMIC_f2sflt16(air_pressurehPa/10000.0); // adjust for the f2sflt16 range (-1 to 1)
      dataTX[n++] = lowByte(payload_airPres);
      dataTX[n++] = highByte(payload_airPres);
      Serial.print(F("bme280 read success: "));
      Serial.println(air_tempC);
      dataTX[0] = PAYLOAD_BATT_ANALOG_AIR;
    }
  }

  // we detect the sensor version because it is already begun
  if (sensor.getVersion() != 0xFF) {
    while (sensor.isBusy()) delay(50); // available since FW 2.3
    
    unsigned int soil_moisture = sensor.getCapacitance();

    float soil_tempC = ((float)sensor.getTemperature())/10.0;

    unsigned int lightR = sensor.getLight(true); //request light measurement, wait and read light register
    // convert the light value (65535 = dark, 0 = light) to a scale of 0 to 15
    uint8_t light = lightR / 4096.0;   // 15 = dark, 0 = light
    light = 15 - light;        // 0 = dark, 15 = light
    // put sensor to sleep
    sensor.sleep();
    dataTX[n++] = light;
    uint16_t payload_soilTemp = LMIC_f2sflt16(soil_tempC/100.0); // adjust for the f2sflt16 range (-1 to 1)
    dataTX[n++] = lowByte(payload_soilTemp);
    dataTX[n++] = highByte(payload_soilTemp);
    dataTX[n++] = lowByte(soil_moisture);
    dataTX[n++] = highByte(soil_moisture);
    Serial.print(F("chirp read success: "));
    Serial.println(soil_tempC);
    if(dataTX[0] == PAYLOAD_BATT_ANALOG_AIR) dataTX[0] = PAYLOAD_BATT_ANALOG_SOIL_AIR; // set payload  type
    else dataTX[0] = PAYLOAD_BATT_ANALOG_SOIL;
  } 
  datasize = n; // set the datasize so we know how much data to transmit.
}

void alarmMatch(){
  digitalWrite(8, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
};

/* **************************************************************
* sleep
* *************************************************************/
void do_sleep() {
  
  Serial.print("Sleeping for ");
  Serial.print(SLEEP_MINUTES);
  Serial.println(" minutes..." );
  Serial.flush();

#ifdef USBCON
  USBDevice.detach();
#endif

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(8, HIGH);
  
  rtc.setAlarmMinutes(SLEEP_MINUTES);
  rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.attachInterrupt(alarmMatch);
  rtc.standbyMode();

#ifdef USBCON
  USBDevice.attach();
#endif
  digitalWrite(8, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
}

/* **************************************************************
 * radio code, typical would be init_node(), do_send(), etc
 * *************************************************************/
/* **************************************************************
 * init the Node
 * *************************************************************/
void init_node() {

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  // LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);

  // got this fix from forum: https://www.thethingsnetwork.org/forum/t/over-the-air-activation-otaa-with-lmic/1921/36
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  LMIC_selectSubBand(1);
}

/* *****************************************************************************
* send_message
* ****************************************************************************/
void send_message(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, dataTX, datasize, 0);
    Serial.println(F("Packet queued"));
  }
}


/* **************************************************************
 * send the message
 * *************************************************************/
void do_send() {
  Serial.println(F("Sending..."));  
  send_message(&sendjob);
  // wait for send to complete
  Serial.println(F("Waiting..."));  
  while ( (LMIC.opmode & OP_JOINING) or (LMIC.opmode & OP_TXRXPEND) ) { os_runloop_once();  }
  Serial.println(F("TX_COMPLETE"));
}
  
/*******************************************************************************/
void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print(F("Data Received: "));
        Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      // schedule next transmission
      // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), send_message);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      break;
  }
    
}



/* **************************************************************
 * loop
 * *************************************************************/
void loop() {
  rtc.setTime(0, 0, 0);
  do_sense();

  // // check if need to send
  do_send();
  do_sleep();  // sleep minus elapsed time
}

/* **************************************************************
 * setup
 * *************************************************************/
void setup() {
  // Wait (max 10 seconds) for the Serial Monitor
  while ((!Serial) && (millis() < 10000)){ }

  //Set baud rate
  Serial.begin(9600);
  Serial.println(F("Lora soil moisture sensor node (template version: 13Jan2017 generated: 19Aug2019)"));
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  

  rtc.begin(true);
  init_node();
  init_sensor();
}
