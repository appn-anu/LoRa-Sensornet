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
   .dio = {7, 6, LMIC_UNUSED_PIN}, 
};


/* **************************************************************
 * low power
 * *************************************************************/

#include "LowPower.h"

// show debug statements; comment next line to disable debug statements
#define DEBUG

/* ************************************************************** 
 *  sensor settings
 * *************************************************************/
// I2C soil moisture settings (HAS)
// https://www.tindie.com/products/miceuz/i2c-soil-moisture-sensor/

#include <I2CSoilMoistureSensor.h>
#include <Wire.h>

#define VBATPIN A9

I2CSoilMoistureSensor sensor;

unsigned int batt_mV; // battery millivolts

unsigned int light;     // light on a scale from 0 (dark) to 15 (light)
float soil_tempC;
unsigned int soil_moisture; 
float air_tempC;
float air_relativehumidity;
unsigned int air_pressurehPa;

// data to send
static uint8_t dataTX[52]; // max packet size
enum payload_types 
{
  PAYLOAD_NONE,
  PAYLOAD_MV_ONLY,
  PAYLOAD_SOIL,
  PAYLOAD_AIR,
  PAYLOAD_SOIL_AND_AIR
};


/* **************************************************************
 * user settings
 * *************************************************************/
unsigned long cycle_length = 10 * 60 * 1000UL; // cycle * mins_or_secs * 1000;
unsigned long sense_every = 1;
unsigned long send_every = 1;

unsigned long cycle = -1;  //  init at -1, so first cycle starts as cycle 0 for 1st sense/send
unsigned long prevSleep = 0; 

size_t datasize;
payload_types payload_type = PAYLOAD_NONE;

/* **************************************************************
 * sensor code, typical would be init_sensor(), do_sense(), build_data()
 * *************************************************************/
/* **************************************************************
 * init the sensor
 * *************************************************************/
void init_sensor() {
  Wire.begin();
  sensor.begin(); // reset sensor
  delay(1000); // give some time to boot up

  #ifdef DEBUG
    Serial.print("I2C Soil Moisture Sensor Address: ");
    Serial.println(sensor.getAddress(),HEX);
    Serial.print("Sensor Firmware version: ");
    Serial.println(sensor.getVersion(),HEX);
    Serial.println();
  #endif  
}

/* **************************************************************
 * do the reading
 * *************************************************************/
void do_sense() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *=2;
  measuredvbat *= 3.3;
  measuredvbat /= 1024;
  measuredvbat *= 1000;
  batt_mV = (unsigned int)measuredvbat;
  payload_type = PAYLOAD_MV_ONLY;
  
  if (sensor.getVersion() != 0xFF) {
    while (sensor.isBusy()) delay(50); // available since FW 2.3
    soil_moisture = sensor.getCapacitance();

    soil_tempC = ((float)sensor.getTemperature())/10.0;

    unsigned int lightR = sensor.getLight(true); //request light measurement, wait and read light register
    // convert the light value (65535 = dark, 0 = light) to a scale of 0 to 15
    light = lightR / 4096.0;   // 15 = dark, 0 = light
    light = 15 - light;        // 0 = dark, 15 = light

    // put sensor to sleep  
    sensor.sleep();

    #ifdef DEBUG
      Serial.print(F("light:"));
      Serial.print(light);
      Serial.print(F(" temp:"));
      Serial.print(soil_tempC);
      Serial.print(F(" soil_moisture:"));
      Serial.print(soil_moisture);
      Serial.println(F(""));
      Serial.print(F(" battery:"));
      Serial.print(batt_mV);
      Serial.println(F(""));
    #endif
    payload_type = PAYLOAD_SOIL;
  }
  
  if (false){// readings for bme280
    air_tempC = 0.01;
    air_pressurehPa = 1016.82; //some pressure here
    // set payload type
    if (payload_type == PAYLOAD_SOIL) {
      payload_type = PAYLOAD_SOIL_AND_AIR;
    }else{
      payload_type = PAYLOAD_AIR;
    }
  }

  
}

void build_data() {
  
  dataTX[0] = payload_type; 
  size_t n = 1;
  
  dataTX[n++] = lowByte(batt_mV);
  dataTX[n++] = highByte(batt_mV);
  if (payload_type == PAYLOAD_SOIL || payload_type == PAYLOAD_SOIL_AND_AIR){
    dataTX[n++] = lowByte(light);
    dataTX[n++] = highByte(light);
    uint16_t payload_soilTemp = LMIC_f2sflt16(soil_tempC/100.0); // adjust for the f2sflt16 range (-1 to 1)
    dataTX[n++] = lowByte(payload_soilTemp);
    dataTX[n++] = highByte(payload_soilTemp);
    dataTX[n++] = lowByte(soil_moisture);
    dataTX[n++] = highByte(soil_moisture);
  }
  if (payload_type == PAYLOAD_AIR || payload_type == PAYLOAD_SOIL_AND_AIR){
    uint16_t payload_airTemp = LMIC_f2sflt16(air_tempC/100.0); // adjust for the f2sflt16 range (-1 to 1)
    dataTX[n++] = lowByte(payload_airTemp);
    dataTX[n++] = highByte(payload_airTemp);
    uint16_t payload_airRh = LMIC_f2sflt16(air_relativehumidity/1000.0); // adjust for the f2sflt16 range (-1 to 1)
    dataTX[n++] = lowByte(payload_airRh);
    dataTX[n++] = highByte(payload_airRh);
    uint16_t payload_airPres = LMIC_f2sflt16(air_pressurehPa/10000.0); // adjust for the f2sflt16 range (-1 to 1)
    dataTX[n++] = lowByte(payload_airPres);
    dataTX[n++] = highByte(payload_airPres);
  }
  datasize = n;
}


/* **************************************************************
* sleep
* *************************************************************/
void do_sleep(float sleepTime) {

  #ifdef DEBUG
    Serial.print(F("Sleep for "));
    Serial.print(sleepTime/1000, 3);
    Serial.println(F(" seconds"));
  #endif

  Serial.flush();
  // sleep logic using LowPower library
  int delays[] = {8000, 4000, 2000, 1000, 500, 250, 120, 60, 30, 15};
  period_t sleep[] = {SLEEP_8S, SLEEP_4S, SLEEP_2S, SLEEP_1S, SLEEP_500MS,  SLEEP_250MS, SLEEP_120MS, SLEEP_60MS, SLEEP_30MS, SLEEP_15MS};

  // correction for overhead in this routine
  sleepTime = sleepTime * 0.93;

  float x;
  unsigned int i;
  for (i=0; i<=9; i++) {
    for (x=sleepTime; x>=delays[i]; x-=delays[i]) {
      LowPower.powerDown(sleep[i], ADC_OFF, BOD_ON);
      sleepTime -= delays[i];
    } 
  }

}

// test payload :F000F400EF1100 
// light:15 temp:244 moisture:239 battery:4460
  
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

  Serial.print(millis());
  Serial.print(F(" Sending.. "));  

  send_message(&sendjob);

  // wait for send to complete
  Serial.print(millis());
  Serial.print(F(" Waiting.. "));  
 
  while ( (LMIC.opmode & OP_JOINING) or (LMIC.opmode & OP_TXRXPEND) ) { os_runloop_once();  }
  Serial.print(millis());
  Serial.println(F(" TX_COMPLETE"));
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
  // next cycle
  cycle += 1;
  unsigned long current = millis(); 
  
  if ( (cycle % sense_every) == 0 ) { do_sense(); }

  // check if need to send
  if ( (cycle % send_every) == 0 ) { build_data(); do_send(); }
  
  // go to sleep
  current = millis();
  do_sleep(cycle_length - (current - prevSleep));  // sleep minus elapsed time
  prevSleep = current;
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

  init_node();
  init_sensor();
}
