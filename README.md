# LoRa-Sensornet
sensornet backed by lora and ttn


example euis.h:


```
// The 2 below should be in little endian format (lsb)
static const uint8_t PROGMEM DEVEUI[8]= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t PROGMEM APPEUI[8]= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// This should be in big endian format (msb)
static const uint8_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
```

example .docker-env for the docker telegraf config

```
OUTPUTS_INFLUXDB_USERNAME=admin
OUTPUTS_INFLUXDB_PASSWORD=<admin password
OUTPUTS_INFLUXDB_DATABASE=database
INPUTS_MQTT_CONSUMER_USERNAME=<ttn application id>
INPUTS_MQTT_CONSUMER_PASSWORD=<ttn application pass>
```
