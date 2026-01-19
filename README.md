# ESP32 Multi-RTD MQTT Gateway

Generic IoT gateway firmware for ESP32 devices that reads temperature
data from multiple RTD PT100 sensors using MAX31865 modules and publishes
telemetry over MQTT.

## Features
- Multi-sensor RTD PT100 support
- MQTT gateway compatible with ThingsBoard
- OTA firmware updates
- Persistent credentials using ESP32 Preferences
- Scalable sensor architecture

## Hardware
- ESP32
- MAX31865 RTD amplifiers
- PT100 RTD sensors

## Telemetry Format
```json
{
  "sensors": [
    { "sensor_id": 1, "temperature_celsius": 4.3 },
    { "sensor_id": 2, "temperature_celsius": 4.7 }
  ]
}
