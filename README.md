# Senesp98 - Sen66 sensor with ESP32 Zigbee

## Overview
This project implements a Zigbee End Device (ZED) sensor using the ESP32 platform and ESP-IDF framework.
It's designed to connect to a Zigbee network and transmit sensor data from a SEN66 environmental sensor.

## Features
- Zigbee End Device implementation
- SEN66 environmental sensor integration

## Prerequisites
- ESP-IDF development framework
- SEN66 environmental sensor module
- ESP32 zigbee microcontroller (tested with ESP32-H2)

## Building and Running
1. Open the project in VS Code with Remote Containers
2. Build the project using ESP-IDF commands
3. Flash to your ESP32 device

## Hardware Connections
The SEN66 sensor connects to the ESP32 via I2C:
- SDA -> GPIO10 (green wire)
- SCL -> GPIO22 (yellow wire)
- VCC -> 3.3V
- GND -> GND

Pins can of course be changed in [main/sen66_esp32.c](main/sen66_esp32.c)

## About [SEN66](https://sensirion.com/products/catalog/SEN66)
It includes following sensors:
- PM1, PM2.5, PM4, PM10
- T, RH
- [VOC Index](https://sensirion.com/media/documents/02232963/6294E043/Info_Note_VOC_Index.pdf)
- [NOx Index](https://sensirion.com/media/documents/9F289B95/6294DFFC/Info_Note_NOx_Index.pdf)
- CO2

## About Zigbee
This project exposes usual clusters:
- Temperature
- Humidity
- PM2.5
- CO2

But also custom clusters that are not defined in the Zigbee standard:
- PM1 (0xFC01)
- PM4 (0xFC02)
- PM10 (0xFC03)
- VOC Index (0xFC04)
- NOx Index (0xFC05)

For these custom clusters, we can't use IDs lower than 0x8000 because they are reserved for the standard clusters.
(So we can't use the same IDs as the one defined in the Matter specification)

### Zigbee2MQTT

For now, this device is not yet supported by Zigbee2MQTT. If you want to use it, you can use the [external_converters](https://www.zigbee2mqtt.io/advanced/more/external_converters.html) feature.

You can find the definition in the [sen66.js](zigbee2mqtt/sen66.js) file.

## License

All Sensirion's code defined in [main/sen66](main/sen66) comes from [embedded-i2c-sen66](https://github.com/Sensirion/embedded-i2c-sen66) and is under BSD 3-Clause License