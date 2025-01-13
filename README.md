# Senesp98 - ESP32 Zigbee Sensor Project

## Overview
This project implements a Zigbee End Device (ZED) sensor using the ESP32 platform and ESP-IDF framework. It's designed to connect to a Zigbee network and transmit sensor data from a SEN66 environmental sensor.

## Features
- Zigbee End Device implementation
- SEN66 environmental sensor integration
- Custom partition table configuration

## Prerequisites
- VS Code with Remote Containers extension
- ESP-IDF development framework
- SEN66 environmental sensor module

## Project Structure
- `main/` - Contains the main application source files
- `.devcontainer/` - Docker configuration for development environment
- `sdkconfig.defaults` - Default SDK configuration
- `partitions.csv` - Custom partition table

## Building and Running
1. Open the project in VS Code with Remote Containers
2. The container will automatically set up the ESP-IDF environment
3. Build the project using ESP-IDF commands
4. Flash to your ESP32 device

## Hardware Connections
The SEN66 sensor connects to the ESP32 via I2C:
- SDA -> GPIO21
- SCL -> GPIO22
- VCC -> 3.3V
- GND -> GND

## About [SEN66](https://sensirion.com/products/catalog/SEN66)
It includes following sensors:
- PM1, PM2.5, PM4, PM10
- T, RH
- [VOC Index](https://sensirion.com/media/documents/02232963/6294E043/Info_Note_VOC_Index.pdf)
- [NOx Index](https://sensirion.com/media/documents/9F289B95/6294DFFC/Info_Note_NOx_Index.pdf)
- CO2
