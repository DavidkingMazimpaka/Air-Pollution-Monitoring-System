# Air Quality Monitoring System

## Overview
This project is an **Air Quality Monitoring System** built on the **ESP8266** platform. It uses multiple sensors to monitor air quality and environmental parameters, such as temperature, humidity, CO2 levels, and various other gases. The system sends real-time data to both **Firebase** and **Adafruit IO** for cloud storage and monitoring. The data is also displayed on a 16x2 **LCD screen** and triggers a **buzzer** for CO2 concentration alerts when levels exceed a set threshold.

## Features
- **Air Quality Sensors**: Monitors CO2, CO, NH4, Alcohol, Toluene, and Acetone gas levels.
- **Temperature and Humidity**: Uses a **DHT22** sensor for reading temperature and humidity.
- **GPS Module**: Captures the geographical location of the sensor using the **TinyGPSPlus** library.
- **Firebase Integration**: Sends real-time sensor data to Firebase Realtime Database for remote monitoring.
- **MQTT Integration**: Publishes sensor data to Adafruit IO for cloud-based real-time monitoring.
- **LCD Display**: Displays sensor data on a 16x2 LCD screen with an I2C interface.
- **Buzzer Alert**: Sounds the buzzer when CO2 levels exceed a set threshold (420 PPM).

## Hardware Requirements
- **ESP8266** (e.g., NodeMCU, Wemos D1 Mini)
- **DHT22** (Temperature and Humidity Sensor)
- **MQ-135** (Air Quality Sensor for detecting gases like CO2, CO, NH4, Alcohol, etc.)
- **GPS Module** (e.g., NEO-6M GPS)
- **16x2 LCD Display with I2C**
- **Buzzer** (for CO2 concentration alerts)
- **Breadboard and Jumper wires** for connections

## Libraries Required
To use the provided code, you need to install the following libraries:
- **ESP8266WiFi**: For connecting the ESP8266 to Wi-Fi.
- **Firebase_ESP_Client**: For Firebase integration (Real-time Database).
- **Adafruit_MQTT**: For MQTT communication with Adafruit IO.
- **MQUnifiedsensor**: For interacting with MQ sensors (like MQ-135).
- **LiquidCrystal_I2C**: For controlling the LCD display via I2C.
- **TinyGPSPlus**: This is for reading GPS data.
- **SoftwareSerial**: For software-based serial communication with the GPS module.
- **DHT**: For reading data from the DHT22 sensor.
- **TokenHelper** and **RTDBHelper**: For Firebase token management and real-time database helper functions.

You can install these libraries via the **Arduino Library Manager**.

## Setup Instructions

### 1. Install the Required Libraries
In the Arduino IDE, go to **Sketch > Include Library > Manage Libraries**, and install the libraries listed in the **Libraries Required** section above.

### 2. Configure Wi-Fi Credentials
Open the code and modify the following lines with your Wi-Fi network credentials:
```cpp
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";
