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
```

### 3. Configure Firebase
Set up a **Firebase project** in the Firebase Console, and then replace the placeholders in the code with your own **API Key**, **Database URL**, and **User credentials**:
```cpp
#define API_KEY "YOUR_API_KEY"
#define DATABASE_URL "https://your-database-url.firebaseio.com/"
#define USER_EMAIL "YOUR_FIREBASE_EMAIL"
#define USER_PASSWORD "YOUR_FIREBASE_PASSWORD"
```

### 4. Configure Adafruit IO
Create an account on Adafruit IO and create a new feed for each sensor data type. Then, replace the placeholders with your Adafruit IO Username and Adafruit IO Key:
```cpp
#define AIO_USERNAME "YOUR_AIO_USERNAME"
#define AIO_KEY "YOUR_AIO_KEY"
```

### 5. Upload the Code
Once you've configured your Wi-Fi, Firebase, and Adafruit IO settings, upload the code to your ESP8266 using the Arduino IDE.

### 6. Monitor the System
Once the code is uploaded:

- **LCD Display**LCD Display: The sensor data (temperature, humidity, gas concentrations, and GPS coordinates) will be displayed on the LCD screen in a rotating cycle.
- **Buzzer**: The buzzer will sound when CO2 levels exceed the threshold of 420 PPM.
- **Firebase & Adafruit IO**: The sensor data will be uploaded to Firebase and Adafruit IO for cloud-based monitoring.

### Data Sent to Firebase
The following sensor data is sent to Firebase:

- Latitude and Longitude (from GPS)
- Temperature (in °C)
- Humidity (in %)
- CO (in PPM)
- CO2 (in PPM)
- NH4 (in PPM)
- Alcohol (in PPM)
- Toluene (in PPM)
- Acetone (in PPM)
The data is stored in Firebase under the path:
/stations/data/{station_id}

### Data Sent to Adafruit IO
The following data is sent to Adafruit IO via MQTT:

- Temperature (in °C)
- Humidity (in %)
- CO (in PPM)
- CO2 (in PPM)
- NH4 (in PPM)
- Alcohol (in PPM)

### Troubleshooting
**Wi-Fi Connection**

If your ESP8266 is not connecting to Wi-Fi:
Make sure that the SSID and Password are correct.
Ensure that your Wi-Fi network is working and within range.

**GPS Not Acquiring Fix**

Ensure your GPS module has a clear line of sight to the sky.
If the GPS is not providing valid data, check the wiring and ensure proper power supply.

**MQ-135 Calibration** 

If the MQ135 sensor is not providing stable readings:
Try recalibrating the sensor by following the calibration code in the calibrateMQ135() function.
Ensure the sensor is not exposed to high levels of pollutants during calibration.

**Buzzer Not Triggering** 

If the buzzer doesn't sound when CO2 levels exceed the threshold:
Ensure the threshold for CO2 is set correctly in the controlBuzzer() function.
Check that the buzzer wiring is correct and functioning.

### Future Improvements

- **Mobile App**: Build a mobile app to display and analyze the data in real-time.
- **Additional Sensors**: Add more sensors (e.g., PM2.5, sound, light) to monitor additional environmental parameters.
- **Data Logging**: Implement features to log data locally on an SD card or an external database for offline storage.


### Contribution:
1. **Team Section**: Added the `NodeMasters` team name under the **Team** heading.
2. **Author Section**:[**David King Mazimpaka**](https://github.com/DavidkingMazimpaka), as the lead creator of the Team.
