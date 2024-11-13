#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <MQUnifiedsensor.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "DHT.h"

// Provide the token generation process info.
#include <addons/TokenHelper.h>
// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

/* 2. Define the API Key */
#define API_KEY "AIzaSyCMDs8Q24r9mIjxSjF6sXJgVaH5msBjK4U"
/* 3. Define the RTDB URL */
#define DATABASE_URL "https://air-quality-monitoring-app-default-rtdb.firebaseio.com/"

/* 4. Define the user Email and password that already registered or added in your project */
#define USER_EMAIL "bunsen1plus@gmail.com"
#define USER_PASSWORD "bunsen1plus@gmail.com"

#define AIO_USERNAME "IshimweWilliam"
#define AIO_KEY "aio_LJbT46K6Af5xKQpnVqA3Ta4s8T4i"

// MQTT Broker
#define MQTT_SERVER "io.adafruit.com"
#define MQTT_PORT 1883
#define MQTT_USER AIO_USERNAME
#define MQTT_PASS AIO_KEY

#define DHTPIN D5
#define DHTTYPE DHT22
#define BUZZER_PIN D6
#define MQ_135_PIN A0
#define DISPLAY_DELAY 2000  // Adjusted for less frequent updates

#define STATION_NAME "CMU-Africa"
#define STATION_ID "2"

#define placa "ESP8266"
#define Voltage_Resolution 5
#define type "MQ-135"
#define ADC_Bit_Resolution 10
#define RatioMQ135CleanAir 3.6

#define RX_PIN D7
#define TX_PIN D8

// Wi-Fi credentials
char ssid[] = "Bunsen Wi-Fi";
char pass[] = "bunsenplus";

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASS);

// MQTT feeds
Adafruit_MQTT_Publish gpslatlng_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gps/csv");
Adafruit_MQTT_Publish temperature_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidity_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish co2_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/co2");
Adafruit_MQTT_Publish co_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/co");
Adafruit_MQTT_Publish nh4_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/nh4");
Adafruit_MQTT_Publish alcohol_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/alcohol");

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

TinyGPSPlus gps;
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, MQ_135_PIN, type);
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial gpsSerial(RX_PIN, TX_PIN);

// Variables to manage timing
unsigned long previousDisplayMillis = 0;
unsigned long displayInterval = DISPLAY_DELAY;
unsigned long pushInterval = 60000;
unsigned long lastMQTTPublishMillis = 0;
unsigned long lastRTDBublishMillis = 0;
const unsigned long MQTT_PUBLISH_INTERVAL = 30000;  // 30 seconds
int displayCycle = 0;
unsigned long lastGPSRead = 0;
const unsigned long GPS_READ_INTERVAL = 2000;  // 2 second interval

String dataPath = String("/stations/data/") + STATION_ID;

float speed_mph = 0;
float altitude = 0;
float currentLat = 0.0;
float currentLng = 0.0;
char gpsdata[120];


void calibrateMQ135();
void connectAdafruit();
void checkWiFiConnection();
float readDHTHumidity();
float readDHTTemperature();
void readGPSData();
float readGasConcentration(float a, float b);
void displaySensorData(float hum, float temp, float CO, float CO2, float NH4, float Alcohol, float Toluen, float Aceton);
void controlBuzzer(float CO2);
void sendDataToFirebase(float lat, float lng, float hum, float temp, float CO, float CO2, float NH4, float Alcohol, float Toluen, float Aceton);
void updateMQTT(float hum, float temp, float CO, float CO2, float NH4, float Alcohol);


void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Air Quality");
  lcd.setCursor(3, 1);
  lcd.print("Monitoring");

  Serial.begin(9600);
  dht.begin();

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Wi-Fi connected!");

  mqtt.connect();

  gpsSerial.begin(9600);
  delay(1000);

  MQ135.init();
  MQ135.setRegressionMethod(1);

  pinMode(BUZZER_PIN, OUTPUT);
  calibrateMQ135();

  // connect to adafruit io
  connectAdafruit();

  //configure Firebase
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  Firebase.reconnectNetwork(true);
  Firebase.begin(&config, &auth);
  Serial.println("Firebase setup done.");

  // Clear any existing data
  while (gpsSerial.available()) {
    gpsSerial.read();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!mqtt.ping(3)) {
    // reconnect to adafruit io
    if (!mqtt.connected())
      connectAdafruit();
  }

  mqtt.processPackets(1000);  // Ensure MQTT keeps working
  checkWiFiConnection();

  float humidity = readDHTHumidity();
  float temperature = readDHTTemperature();
  readGPSData();

  float CO = readGasConcentration(605.18, -3.937);         // CO concentration
  float CO2 = readGasConcentration(110.47, -2.862) + 400;  // CO2 concentration
  float NH4 = readGasConcentration(102.2, -2.473);         // NH4 concentration
  float Alcohol = readGasConcentration(77.255, -3.18);     // Alcohol concentration
  float Toluen = readGasConcentration(44.947, -3.445);     // Toluen concentration
  float Aceton = readGasConcentration(34.668, -3.369);     // Aceton concentration

  displaySensorData(humidity, temperature, CO, CO2, NH4, Alcohol, Toluen, Aceton);
  controlBuzzer(CO2);

  if (Firebase.ready()) {
    if (millis() - lastRTDBublishMillis >= pushInterval) {
      lastRTDBublishMillis = millis();
      if (gps.location.isValid()) {
        sendDataToFirebase(currentLat, currentLng, humidity, temperature, CO, CO2, NH4, Alcohol, Toluen, Aceton);
      }
    }
  }

  // Only read and publish data every n seconds
  if (millis() - lastMQTTPublishMillis >= MQTT_PUBLISH_INTERVAL) {
    lastMQTTPublishMillis = millis();
    if (gps.location.isValid()) {
      updateMQTT(humidity, temperature, CO, CO2, NH4, Alcohol);
    }
  }
}

void connectAdafruit() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if (ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));

    clearLCDLine(0);
    clearLCDLine(1);

    lcd.setCursor(0, 0);
    lcd.print("Connection Lost");
    lcd.setCursor(0, 1);
    lcd.print("Retrying...");

    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

void getCoordinates() {
  char *p = gpsdata;
  // add speed value
  dtostrf(speed_mph, 2, 6, p);
  p += strlen(p);
  p[0] = ',';
  p++;

  // concat latitude
  dtostrf(currentLat, 2, 6, p);
  p += strlen(p);
  p[0] = ',';
  p++;

  // concat longitude
  dtostrf(currentLng, 3, 6, p);
  p += strlen(p);
  p[0] = ',';
  p++;

  // concat altitude
  dtostrf(altitude, 2, 6, p);
  p += strlen(p);

  // null terminate
  p[0] = 0;
}

void calibrateMQ135() {
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!");

  if (isinf(calcR0) || calcR0 == 0) {
    Serial.println("Warning: Connection issue detected with MQ135.");
    while (1)
      ;
  }
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connection: Lost");
    lcd.setCursor(0, 1);
    lcd.print("Check Wi-Fi...");
    delay(DISPLAY_DELAY);
  }
}

float readDHTHumidity() {
  float humidity = dht.readHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return 0.0;
  }
  return humidity;
}

float readDHTTemperature() {
  float temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return 0.0;
  }
  return temperature;
}

void readGPSData() {
  unsigned long startTime = millis();

  // Read GPS data for up to 1000ms or until valid data is received
  while (millis() - startTime < 1000) {
    while (gpsSerial.available()) {
      if (gps.encode(gpsSerial.read())) {
        // If we have new valid data, process it
        if (gps.location.isValid()) {
          currentLat = gps.location.lat();
          currentLng = gps.location.lng();
          altitude = gps.altitude.value();  // Get altitude from GPS
          speed_mph = gps.speed.mph();      // Get speed from GPS in km/h
          Serial.print("GPS data: Lat=");
          Serial.print(currentLat, 6);
          Serial.print(" Long=");
          Serial.println(currentLng, 6);
          return;  // Exit if we got valid data
        }
      }
    }
  }

  // Check if we haven't received any characters
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected. Check wiring.");
  } else if (!gps.location.isValid()) {
    Serial.println("Waiting for GPS fix...");
  }
}

float readGasConcentration(float a, float b) {
  MQ135.setA(a);
  MQ135.setB(b);
  MQ135.update();
  return MQ135.readSensor();
}

void updateMQTT(float hum, float temp, float CO, float CO2, float NH4, float Alcohol) {
  temperature_feed.publish(temp);
  humidity_feed.publish(hum);
  co2_feed.publish(CO2);
  co_feed.publish(CO);
  nh4_feed.publish(NH4);
  alcohol_feed.publish(Alcohol);

  getCoordinates();
  gpslatlng_feed.publish(gpsdata);
}

void sendDataToFirebase(float lat, float lng, float hum, float temp, float CO, float CO2, float NH4, float Alcohol, float Toluen, float Aceton) {
  FirebaseJson jsonData;
  jsonData.set("station_name", STATION_NAME);
  jsonData.set("latitude", lat);
  jsonData.set("longitude", lng);
  jsonData.set("humidity", hum);
  jsonData.set("temperature", temp);
  jsonData.set("CO", CO);
  jsonData.set("CO2", CO2);
  jsonData.set("NH4", NH4);
  jsonData.set("Alcohol", Alcohol);
  jsonData.set("Toluen", Toluen);
  jsonData.set("Aceton", Aceton);

  if (Firebase.RTDB.push(&fbdo, dataPath, &jsonData)) {
    Serial.println("Data sent successfully to Firebase!");
  } else {
    Serial.println("Failed to send data to Firebase: " + fbdo.errorReason());
  }
}


void displaySensorData(float hum, float temp, float CO, float CO2, float NH4, float Alcohol, float Toluen, float Aceton) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousDisplayMillis >= displayInterval) {
    previousDisplayMillis = currentMillis;
    clearLCDLine(0);
    clearLCDLine(1);

    switch (displayCycle) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(temp);
        lcd.print("C");
        lcd.setCursor(0, 1);
        lcd.print("Humidity: ");
        lcd.print(hum);
        lcd.print("%");
        break;
      case 1:
        lcd.setCursor(0, 0);
        lcd.print("CO2: ");
        lcd.print(CO2);
        lcd.print(" PPM");
        lcd.setCursor(0, 1);
        lcd.print("CO: ");
        lcd.print(CO);
        lcd.print(" PPM");
        break;
      case 2:
        lcd.setCursor(0, 0);
        lcd.print("NH4: ");
        lcd.print(NH4);
        lcd.print(" PPM");
        lcd.setCursor(0, 1);
        lcd.print("Alcohol: ");
        lcd.print(Alcohol);
        lcd.print(" PPM");
        break;
      case 3:
        lcd.setCursor(0, 0);
        lcd.print("Toluen: ");
        lcd.print(Toluen);
        lcd.print(" PPM");
        lcd.setCursor(0, 1);
        lcd.print("Aceton: ");
        lcd.print(Aceton);
        lcd.print(" PPM");
        break;
      case 4:  // New case for GPS coordinates
        lcd.setCursor(0, 0);
        lcd.print("Lat: ");
        if (gps.location.isValid()) lcd.print(currentLat, 6);
        else lcd.print("-----");

        lcd.setCursor(0, 1);
        lcd.print("Lng: ");
        if (gps.location.isValid()) lcd.print(currentLng, 6);
        else lcd.print("-----");
        break;
    }
    displayCycle = (displayCycle + 1) % 5;  // Changed to 5 to include GPS display
    // GPS Data on Serial Monitor
    Serial.print("\nTemp: ");
    Serial.print(temp);
    Serial.print("C\tHumidity: ");
    Serial.print(hum);
    Serial.print("%\tCO2: ");
    Serial.print(CO2);
    Serial.print(" PPM\n");
  }
}

void clearLCDLine(int line) {
  lcd.setCursor(0, line);
  for (int i = 0; i < 16; i++) {
    lcd.print(" ");
  }
}

void controlBuzzer(float CO2) {
  if (CO2 > 420) {
    tone(BUZZER_PIN, 1000);  // Sound the buzzer if CO2 is above threshold
  } else {
    noTone(BUZZER_PIN);
  }
}
