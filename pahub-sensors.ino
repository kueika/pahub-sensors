#include <Wire.h>
#include "ClosedCube_TCA9548A.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
// ENV2 lib
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "SHT3X.h"
#include <M5Stack.h>
SHT3X sht30;
Adafruit_BMP280 bme;
// hub settings
#define PaHub_I2C_ADDRESS  0x70
ClosedCube::Wired::TCA9548A tca9548a;

// ncir val
uint16_t result;
float temperature;
// env2 val
float tmp = 0.0;
float hum = 0.0;
float pressure = 0.0;
// WiFi
const char ssid[] = "";
const char passwd[] = "";
// Pub/Sub
const char* mqttHost = "192.168.1.200"; // MQTTのIPかホスト名
const int mqttPort = 1883;       // MQTTのポート
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
const char* topic = "pub/M5stack";     // 送信先のトピック名
char* payload;                   // 送信するデータ

uint8_t ncirSensing() {
  Wire.beginTransmission(0x5A);          // Send Initial Signal and I2C Bus Address
  Wire.write(0x07);                      // Send data only once and add one address automatically.
  Wire.endTransmission(false);           // Stop signal
  Wire.requestFrom(0x5A, 2);             // Get 2 consecutive data from 0x5A, and the data is stored only.
  result = Wire.read();                  // Receive DATA
  result |= Wire.read() << 8;            // Receive DATA
  temperature = result * 0.02 - 273.15;
  return temperature;
}

void bmpSensing() {
  pressure = bme.readPressure();
  if (sht30.get() == 0) {
    tmp = sht30.cTemp;
    hum = sht30.humidity;
  }
  Serial.print("pressure =");
  Serial.println(pressure);
  Serial.print("tmp = ");
  Serial.println(tmp);
  Serial.print("hum = ");
  Serial.println(hum);
}


void PaHUB() {
  uint8_t returnCode = 0;
  uint8_t address;
  for ( uint8_t channel = 0; channel < TCA9548A_MAX_CHANNELS; channel++ ) {
    returnCode = tca9548a.selectChannel(channel);

    // this channel value equivalents to pahub number
    switch (channel) {
      case 0:
        Serial.println("ncir value");
        Serial.println(ncirSensing());
        break;
      case 1:
        break;
      case 2:
        // ENV2 function write here
        Serial.println("ENV2 value");
        bmpSensing();
        break;
      case 3:
        break;
      case 4:
        break;
      case 5:
        break;
    }
    delay(1000);
  }
}

void PaHUBinit() {
  uint8_t returnCode = 0;
  uint8_t address;
  for ( uint8_t channel = 0; channel < TCA9548A_MAX_CHANNELS; channel++ ) {
    returnCode = tca9548a.selectChannel(channel);
    if ( returnCode == 0 ) {
      for (address = 0x01; address < 0x7F; address++ ) {
        Wire.beginTransmission(address);
        returnCode = Wire.endTransmission();
        if (returnCode == 0) {
          Serial.print(address);
          Serial.println("done");
        }
      }
    }
    delay(1000);
  }
}

void connectWifi() {
  WiFi.begin();
  Serial.println("WiFi connecting ....");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("connected");
  Serial.println(WiFi.localIP());
}

void connectMqtt() {
  mqttClient.setServer(mqttHost, mqttPort);
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT");
    String clientId = "M5stack-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
    }
    delay(1000);
    randomSeed(micros());
  }
}

void showLCD() {
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextSize(2);
  M5.Lcd.print("ENV_TEMP=");
  M5.Lcd.println(tmp);
  M5.Lcd.print("ENV_HUMIDITY=");
  M5.Lcd.println(hum);
  M5.Lcd.print("ENV_PRESS=");
  M5.Lcd.println(pressure);
  M5.Lcd.print("BODY_TEMP=");
  M5.Lcd.println(temperature);
}

void setup() {
  M5.begin();
  M5.Power.begin();
  Wire.begin();
  M5.Lcd.fillScreen(TFT_BLACK);
  tca9548a.address(PaHub_I2C_ADDRESS);
  connectWifi();
  connectMqtt();
  PaHUBinit();
}

void loop() {
  M5.update();
  PaHUB();
  showLCD();
  char json[200];
  const size_t capacity = JSON_OBJECT_SIZE(2);
  StaticJsonDocument<capacity> doc;
  doc["env_temperature"] = tmp;
  doc["body_temperature"] = temperature;
  serializeJson(doc, json);
  mqttClient.publish(topic,json);
  delay(1000);
  // WiFi
  if ( WiFi.status() == WL_DISCONNECTED ) {
    connectWii();
  }
  // MQTT
  if ( ! mqttClient.connected() ) {
    connectMqtt();
  }
  mqttClient.loop();

}
