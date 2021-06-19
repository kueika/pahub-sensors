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
//#include <Adafruit_NeoPixel.h>

// M5Stack ID
const int deviceId = 1;

// ncir val
uint16_t result;
float temperature;
// env2 val
float tmp = 0.0;
float hum = 0.0;
float pressure = 0.0;
//==============WiFi設定=================
// WiFi at home
//const char ssid[] = "Buffalo-G-862A";
//const char password[] = "b3k65fbn36arf";
// WiFi at CPSLAB
const char ssid[] = "NETGEAR83";
const char password[] = "festivemango615";

// Publish　お家ラズパイIP指定
const char* mqttHost = "192.168.1.100";
//const char* mqttHost = "192.168.1.11";
// cpslab用　都度変えるように
//const char* mqttHost = "192.168.1.16";
//=====================================
const int mqttPort = 1883;       // MQTTのポート
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
const char* topic = "pub/M5stack";     // 送信先のトピック名
char* payload;                   // 送信するデータ
// Subscribe
unsigned int count_rest;
char last_rest_time[] = "";
const char* sub_topic = "sub/M5stack";
String message = "";

boolean flag = false;

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

// toneEx
//  引数
//   frequency (Hz)
//   vol (0 ～ 9、0:無音 9:最大)
void toneEx(uint16_t frequency, uint16_t vol) {
  ledcSetup(TONE_PIN_CHANNEL, frequency, 10);
  ledcWrite(TONE_PIN_CHANNEL, 0x1FF >> (9 - vol));
}

// NoToneEx
void noToneEx() {
  ledcWriteTone(TONE_PIN_CHANNEL, 0);
  digitalWrite(SPEAKER_PIN, 0);
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
  WiFi.begin(ssid, password);
  Serial.println("WiFi connecting ....");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("connected");
  Serial.println(WiFi.localIP());
}

void connectMqtt() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT");
    String clientId = "M5stack-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
      // int qos = 0
      mqttClient.subscribe(sub_topic, 0);
      Serial.println("Subscribed.");
    }
    delay(1000);
    randomSeed(micros());
  }
}
void mqttCallback (char* topic, byte* payload, unsigned int length) {
  String Json = String((char*) payload);
  StaticJsonDocument<200> doc;
  DeserializationError err = deserializeJson(doc,Json);
  if(err){
    Serial.print("deserializeJson() failed: ");
    Serial.println(err.c_str());
    return;
  }
  const char* cmd = doc["message"];
  String cmd_s = String((cmd));
  Serial.println(cmd_s);
  if(cmd_s == "Please take a rest"){
    flag = true;
  }
  Serial.print("Flag is ");
  Serial.println(flag);
  Serial.print("\n");
  delay(300);

}
void showLCD() {
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextSize(2);
  M5.Lcd.println("THIS_M5_IP_ADDRESS: ");
  M5.Lcd.println(WiFi.localIP());
  M5.Lcd.print("ENV_TEMP=");
  M5.Lcd.println(tmp);
  M5.Lcd.print("ENV_HUMIDITY=");
  M5.Lcd.println(hum);
  M5.Lcd.print("BODY_TEMP=");
  M5.Lcd.println(temperature);
  M5.Lcd.print("rest_count=");
  M5.Lcd.println(count_rest);
  M5.Lcd.print("rest_count=");
  M5.Lcd.println(count_rest);
  M5.Lcd.print("Device_ID:");
  M5.Lcd.println(deviceId);
}

void setup() {
  M5.begin();
  M5.Power.begin();
  Wire.begin();
  M5.Lcd.fillScreen(TFT_BLACK);
  // 音
  ledcSetup(TONE_PIN_CHANNEL, 0, 10);
  ledcAttachPin(SPEAKER_PIN, TONE_PIN_CHANNEL);
  tca9548a.address(PaHub_I2C_ADDRESS);
  connectWifi();
  mqttClient.setServer(mqttHost, mqttPort);
  mqttClient.setCallback(mqttCallback);
  connectMqtt();
  PaHUBinit();
}

void mqttloop() {
  // MQTT
  if ( ! mqttClient.connected() ) {
    connectMqtt();
  }
  mqttClient.loop();
}


void loop() {
  PaHUB();
  if (M5.BtnA.isPressed() || M5.BtnB.isPressed() || M5.BtnC.isPressed()) {
    count_rest++;
    Serial.println("pressed!");
  }
  M5.update();
  showLCD();
  if (flag) {
    // beep 鳴らす処理
    Serial.println("BEEP!!!!");
    toneEx(440,1);
    flag = false;
    delay(1000);
    noToneEx();
  }
  char json[200];
  const size_t capacity = JSON_OBJECT_SIZE(5);
  StaticJsonDocument<capacity> doc;
  doc["env_temperature"] = tmp;
  doc["body_temperature"] = temperature;
  doc["rest_count"] = count_rest;
  doc["humidity"] = hum;
  doc["deviceID"] = deviceId;
  serializeJson(doc, json);
  mqttClient.publish(topic, json);
  delay(1000);
  // WiFi
  if ( WiFi.status() == WL_DISCONNECTED ) {
    connectWifi();
  }
  mqttloop();

}
