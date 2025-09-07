#include <Arduino.h>
#include <WiFiMulti.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "secrets.h"

#define DEVICE "ESP32"
#define TZ_INFO "AEST-10AEDT,M10.1.0,M4.1.0/3"
#define PIN_ANALOG_IN 32
#define SDA_PIN 21
#define SCL_PIN 22
#define BTN_TOP 13
#define BTN_MID 0
#define BTN_BOT 18
#define POT_PIN 34
#define SERVO_PIN 14
#define LED_RED_PIN 15
#define LED_GRN_PIN 5
#define LED_BLU_PIN 4
#define CH_RED 0
#define CH_GRN 1
#define CH_BLU 2

WiFiMulti wifiMulti;
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
Point sensor("SmartLock");
LiquidCrystal_I2C* lcdPtr;
Servo servo1;
BluetoothSerial SerialBT;

int keyValue = 0;
int keyInput = 0;
int lockStatus = 0;
bool firstTime = true;
char btBuf[32];
int btCnt = 0;

bool i2cAddrOk(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

void lcdWrite(const String& l1, const String& l2) {
  String a = l1 + "                ";
  String b = l2 + "                ";
  lcdPtr->setCursor(0,0);
  lcdPtr->print(a);
  lcdPtr->setCursor(0,1);
  lcdPtr->print(b);
}

void ledSetRed() {
  ledcWrite(CH_RED, 0);
  ledcWrite(CH_GRN, 255);
  ledcWrite(CH_BLU, 255);
}
void ledSetGreen() {
  ledcWrite(CH_RED, 255);
  ledcWrite(CH_GRN, 0);
  ledcWrite(CH_BLU, 255);
}
void ledSetBlue() {
  ledcWrite(CH_RED, 255);
  ledcWrite(CH_GRN, 255);
  ledcWrite(CH_BLU, 0);
}

float getTemperature() {
  int adc = analogRead(PIN_ANALOG_IN);
  if (adc <= 0 || adc >= 4095) return -100.0f;
  double v = (double)adc / 4095.0 * 3.3;
  double Rt = 10.0 * v / (3.3 - v);
  double tK = 1.0 / (1.0 / (273.15 + 25.0) + log(Rt / 10.0) / 3950.0);
  return (float)(tK - 273.15);
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(100);
  }

  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  client.validateConnection();
  sensor.addTag("device", DEVICE);
  sensor.addTag("location", "Unit");
  sensor.addTag("esp32_id", String(WiFi.BSSIDstr().c_str()));

  ledcSetup(CH_RED, 1000, 8);
  ledcSetup(CH_GRN, 1000, 8);
  ledcSetup(CH_BLU, 1000, 8);
  ledcAttachPin(LED_RED_PIN, CH_RED);
  ledcAttachPin(LED_GRN_PIN, CH_GRN);
  ledcAttachPin(LED_BLU_PIN, CH_BLU);
  ledSetBlue();

  pinMode(BTN_TOP, INPUT_PULLUP);
  pinMode(BTN_MID, INPUT_PULLUP);
  pinMode(BTN_BOT, INPUT_PULLUP);

  servo1.setPeriodHertz(50);
  servo1.attach(SERVO_PIN, 500, 2500);

  Wire.begin(SDA_PIN, SCL_PIN);
  uint8_t addr = i2cAddrOk(0x27) ? 0x27 : (i2cAddrOk(0x3F) ? 0x3F : 0x00);
  if (addr == 0x00) while (true) { delay(1000); }
  lcdPtr = new LiquidCrystal_I2C(addr, 16, 2);
  lcdPtr->init();
  lcdPtr->backlight();

  SerialBT.begin("SmartLockHub");
  lcdWrite("Welcome", "");
}

void loop() {
  float t = getTemperature();
  if (t > -90.0f) {
    sensor.clearFields();
    sensor.addField("temperature", t);
    if (wifiMulti.run() == WL_CONNECTED) {
      client.writePoint(sensor);
    }
  }

  keyInput = map(analogRead(POT_PIN), 0, 4095, 0, 180) / 10;
  lcdWrite("Current Input:", String(keyInput));

  if (firstTime) {
    lcdWrite("Set Key as", String(keyInput));
    if (digitalRead(BTN_MID) == LOW) {
      delay(200);
      if (digitalRead(BTN_MID) == LOW) {
        keyValue = keyInput;
        delay(4000);
        firstTime = false;
      }
    }
    return;
  }

  while (SerialBT.available()) {
    if (btCnt < (int)sizeof(btBuf) - 1) btBuf[btCnt++] = (char)SerialBT.read();
    else SerialBT.read();
  }
  if (btCnt > 0) {
    btBuf[btCnt] = 0;
  }

  bool cmdLock = (btCnt >= 4 && strncmp(btBuf, "Lock", 4) == 0);
  bool cmdUnlock = (btCnt >= 6 && strncmp(btBuf, "Unlock", 6) == 0);

  if (digitalRead(BTN_TOP) == LOW || cmdLock) {
    delay(200);
    if (digitalRead(BTN_TOP) == LOW || cmdLock) {
      ledSetRed();
      servo1.write(180);
      lcdWrite("Locked", "Press to Unlock");
      btCnt = 0;
      memset(btBuf, 0, sizeof(btBuf));
      lockStatus = 0;
      delay(4000);
    }
  } else if (digitalRead(BTN_MID) == LOW && lockStatus == 1) {
    delay(200);
    if (digitalRead(BTN_MID) == LOW && lockStatus == 1) {
      ledSetGreen();
      lcdWrite("Key Set", "");
      keyValue = map(analogRead(POT_PIN), 0, 4095, 0, 180) / 10;
      delay(4000);
    }
  } else if (digitalRead(BTN_BOT) == LOW || cmdUnlock) {
    delay(200);
    if (digitalRead(BTN_BOT) == LOW || cmdUnlock) {
      if (keyValue == keyInput) {
        ledSetBlue();
        servo1.write(0);
        lcdWrite("Unlocked", "Press to Lock");
        btCnt = 0;
        memset(btBuf, 0, sizeof(btBuf));
        lockStatus = 1;
        delay(4000);
      } else {
        lcdWrite("Incorrect Pin", "Please Try Again");
      }
    }
  }

  while (digitalRead(BTN_TOP) == LOW) {}
  while (digitalRead(BTN_MID) == LOW) {}
  while (digitalRead(BTN_BOT) == LOW) {}
}
