#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-9876-abcdefabcdef"

BLERemoteCharacteristic* pRemoteCharacteristic1;  // 타입을 BLERemoteCharacteristic로 변경
BLERemoteCharacteristic* pRemoteCharacteristic2;  // 두 번째 송신 ESP32의 특성

bool deviceFound1 = false;      // 첫 번째 송신 ESP32 발견 여부
bool deviceFound2 = false;      // 두 번째 송신 ESP32 발견 여부
BLEClient* pClient1 = nullptr;  // 첫 번째 송신 ESP32 클라이언트
BLEClient* pClient2 = nullptr;  // 두 번째 송신 ESP32 클라이언트

unsigned long lastTime = 0;
unsigned long currentTime;

int analogPin = 36;
int S0 = 0;
int S1 = 4;
int S2 = 16;

int hammer1 = 12;
int hammer2 = 14;
int hammer3 = 27;
int hammer4 = 26;
int hammer5 = 25;
int hammer6 = 33;
int hammer7 = 32;
int hammer8 = 35;

int rc1 = 22;
int rc2 = 19;
int rc3 = 23;
int rc4 = 18;

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");  // BLE 장치 초기화

  // BLE 스캔 시작
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);  // 활성 스캔
  pBLEScan->start(5);             // 5초 동안 스캔

  //for the sequencer
  pinMode(analogPin, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  pinMode(hammer1, INPUT);
  pinMode(hammer2, INPUT);
  pinMode(hammer3, INPUT);
  pinMode(hammer4, INPUT);
  pinMode(hammer5, INPUT);
  pinMode(hammer6, INPUT);
  pinMode(hammer7, INPUT);
  pinMode(hammer8, INPUT);
  pinMode(rc1, INPUT);
  pinMode(rc2, INPUT);
  pinMode(rc3, INPUT);
  pinMode(rc4, INPUT);
}

void loop() {
  currentTime = millis();

  if (!deviceFound1 || !deviceFound2) {
    // BLE 장치 스캔 및 연결
    BLEScanResults foundDevices = BLEDevice::getScan()->getResults();
    for (int i = 0; i < foundDevices.getCount(); i++) {
      BLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);
      String deviceName = String(advertisedDevice.getName().c_str());  // std::string -> String 변환

      // 첫 번째 송신 ESP32 찾기
      if (deviceName == "ESP32_Sensor_1" && !deviceFound1) {
        Serial.println("Found ESP32_Sensor_1");
        pClient1 = BLEDevice::createClient();
        BLEAddress addr1 = advertisedDevice.getAddress();
        if (pClient1->connect(addr1)) {
          Serial.println("Connected to ESP32_Sensor_1");
          BLERemoteService* pService1 = pClient1->getService(SERVICE_UUID);
          if (pService1 != nullptr) {
            pRemoteCharacteristic1 = pService1->getCharacteristic(CHARACTERISTIC_UUID);
            if (pRemoteCharacteristic1 != nullptr) {
              deviceFound1 = true;
            }
          }
        }
      }

      // 두 번째 송신 ESP32 찾기
      if (deviceName == "ESP32_Sensor_2" && !deviceFound2) {
        Serial.println("Found ESP32_Sensor_2");
        pClient2 = BLEDevice::createClient();
        BLEAddress addr2 = advertisedDevice.getAddress();
        if (pClient2->connect(addr2)) {
          Serial.println("Connected to ESP32_Sensor_2");
          BLERemoteService* pService2 = pClient2->getService(SERVICE_UUID);
          if (pService2 != nullptr) {
            pRemoteCharacteristic2 = pService2->getCharacteristic(CHARACTERISTIC_UUID);
            if (pRemoteCharacteristic2 != nullptr) {
              deviceFound2 = true;
            }
          }
        }
      }
    }
  }

  // 데이터를 주기적으로 읽기
  if (deviceFound1 && deviceFound2) {
    // 첫 번째 송신 ESP32에서 데이터 읽기
    if (pRemoteCharacteristic1 != nullptr) {
      std::string value1 = pRemoteCharacteristic1->readValue();
      Serial.print(String(value1.c_str()));
      Serial.print(" ");
    }
    // 두 번째 송신 ESP32에서 데이터 읽기
    if (pRemoteCharacteristic2 != nullptr) {
      std::string value2 = pRemoteCharacteristic2->readValue();
      Serial.print(String(value2.c_str()));
      Serial.print(" ");
    }
    for (int i = 0; i < 8; i++) {
      // 선택 핀에 값 설정
      digitalWrite(S0, (i & 0x01));  // LSB
      digitalWrite(S1, (i & 0x02) >> 1);
      digitalWrite(S2, (i & 0x04) >> 2);


      // 선택된 채널에서 아날로그 값 읽기
      int sensorValue = analogRead(analogPin);
      Serial.print(sensorValue);
      Serial.print(" ");
    }
    Serial.print(digitalRead(hammer1));
    Serial.print(" ");
    Serial.print(digitalRead(hammer2));
    Serial.print(" ");
    Serial.print(digitalRead(hammer3));
    Serial.print(" ");
    Serial.print(digitalRead(hammer4));
    Serial.print(" ");
    Serial.print(digitalRead(hammer5));
    Serial.print(" ");
    Serial.print(digitalRead(hammer6));
    Serial.print(" ");
    Serial.print(digitalRead(hammer7));
    Serial.print(" ");
    Serial.print(digitalRead(hammer8));
    Serial.print(" ");
    Serial.print(!digitalRead(rc1));
    Serial.print(" ");
    Serial.print(!digitalRead(rc2));
    Serial.print(" ");
    Serial.print(!digitalRead(rc3));
    Serial.print(" ");
    Serial.print(!digitalRead(rc4));
    Serial.print(" ");
    Serial.println();
  }
  delay(10);
}
