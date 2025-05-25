#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-9876-abcdefabcdef"

// Remote characteristics for two transmitter ESP32 devices
BLERemoteCharacteristic* pRemoteCharacteristic1;
BLERemoteCharacteristic* pRemoteCharacteristic2;

bool deviceFound1 = false;      // Whether the first transmitter ESP32 is found
bool deviceFound2 = false;      // Whether the second transmitter ESP32 is found
BLEClient* pClient1 = nullptr;  // BLE client for the first transmitter ESP32
BLEClient* pClient2 = nullptr;  // BLE client for the second transmitter ESP32

unsigned long lastTime = 0;
unsigned long currentTime;

// Multiplexer and sensor pins
int analogPin = 36;
int S0 = 0;
int S1 = 4;
int S2 = 16;

// Hammer sensor pins
int hammer1 = 12;
int hammer2 = 14;
int hammer3 = 27;
int hammer4 = 26;
int hammer5 = 25;
int hammer6 = 33;
int hammer7 = 32;
int hammer8 = 35;

// Reed switch pins
int rc1 = 22;
int rc2 = 19;
int rc3 = 23;
int rc4 = 18;

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");  // Initialize BLE device

  // Start BLE scan
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);  // Enable active scan
  pBLEScan->start(5);             // Scan for 5 seconds

  // Set up pins for the sequencer (multiplexer)
  pinMode(analogPin, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  // Set up hammer sensor pins
  pinMode(hammer1, INPUT);
  pinMode(hammer2, INPUT);
  pinMode(hammer3, INPUT);
  pinMode(hammer4, INPUT);
  pinMode(hammer5, INPUT);
  pinMode(hammer6, INPUT);
  pinMode(hammer7, INPUT);
  pinMode(hammer8, INPUT);

  // Set up reed switch pins
  pinMode(rc1, INPUT);
  pinMode(rc2, INPUT);
  pinMode(rc3, INPUT);
  pinMode(rc4, INPUT);
}

void loop() {
  currentTime = millis();

  // If either transmitter ESP32 is not found, scan and connect
  if (!deviceFound1 || !deviceFound2) {
    BLEScanResults foundDevices = BLEDevice::getScan()->getResults();
    for (int i = 0; i < foundDevices.getCount(); i++) {
      BLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);
      String deviceName = String(advertisedDevice.getName().c_str());

      // Find the first transmitter ESP32
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

      // Find the second transmitter ESP32
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

  // Periodically read data if both devices are connected
  if (deviceFound1 && deviceFound2) {
    // Read data from the first transmitter ESP32
    if (pRemoteCharacteristic1 != nullptr) {
      std::string value1 = pRemoteCharacteristic1->readValue();
      Serial.print(String(value1.c_str()));
      Serial.print(" ");
    }
    // Read data from the second transmitter ESP32
    if (pRemoteCharacteristic2 != nullptr) {
      std::string value2 = pRemoteCharacteristic2->readValue();
      Serial.print(String(value2.c_str()));
      Serial.print(" ");
    }
    // Read analog values from 8 multiplexer channels
    for (int i = 0; i < 8; i++) {
      // Set selection pins for the multiplexer
      digitalWrite(S0, (i & 0x01));        // LSB
      digitalWrite(S1, (i & 0x02) >> 1);
      digitalWrite(S2, (i & 0x04) >> 2);

      // Read analog value from the selected channel
      int sensorValue = analogRead(analogPin);
      Serial.print(sensorValue);
      Serial.print(" ");
    }
    // Read hammer sensor digital values
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
    // Read reed switch digital values (inverted)
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
  delay(10); // Small delay for stability
}