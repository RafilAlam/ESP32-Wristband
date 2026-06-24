#include <PulseSensorPlayground.h>
#include "LIS3DHTR.h"
#include <Wire.h>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID "03e0be01-38fe-41c8-a7e5-c294732282b7"
#define CHARACTERISTIC_UUID "d5b00152-835a-4bf0-91c6-d9e15ebb270c"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->getAdvertising()->start();
  }
};

LIS3DHTR<TwoWire> LIS;

int ResetPin = 10;
int BuzzerPin = 0;

unsigned long lastbuzzertoggle = 0;
unsigned long buzzpulsewidth = 50;
bool buzzing = false;

unsigned long maxfreefallTime = 1000;
unsigned long lastfreefallTime = 0;
float freefallThreshold = 0.5;
float impactThreshold = 5.0;

bool fallen = false;
bool freefalling = false;

unsigned long mstime = 0;
void setup() {
  Serial.begin(115200);

  pinMode(BuzzerPin, OUTPUT);
  pinMode(ResetPin, INPUT_PULLUP);

  LIS.begin(Wire, 0x19);
  delay(100);
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  LIS.setHighSolution(true);

  if (!LIS) {
    Serial.println("LIS3DHTR had problems connecting.");
    while(1);
    return;
  }

  BLEDevice::init("ESP32-Wristband");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  Serial.println("BLE Device is advertising! Connect your phone.");
}

void loop() {
  mstime = millis();

  float x = LIS.getAccelerationX();
  float y = LIS.getAccelerationY();
  float z = LIS.getAccelerationZ();
  float mag = sqrt(x*x + y*y + z*z);
  
  Serial.print(0);
  Serial.print(" ");
  Serial.println(18);

  Serial.print("magnitude:"); Serial.println(mag);
  if (mag < freefallThreshold) {
    freefalling = true;
    lastfreefallTime = mstime;
  }

  if (mstime - lastfreefallTime > maxfreefallTime) {
    freefalling = false;
  }

  if (freefalling && mag > impactThreshold) {
    fallen = true;
    freefalling = false;
  }

  if (fallen && mstime - lastbuzzertoggle > buzzpulsewidth) {
    lastbuzzertoggle = mstime;
    buzzing = !buzzing;
    digitalWrite(BuzzerPin, buzzing);
  }

  if (digitalRead(ResetPin) == 0) {
    digitalWrite(BuzzerPin, 0);
    fallen = false;
    buzzing = false;
  }

  if (deviceConnected) {
    String newValue = "Acceleration Magnitude: " + String(mag);
    pCharacteristic->setValue(newValue.c_str());
    pCharacteristic->notify();

    Serial.println("Sent " + newValue);
  }

  delay(20);
}
