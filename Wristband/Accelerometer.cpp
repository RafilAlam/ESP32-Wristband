#include <PulseSensorPlayground.h>
#include "LIS3DHTR.h"
#include "MAX30105.h"
#include "heartRate.h"
#include <Wire.h>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID "03e0be01-38fe-41c8-a7e5-c294732282b7"
#define CHARACTERISTIC_UUID "d5b00152-835a-4bf0-91c6-d9e15ebb270c"

int ResetPin = 0;
int BuzzerPin = 4;

LIS3DHTR<TwoWire> LIS;
MAX30105 particleSensor;

// Accelerometer
unsigned long lastaccelupdate = 0;
unsigned long maxfreefallTime = 1000;
unsigned long lastfreefallTime = 0;
unsigned long lastfall = 0;
float freefallThreshold = 0.5;
float impactThreshold = 5.0;
float mag = 0;

bool fallen = false;
bool freefalling = false;

// Buzzer
enum BuzzerState {
  BUZZER_IDLE,
  BUZZER_ALARM,
  BUZZER_REMIND,
  BUZZER_PING
};

BuzzerState currentBuzzerState = BUZZER_IDLE;

struct BuzzerPattern {
  uint16_t intervals[6];
  uint16_t totalSteps;
  uint16_t currentStep;
  unsigned long lastToggle;
};

BuzzerPattern alarmPattern  = { {0, 50}, 2, 0, 0};
BuzzerPattern remindPattern = { {0, 100, 200, 100}, 4, 0, 0};
BuzzerPattern pingPattern   = { {0, 30}, 2, 0, 0};

void updateBuzzer(unsigned long mstime) {
  BuzzerPattern* activePattern = NULL;

  if (fallen && mstime - lastfall > 5000) {
    currentBuzzerState = BUZZER_ALARM;
  } else if (currentBuzzerState == BUZZER_ALARM) {
    currentBuzzerState = BUZZER_IDLE;
    digitalWrite(BuzzerPin, 0);
  }

  switch(currentBuzzerState) {
    case BUZZER_ALARM:       activePattern = &alarmPattern; break;
    case BUZZER_REMIND:     activePattern = &remindPattern; break;
    case BUZZER_PING:  activePattern = &pingPattern; break;
    default:
      digitalWrite(BuzzerPin, 0);
      return;
  }

  if (mstime - activePattern->lastToggle >= activePattern->intervals[activePattern->currentStep]) {
    activePattern->lastToggle = mstime;

    bool buzzerOn = (activePattern->currentStep % 2 == 0);
    digitalWrite(BuzzerPin, buzzerOn);

    activePattern->currentStep++;

    if (activePattern->currentStep >= activePattern->totalSteps) {
      activePattern->currentStep = 0;

      if (currentBuzzerState == BUZZER_PING || currentBuzzerState == BUZZER_REMIND) {
        currentBuzzerState = BUZZER_IDLE;
        digitalWrite(BuzzerPin, 0);
      }
    }
  }
}

// Pulse Sensor
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// Bluetooth
unsigned long LastUpdate = 0;
struct __attribute__((packed)) SensorPayload {
  bool fallen;
  uint8_t heartRate;
};

SensorPayload myData;

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

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
      int command = value[0];
      switch(command) {
        case 1:
          pingPattern.currentStep = 0;
          currentBuzzerState = BUZZER_PING;
          break;
        case 2:
          pingPattern.currentStep = 0;
          currentBuzzerState = BUZZER_REMIND;
          return;
      }
    }
  }
};

unsigned long mstime = 0;
void setup() {
  Serial.begin(115200);

  pinMode(BuzzerPin, OUTPUT);
  pinMode(ResetPin, INPUT_PULLUP);

    // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    //Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  //Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  LIS.begin(Wire, 0x19);
  delay(100);
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  LIS.setHighSolution(true);

  if (!LIS) {
    //Serial.println("LIS3DHTR had problems connecting.");
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
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  //Serial.println("BLE Device is advertising! Connect your phone.");
}

void loop() {
  mstime = millis();

  // Pulse Sensor
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = mstime - lastBeat;
    lastBeat = mstime;

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  //Serial.print("IR=");
  //Serial.print(irValue);
  //Serial.print(", BPM=");
  //Serial.print(beatsPerMinute);
  //Serial.print(", Avg BPM=");
  //Serial.print(beatAvg);

  //if (irValue < 50000)
    //Serial.print(" No finger?");

  //Serial.println();

  // Accelerometer
  if (mstime - lastaccelupdate > 20) {
    lastaccelupdate = mstime;
    float x = LIS.getAccelerationX();
    float y = LIS.getAccelerationY();
    float z = LIS.getAccelerationZ();
    mag = sqrt(x*x + y*y + z*z);
    
    //Serial.print(0);
    //Serial.print(" ");
    //Serial.println(18);

    //Serial.print("magnitude:"); Serial.println(mag);
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
      lastfall = mstime;
      pingPattern.currentStep = 0;
      currentBuzzerState = BUZZER_PING;
    }
  }

  updateBuzzer(mstime);

  if (digitalRead(ResetPin) == 0) {
    if (fallen) {
      pingPattern.currentStep = 0;
      currentBuzzerState = BUZZER_PING;
    }
    fallen = false;
  }

  if (mstime - LastUpdate > 200) {
    LastUpdate = mstime;
    if (deviceConnected) {
      myData.fallen = fallen;
      myData.heartRate = beatAvg;

      uint8_t* bytePointer = (uint8_t*)&myData;
      size_t payloadSize = sizeof(SensorPayload);
      
      pCharacteristic->setValue(bytePointer, payloadSize);
      pCharacteristic->notify();

      //Serial.println("Sent " + newValue);
    }
  }
}
