#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Define UUIDs (use lowercase for compatibility)
#define SERVICE_UUID "79daf682-341b-42b5-42b5-891a-1647a8a9517b"
#define CHARACTERISTIC_UUID_RX "daa5f483-1420-4f26-9095-165d8fc6a321"
#define CHARACTERISTIC_UUID_TX "b6f055b0-cb3f-4c99-8098-2a793916bada"

BLEServer *pServer = NULL;
BLEService *pService = NULL;
BLECharacteristic *pCharacteristicRX = NULL;
BLECharacteristic *pCharacteristicTX = NULL;

bool deviceConnected = false;
bool advertising = false; // Add this flag

// Placeholder functions for spectrophotometer control
String turnOnRed() {
  // Replace with code to turn on red LED
  Serial.println("Turning on Red LED");
  return "Red LED ON";
}

String turnOnGreen() {
  // Replace with code to turn on green LED
  Serial.println("Turning on Green LED");
  return "Green LED ON";
}

String turnOnBlue() {
  // Replace with code to turn on blue LED
  Serial.println("Turning on Blue LED");
  return "Blue LED ON";
}

String setZero() {
  // Replace with code to set zero
  Serial.println("Setting Zero");
  return "Zero Set";
}

String takeReading() {
  // Replace with code to take a reading
  float absorbance = 0.543; // Placeholder for absorbance value
  Serial.print("Taking Reading. Absorbance: ");
  Serial.println(absorbance);
  return "d:" + String(absorbance); // Prefix with "d:" to identify data
}


// Characteristic Callback: Handles writes to the RX characteristic
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.print("Received Value: ");
      Serial.println(rxValue.c_str());

      String command = String(rxValue.c_str());

      if (command == "TURN_ON_RED") {
        String result = turnOnRed();
        pCharacteristicTX->setValue(result.c_str());
        pCharacteristicTX->notify();
        Serial.println(result);
      } else if (command == "TURN_ON_GREEN") {
        String result = turnOnGreen();
        pCharacteristicTX->setValue(result.c_str());
        pCharacteristicTX->notify();
        Serial.println(result);
      } else if (command == "TURN_ON_BLUE") {
        String result = turnOnBlue();
        pCharacteristicTX->setValue(result.c_str());
        pCharacteristicTX->notify();
        Serial.println(result);
      } else if (command == "SET_ZERO") {
        String result = setZero();
        pCharacteristicTX->setValue(result.c_str());
        pCharacteristicTX->notify();
        Serial.println(result);
      } else if (command == "TAKE_READING") {
        String result = takeReading();
        pCharacteristicTX->setValue(result.c_str());
        pCharacteristicTX->notify();
        Serial.println(result);
      } else {
        Serial.println("Unknown command");
      }
    }
  }
};

void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("ESP32-Spectro"); // Give your ESP32 a name

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  pService = pServer->createService(BLEUUID(SERVICE_UUID));

  // Create the RX Characteristic (for receiving data from the web app)
  pCharacteristicRX = pService->createCharacteristic(
      BLEUUID(CHARACTERISTIC_UUID_RX),
      BLECharacteristic::PROPERTY_WRITE);
  pCharacteristicRX->setCallbacks(new MyCallbacks());

  // Create the TX Characteristic (for sending data to the web app)
  pCharacteristicTX = pService->createCharacteristic(
      BLEUUID(CHARACTERISTIC_UUID_TX),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTX->addDescriptor(new BLE2902()); // Needed for notifications

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising(); // Use pServer, not BLEDevice
  pAdvertising->addServiceUUID(BLEUUID(SERVICE_UUID));
  pAdvertising->setScanResponse(false); // Keep advertising simple
  pAdvertising->setMinPreferred(0x0);     // Allow any connection interval
  pAdvertising->setMaxPreferred(0x0);

  BLEDevice::startAdvertising();
  advertising = true; // Set advertising flag
  Serial.println("Advertising started");
}

void loop() {
  delay(2000);
}
