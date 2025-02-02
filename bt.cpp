#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Define UUIDs (use lowercase for compatibility)
#define SERVICE_UUID           "79daf682-341b-42b5-891a-1647a8a9517b"
#define CHARACTERISTIC_UUID_RX "daa5f483-1420-4f26-9095-165d8fc6a321"
#define CHARACTERISTIC_UUID_TX "b6f055b0-cb3f-4c99-8098-2a793916bada"

BLEServer *pServer = NULL;
BLEService *pService = NULL;
BLECharacteristic *pCharacteristicRX = NULL;
BLECharacteristic *pCharacteristicTX = NULL;

bool deviceConnected = false;
bool advertising = false; // Add this flag

// Server Callback: Handles connection and disconnection
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
    // Restart advertising after disconnect
    pServer->startAdvertising();
    advertising = true; // Set advertising flag
    Serial.println("Restarted advertising");
  }
};

// Characteristic Callback: Handles writes to the RX characteristic
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]);
      }
      Serial.println();

      // Echo the received data back to the client (optional)
      pCharacteristicTX->setValue(rxValue);
      pCharacteristicTX->notify();
      Serial.println("Sent echoed value back to client");
    }
  }
};

void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("ESP32-WebBluetooth"); // Give your ESP32 a name

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  pService = pServer->createService(BLEUUID(SERVICE_UUID));

  // Create the RX Characteristic (for receiving data from the web app)
  pCharacteristicRX = pService->createCharacteristic(
    BLEUUID(CHARACTERISTIC_UUID_RX),
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristicRX->setCallbacks(new MyCallbacks());

  // Create the TX Characteristic (for sending data to the web app)
  pCharacteristicTX = pService->createCharacteristic(
    BLEUUID(CHARACTERISTIC_UUID_TX),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristicTX->addDescriptor(new BLE2902()); // Needed for notifications

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising(); // Use pServer, not BLEDevice
  pAdvertising->addServiceUUID(BLEUUID(SERVICE_UUID));
  pAdvertising->setScanResponse(false); // Keep advertising simple
  pAdvertising->setMinPreferred(0x0);    // Allow any connection interval
  pAdvertising->setMaxPreferred(0x0);

  BLEDevice::startAdvertising();
  advertising = true; // Set advertising flag
  Serial.println("Advertising started");
}

void loop() {
  // Send notifications periodically when connected
  if (deviceConnected) {
    static unsigned long lastNotifyTime = 0; 
    if (millis() - lastNotifyTime > 5000) {
      
      // Correctly create the notification message using c_str()
      std::string notificationMessage = "Notification from ESP32 at ";
      notificationMessage += String(millis()).c_str(); 

      // Set the characteristic value and notify
      pCharacteristicTX->setValue(notificationMessage);
      pCharacteristicTX->notify();

      Serial.println("Notification sent: " + String(notificationMessage.c_str()));

      lastNotifyTime = millis(); 
    }
  }

  // Print connection status periodically
  if (deviceConnected) {
    Serial.println("Device is connected");
  } else if (advertising) {
    Serial.println("Advertising for connections...");
  }

  delay(10); 
}
