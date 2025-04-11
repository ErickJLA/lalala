#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLECharacteristic.h>
#include <String>
#include <Wire.h> // Make sure this is included
#include <cmath>  // Include for log10 if calculateAbsorbance uses it
#include <limits.h> // Include for UINT16_MAX

// ============================================
// definitions apds start
// ============================================

#define APDS9930_I2C_ADDR 0x39
#define AUTO_INCREMENT 0xA0
#define ERROR 0xFF
#define APDS9930_ID_1 0x12
#define APDS9930_ID_2 0x39
#define APDS9930_ENABLE 0x00
#define APDS9930_ATIME 0x01
#define APDS9930_CONTROL 0x0F
#define APDS9930_ID 0x12
#define APDS9930_Ch0DATAL 0x14
#define APDS9930_Ch0DATAH 0x15
#define APDS9930_Ch1DATAL 0x16
#define APDS9930_Ch1DATAH 0x17
#define APDS9930_PON 0b00000001
#define APDS9930_AEN 0b00000010
#define OFF 0
#define ON 1
#define POWER 0
#define AMBIENT_LIGHT 1
#define AGAIN_1X 0
#define AGAIN_8X 1
#define AGAIN_16X 2
#define AGAIN_120X 3

// ============================================
// definitions apds end
// ============================================

// Define BLE Service and Characteristic UUIDs (must match the web interface)
#define SERVICE_UUID "79daf682-341b-42b5-891a-1647a8a9517b"
#define CHARACTERISTIC_TX_UUID "b6f055b0-cb3f-4c99-8098-2a793916bada" // Transmit (ESP32 -> Web)
#define CHARACTERISTIC_RX_UUID "daa5f483-1420-4f26-9095-165d8fc6a321" // Receive (Web -> ESP32)

// Define LED pins
const int redLEDPin = 27;
const int greenLEDPin = 26;
const int blueLEDPin = 25;

// BLE Globals
BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
BLECharacteristic *pRxCharacteristic = nullptr;
BLEAdvertising *pAdvertising = nullptr;

bool deviceConnected = false;
// String receivedMessage = ""; // This seems unused

// ============================================
// global variables apds start
// ============================================

uint16_t ch0_reading; // Still used for single reads in loop()
uint16_t ch1_reading; // Unused in current logic
uint16_t zeroReading = 0; // Initialize zero reading

// ============================================
// global variables apds end
// ============================================

// --- Forward Declarations ---
bool initAPD();
bool setIntegrationTimePeriods(uint8_t periods);
bool setAmbientLightGain(uint8_t gain);
bool readCh0Light(uint16_t &val);
bool readCh1Light(uint16_t &val);
bool setMode(uint8_t mode, uint8_t enable);
uint8_t getMode();
bool enablePower();
bool wireWriteDataByte(uint8_t reg, uint8_t val);
bool wireReadDataByte(uint8_t reg, uint8_t &val);
bool wireWriteByte(uint8_t val);
void optimizeSensorSettings();
float calculateAbsorbance(uint16_t sampleReading);
uint16_t performMultisampling(int numSamples, int delayBetweenSamples); // New function


// ============================================
// helper functions start
// ============================================

// ... (initAPD, setIntegrationTimePeriods, setAmbientLightGain, readCh1Light, setMode, getMode, enablePower, I2C functions remain the same as esp32_code_minimal_zero_confirm_v1) ...
bool initAPD()
{
  uint8_t id;
  if (!wireReadDataByte(APDS9930_ID, id))
  {
    Serial.println("Failed to read APDS ID!");
    return false;
  }
  if (!(id == APDS9930_ID_1 || id == APDS9930_ID_2))
  {
    Serial.print("Incorrect APDS ID: 0x");
    Serial.println(id, HEX);
    return false;
  }
 
  // Using original default settings from user code
  if (!setIntegrationTimePeriods(220)) // Original value
  {
    Serial.println("Failed to set APDS default integration time!");
    return false;
  }
  if (!setAmbientLightGain(AGAIN_1X)) // Original value
  {
    Serial.println("Failed to set APDS default gain!");
    return false;
  }
  if (!enablePower())
  {
    Serial.println("Failed to enable APDS power!");
    return false;
  }
  delay(10);
  if (!setMode(AMBIENT_LIGHT, ON))
  {
    Serial.println("Failed to enable APDS ambient light sensor!");
    return false;
  }
  delay(120);
  return true;
}

bool setIntegrationTimePeriods(uint8_t periods)
{
  // Original calculation
  uint8_t atime_val = 256 - periods;
  if (!wireWriteDataByte(APDS9930_ATIME, atime_val))
  {
    return false;
  }
  return true;
}

bool setAmbientLightGain(uint8_t gain)
{
  uint8_t control_val;
  if (!wireReadDataByte(APDS9930_CONTROL, control_val))
  {
    return false;
  }
  gain &= 0b00000011;
  control_val &= 0b11111100;
  control_val |= gain;
  if (!wireWriteDataByte(APDS9930_CONTROL, control_val))
  {
    return false;
  }
  return true;
}

bool readCh0Light(uint16_t &val)
{
  uint8_t val_low, val_high;
  val = 0;
  if (!wireReadDataByte(APDS9930_Ch0DATAL, val_low))
    return false;
  if (!wireReadDataByte(APDS9930_Ch0DATAH, val_high))
    return false;
  val = (uint16_t)val_high << 8 | val_low;
  return true;
}

bool readCh1Light(uint16_t &val)
{
  uint8_t val_low, val_high;
  val = 0;
  if (!wireReadDataByte(APDS9930_Ch1DATAL, val_low))
    return false;
  if (!wireReadDataByte(APDS9930_Ch1DATAH, val_high))
    return false;
  val = (uint16_t)val_high << 8 | val_low;
  return true;
}

bool setMode(uint8_t mode, uint8_t enable)
{
  uint8_t reg_val = getMode();
  if (reg_val == ERROR)
    return false;
  enable &= 0x01;
  if (mode == POWER || mode == AMBIENT_LIGHT)
  {
    if (enable)
      reg_val |= (1 << mode);
    else
      reg_val &= ~(1 << mode);
  }
  else
  {
    return false;
  }
  if (!wireWriteDataByte(APDS9930_ENABLE, reg_val))
    return false;
  return true;
}

uint8_t getMode()
{
  uint8_t enable_value;
  if (!wireReadDataByte(APDS9930_ENABLE, enable_value))
    return ERROR;
  return enable_value;
}

bool enablePower()
{
  return setMode(POWER, ON);
}

bool wireWriteDataByte(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(APDS9930_I2C_ADDR);
  Wire.write(reg | AUTO_INCREMENT);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool wireReadDataByte(uint8_t reg, uint8_t &val)
{
  if (!wireWriteByte(reg | AUTO_INCREMENT))
    return false;
  Wire.requestFrom(APDS9930_I2C_ADDR, 1);
  if (Wire.available())
  {
    val = Wire.read();
    return true;
  }
  return false;
}

bool wireWriteByte(uint8_t val)
{
  Wire.beginTransmission(APDS9930_I2C_ADDR);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}


// ============================================
// Original function: optimizeSensorSettings
// ============================================
uint8_t currentIntegrationTime = 255;

// ============================================
// Original function: calculateAbsorbance
// ============================================
float calculateAbsorbance(uint16_t sampleReading)
{
  if (zeroReading == 0)
  {
    Serial.println("Error: Zero reading not set!");
    return -1.0; // Return error indicator instead of 0.0
  }
  if (sampleReading == 0) {
      Serial.println("Warning: Sample reading is zero. Absorbance is effectively infinite.");
      return 99.0; // Indicate near infinite absorbance
  }
  //if (sampleReading >= zeroReading) {
      // If sample is brighter than or equal to zero reading, absorbance is zero or negative (error)
      // Serial.println("Warning: Sample reading >= Zero reading. Setting absorbance to 0.");
    //  return 0.0;
  //}

  float transmittance = (float)sampleReading / (float)zeroReading;

  if (transmittance <= 0.0f) {
      Serial.print("Error: Invalid transmittance calculated: ");
      Serial.println(transmittance, 6);
      return -1.0; // Return error indicator
  }

  float absorbance = -log10f(transmittance); // Use log10f for float

  if (isnan(absorbance) || isinf(absorbance)) {
      Serial.println("Error: Absorbance calculation resulted in NaN or Infinity.");
      return -1.0; // Return error indicator
  }
  return absorbance;
}

// ============================================
// NEW FUNCTION: performMultisampling
// Takes multiple readings and returns the average.
// Returns 0 if all reads fail.
// ============================================
uint16_t performMultisampling(int numSamples = 5, int delayBetweenSamples = 50) {
    unsigned long totalReading = 0;
    int successfulReads = 0;
    uint16_t currentSampleReading = 0;

    Serial.print("Performing multisampling (");
    Serial.print(numSamples);
    Serial.println(" samples)...");

    for (int i = 0; i < numSamples; i++) {
        if (readCh0Light(currentSampleReading)) {
            totalReading += currentSampleReading;
            successfulReads++;
            // Serial.print("Sample "); Serial.print(i+1); Serial.print(": "); Serial.println(currentSampleReading); // Optional: print each sample
        } else {
            Serial.print("Multisampling: Read failed on sample ");
            Serial.println(i + 1);
            // Optionally add a small retry delay here? For now, just skip.
        }
        delay(delayBetweenSamples); // Delay between samples
    }

    if (successfulReads > 0) {
        uint16_t averageReading = (uint16_t)(totalReading / successfulReads);
        Serial.print("Multisampling successful. Average: ");
        Serial.println(averageReading);
        return averageReading;
    } else {
        Serial.println("Multisampling failed: No successful reads.");
        return 0; // Indicate failure
    }
}


// ============================================
// helper functions end
// ============================================

// --- BLE Server Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServerInstance) // Renamed parameter
  {
    deviceConnected = true;
    Serial.println("Client connected");
    if (pAdvertising != nullptr) {
        pAdvertising->stop();
        Serial.println("Advertising stopped.");
    } else {
        Serial.println("Warning: pAdvertising object is null on connect.");
    }
  };

  void onDisconnect(BLEServer *pServerInstance)
  {
    deviceConnected = false;
    Serial.println("Client disconnected");
     if (pAdvertising != nullptr) {
        BLEDevice::startAdvertising(); // Use standard function to restart
        Serial.println("Advertising restarted.");
    } else {
        Serial.println("Error: pAdvertising object is null on disconnect!");
    }
  }
};

// --- BLE Characteristic Callbacks ---
class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    String rxValueString = pCharacteristic->getValue(); // Original method

    if (rxValueString.length() > 0)
    {
      Serial.print("Received: ");
      Serial.println(rxValueString);

      if (rxValueString == "READ_SENSOR") {
          // --- MODIFICATION: Use multisampling for sample reading ---
          uint16_t averagedSampleReading = performMultisampling(5, 50); // Example: 5 samples, 50ms delay

          if (averagedSampleReading > 0) { // Check if multisampling was successful
              Serial.print("Averaged Ch0: ");
              Serial.println(averagedSampleReading);

              // Use the averaged reading for absorbance calculation
              float absorbance = calculateAbsorbance(averagedSampleReading);

              // Check if absorbance calculation was valid
              if (absorbance >= 0.0 || absorbance < 0.0) { // Basic check if it's a number
                  Serial.print("Absorbance (formatted): ");
                  char absorbanceString[20];
                  dtostrf(absorbance, 2, 4, absorbanceString);
                  Serial.println(absorbanceString);

                  Serial.print("Absorbance (raw): ");
                  Serial.println(absorbance, 4);

                  // Send absorbance prefixed with 'd:'
                  String dataString = "d:" + String(absorbanceString);
                  if (pTxCharacteristic != nullptr) {
                      pTxCharacteristic->setValue((uint8_t*)dataString.c_str(), dataString.length());
                      pTxCharacteristic->notify();
                  }

                  // Send continuous update prefixed with 'a:' (using the same averaged value)
                   String continuousDataString = "a:" + String(absorbanceString);
                   if (pTxCharacteristic != nullptr) {
                       pTxCharacteristic->setValue((uint8_t*)continuousDataString.c_str(), continuousDataString.length());
                       pTxCharacteristic->notify();
                   }
              } else {
                   Serial.println("Absorbance calculation failed after multisampling.");
                   String errorMsg = "Error: Absorbance calc failed";
                   if (pTxCharacteristic != nullptr) {
                       pTxCharacteristic->setValue((uint8_t*)errorMsg.c_str(), errorMsg.length());
                       pTxCharacteristic->notify();
                   }
              }
          } else {
              Serial.println("Multisampling failed for READ_SENSOR");
              String errorMsg = "Error: Sensor read failed (multi)";
               if (pTxCharacteristic != nullptr) {
                   pTxCharacteristic->setValue((uint8_t*)errorMsg.c_str(), errorMsg.length());
                   pTxCharacteristic->notify();
               }
          }
          // --- End of READ_SENSOR modification ---
      }
      else if (rxValueString == "SET_ZERO") // Original SET_ZERO block
      {
        Serial.println("SET_ZERO command received (integrated into LED commands)");
        String logMessage = "SET_ZERO command received (integrated)";
        if (pTxCharacteristic != nullptr) {
            pTxCharacteristic->setValue((uint8_t *)logMessage.c_str(), logMessage.length());
            pTxCharacteristic->notify();
        }
      }
      else if (rxValueString == "LED_RED_ON"){
        
        int periods = 150;
        digitalWrite(redLEDPin, HIGH);
        digitalWrite(greenLEDPin, LOW);
        digitalWrite(blueLEDPin, LOW);
        Serial.println("Red LED ON");
        delay(250);
        setIntegrationTimePeriods(periods);
        setAmbientLightGain(AGAIN_8X);
        readCh0Light(ch0_reading);
        delay(periods*3);

        uint16_t averagedZeroReading = performMultisampling(3, periods * 3);
        zeroReading = averagedZeroReading;

        readCh0Light(ch0_reading);
        Serial.println(calculateAbsorbance(ch0_reading));

        //while(calculateAbsorbance(ch0_reading)!= 0){  
          while (calculateAbsorbance(ch0_reading) > 0.0001 || calculateAbsorbance(ch0_reading) < -0.0001){ 
          averagedZeroReading = performMultisampling(3, periods * 3);
          zeroReading = averagedZeroReading;
          delay(periods * 3);
          readCh0Light(ch0_reading);
          Serial.println("calculated again");
          Serial.println(calculateAbsorbance(ch0_reading));
        }

        
        String zeroDoneMessage = "z:DONE";
        if (pTxCharacteristic != nullptr) {
        pTxCharacteristic->setValue((uint8_t*)zeroDoneMessage.c_str(), zeroDoneMessage.length());
        pTxCharacteristic->notify();
        Serial.println("Sent: z:DONE");
        }


      }
      else if (rxValueString == "LED_GREEN_ON"){
        int periods = 150;
        digitalWrite(redLEDPin, LOW);
        digitalWrite(greenLEDPin, HIGH);
        digitalWrite(blueLEDPin, LOW);
        Serial.println("Green LED ON");
        delay(250);
        setIntegrationTimePeriods(periods);
        setAmbientLightGain(AGAIN_8X);
        readCh0Light(ch0_reading);
        delay(periods*3);

        uint16_t averagedZeroReading = performMultisampling(3, periods * 3);
        zeroReading = averagedZeroReading;

        readCh0Light(ch0_reading);
        Serial.println(calculateAbsorbance(ch0_reading));

        //while(calculateAbsorbance(ch0_reading)!= 0){  
          while (calculateAbsorbance(ch0_reading) > 0.0001 || calculateAbsorbance(ch0_reading) < -0.0001){
          averagedZeroReading = performMultisampling(3, periods * 3);
          zeroReading = averagedZeroReading;
          delay(periods * 3);
          readCh0Light(ch0_reading);
          Serial.println("calculated again");
          Serial.println(calculateAbsorbance(ch0_reading));
        }

        
        String zeroDoneMessage = "z:DONE";
        if (pTxCharacteristic != nullptr) {
        pTxCharacteristic->setValue((uint8_t*)zeroDoneMessage.c_str(), zeroDoneMessage.length());
        pTxCharacteristic->notify();
        Serial.println("Sent: z:DONE");
        }

      }
      else if (rxValueString == "LED_BLUE_ON"){
        int periods = 150;
        digitalWrite(redLEDPin, LOW);
        digitalWrite(greenLEDPin, LOW);
        digitalWrite(blueLEDPin, HIGH);
        Serial.println("Blue LED ON");
        delay(250);
        setIntegrationTimePeriods(periods);
        setAmbientLightGain(AGAIN_8X);
        readCh0Light(ch0_reading);
        delay(periods*3);

        uint16_t averagedZeroReading = performMultisampling(3, periods * 3);
        zeroReading = averagedZeroReading;

        readCh0Light(ch0_reading);
        Serial.println(calculateAbsorbance(ch0_reading));

        //while(calculateAbsorbance(ch0_reading)!= 0){  
          while (calculateAbsorbance(ch0_reading) > 0.0001 || calculateAbsorbance(ch0_reading) < -0.0001){
          averagedZeroReading = performMultisampling(3, periods * 3);
          zeroReading = averagedZeroReading;
          delay(periods * 3);
          readCh0Light(ch0_reading);
          Serial.println("calculated again");
          Serial.println(calculateAbsorbance(ch0_reading));
        }

        
        String zeroDoneMessage = "z:DONE";
        if (pTxCharacteristic != nullptr) {
        pTxCharacteristic->setValue((uint8_t*)zeroDoneMessage.c_str(), zeroDoneMessage.length());
        pTxCharacteristic->notify();
        Serial.println("Sent: z:DONE");
        }

      }
      else // Original unknown command handler
      {
          String logMessage = "Received unknown command: " + rxValueString;
           if (pTxCharacteristic != nullptr) {
              pTxCharacteristic->setValue((uint8_t *)logMessage.c_str(), logMessage.length());
              pTxCharacteristic->notify();
           }
      }
    }
  }
};

// --- Arduino Setup Function ---
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BLE server!");

  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(blueLEDPin, LOW);

  Wire.begin(); // Initialize I2C

  // ============================================
  // setup apds start
  // ============================================
  if (!initAPD())
  {
    Serial.println("APDS-9930 Initialization Failed! Halting.");
    while (1); // Stop execution if sensor fails
  }
  else
  {
    Serial.println("APDS-9930 Initialized Successfully.");
  }
  // Original optional settings from user code
  setAmbientLightGain(AGAIN_1X);
  setIntegrationTimePeriods(200);
  delay(120);
  // ============================================
  // setup apds end
  // ============================================

  // --- Initialize BLE ---
  BLEDevice::init("ESP32_SP"); // Using shorter name from previous suggestion
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // TX Characteristic
  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLEDescriptor(BLEUUID((uint16_t)0x2902))); // CCCD Descriptor

  // RX Characteristic
  pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR // Allow write without response
  );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  // --- Advertising ---
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true); // Set to true if name is short enough for UUID
  BLEDevice::startAdvertising();

  Serial.println("Waiting for a client connection to notify...");
  zeroReading = 0;
}

// --- Arduino Loop Function ---
void loop()
{
  // --- Periodic Absorbance Update (Single Sample - Unchanged) ---
  static unsigned long lastAbsorbanceUpdateTime = 0;
  const unsigned long absorbanceUpdateInterval = currentIntegrationTime * 2.78;
  unsigned long currentMillis = millis();

  if (currentMillis - lastAbsorbanceUpdateTime >= absorbanceUpdateInterval) {
      lastAbsorbanceUpdateTime = currentMillis;
      if (deviceConnected && zeroReading > 0 && pTxCharacteristic != nullptr) {
          // Use global ch0_reading for loop updates
          if (readCh0Light(ch0_reading)) {
              float absorbance = calculateAbsorbance(ch0_reading);
              // Basic check if calculation is valid
              if (absorbance >= 0.0 || absorbance < 0.0) {
                  char absorbanceString[10];
                  dtostrf(absorbance, 1, 4, absorbanceString);
                  String continuousDataString = "a:" + String(absorbanceString);
                  pTxCharacteristic->setValue((uint8_t*)continuousDataString.c_str(), continuousDataString.length());
                  pTxCharacteristic->notify();
              }
          }
      }
  }
  // --- End of Periodic Absorbance Update ---

  delay(10); // Original delay
}
