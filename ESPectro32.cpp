#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLECharacteristic.h>
#include <String>
#include <Wire.h> // Make sure this is included
// ============================================
// definitions apds start
// ============================================

#define APDS9930_I2C_ADDR       0x39
#define AUTO_INCREMENT        0xA0
#define ERROR               0xFF
#define APDS9930_ID_1       0x12
#define APDS9930_ID_2       0x39
#define APDS9930_ENABLE       0x00
#define APDS9930_ATIME        0x01
#define APDS9930_CONTROL      0x0F
#define APDS9930_ID         0x12
#define APDS9930_Ch0DATAL     0x14
#define APDS9930_Ch0DATAH     0x15
#define APDS9930_Ch1DATAL     0x16
#define APDS9930_Ch1DATAH     0x17
#define APDS9930_PON          0b00000001
#define APDS9930_AEN          0b00000010
#define OFF                 0
#define ON                  1
#define POWER               0
#define AMBIENT_LIGHT       1
#define AGAIN_1X            0
#define AGAIN_8X            1
#define AGAIN_16X           2
#define AGAIN_120X          3

// ============================================
// definitions apds end
// ============================================

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLECharacteristic.h>
#include <String>

// Define BLE Service and Characteristic UUIDs (must match the web interface)
#define SERVICE_UUID        "79daf682-341b-42b5-891a-1647a8a9517b"
#define CHARACTERISTIC_TX_UUID "b6f055b0-cb3f-4c99-8098-2a793916bada" // Transmit (ESP32 -> Web)
#define CHARACTERISTIC_RX_UUID "daa5f483-1420-4f26-9095-165d8fc6a321" // Receive (Web -> ESP32)

// Define LED pins
const int redLEDPin = 27;
const int greenLEDPin = 26;
const int blueLEDPin = 25;

BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
BLECharacteristic* pRxCharacteristic = nullptr;

bool deviceConnected = false;
String receivedMessage = "";

// ============================================
// global variables apds start
// ============================================

uint16_t ch0_reading;
uint16_t ch1_reading;
uint16_t zeroReading; 

// ============================================
// global variables apds end
// ============================================

// ============================================
// helper functions start (COPY THIS ENTIRE BLOCK)
// ============================================

bool initAPD() {
    uint8_t id;
    if (!wireReadDataByte(APDS9930_ID, id)) {
        Serial.println("Failed to read APDS ID!");
        return false;
    }
    if (!(id == APDS9930_ID_1 || id == APDS9930_ID_2)) {
        Serial.print("Incorrect APDS ID: 0x");
        Serial.println(id, HEX);
        return false;
    }
    if (!setIntegrationTimePeriods(220)) {
        Serial.println("Failed to set APDS default integration time!");
        return false;
    }
    if (!setAmbientLightGain(AGAIN_1X)) {
        Serial.println("Failed to set APDS default gain!");
        return false;
    }
    if (!enablePower()) {
        Serial.println("Failed to enable APDS power!");
        return false;
    }
    delay(10);
    if (!setMode(AMBIENT_LIGHT, ON)) {
        Serial.println("Failed to enable APDS ambient light sensor!");
        return false;
    }
    delay(120);
    return true;
}

bool setIntegrationTimePeriods(uint8_t periods) {
    uint8_t atime_val = 256 - periods;
    if (!wireWriteDataByte(APDS9930_ATIME, atime_val)) {
        return false;
    }
    return true;
}

bool setAmbientLightGain(uint8_t gain) {
    uint8_t control_val;
    if (!wireReadDataByte(APDS9930_CONTROL, control_val)) {
        return false;
    }
    gain &= 0b00000011;
    control_val &= 0b11111100;
    control_val |= gain;
    if (!wireWriteDataByte(APDS9930_CONTROL, control_val)) {
        return false;
    }
    return true;
}

bool readCh0Light(uint16_t &val) {
    uint8_t val_low, val_high;
    val = 0;
    if (!wireReadDataByte(APDS9930_Ch0DATAL, val_low)) return false;
    if (!wireReadDataByte(APDS9930_Ch0DATAH, val_high)) return false;
    val = (uint16_t)val_high << 8 | val_low;
    return true;
}

bool readCh1Light(uint16_t &val) {
    uint8_t val_low, val_high;
    val = 0;
    if (!wireReadDataByte(APDS9930_Ch1DATAL, val_low)) return false;
    if (!wireReadDataByte(APDS9930_Ch1DATAH, val_high)) return false;
    val = (uint16_t)val_high << 8 | val_low;
    return true;
}

bool setMode(uint8_t mode, uint8_t enable) {
    uint8_t reg_val = getMode();
    if (reg_val == ERROR) return false;
    enable &= 0x01;
    if (mode == POWER || mode == AMBIENT_LIGHT) {
        if (enable) reg_val |= (1 << mode);
        else reg_val &= ~(1 << mode);
    } else {
        return false;
    }
    if (!wireWriteDataByte(APDS9930_ENABLE, reg_val)) return false;
    return true;
}

uint8_t getMode() {
    uint8_t enable_value;
    if (!wireReadDataByte(APDS9930_ENABLE, enable_value)) return ERROR;
    return enable_value;
}

bool enablePower() {
    return setMode(POWER, ON);
}

bool wireWriteDataByte(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(APDS9930_I2C_ADDR);
    Wire.write(reg | AUTO_INCREMENT);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool wireReadDataByte(uint8_t reg, uint8_t &val) {
    if (!wireWriteByte(reg | AUTO_INCREMENT)) return false;
    Wire.requestFrom(APDS9930_I2C_ADDR, 1);
    if (Wire.available()) {
        val = Wire.read();
        return true;
    }
    return false;
}

bool wireWriteByte(uint8_t val) {
    Wire.beginTransmission(APDS9930_I2C_ADDR);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

// ============================================
// New function: optimizeSensorSettings
// ============================================

void optimizeSensorSettings() {
    uint8_t currentGain = AGAIN_1X;
    uint8_t currentIntegrationTime = 255; // Maximum integration time (256 - 1)
    uint16_t targetValue = 50000; // Adjust as needed
    uint16_t maxAllowedValue = 60000;
    uint16_t minIntegrationTime = 10; // Minimum usable integration time

    // 1. Try with minimum gain and maximum integration time
    setAmbientLightGain(currentGain);
    setIntegrationTimePeriods(currentIntegrationTime);
    delay(150);
    readCh0Light(ch0_reading);
    Serial.print("Gain: ");
    Serial.print(currentGain);
    Serial.print(" Integration Time: ");
    Serial.print(currentIntegrationTime);
    Serial.print(" Ch0: ");
    Serial.println(ch0_reading);

    if (ch0_reading < targetValue) {
        // 2. Increase Gain if needed
        while (currentGain < AGAIN_120X && ch0_reading < targetValue) {
            currentGain++;
            setAmbientLightGain(currentGain);
            delay(150);
            readCh0Light(ch0_reading);
            Serial.print("Gain: ");
            Serial.print(currentGain);
            Serial.print(" Ch0: ");
            Serial.println(ch0_reading);
        }
    }

    // 3. Adjust Integration Time
    if (ch0_reading > targetValue) {
        while (currentIntegrationTime < 255 && ch0_reading > targetValue) {
            currentIntegrationTime++;
            setIntegrationTimePeriods(currentIntegrationTime);
            delay(150);
            readCh0Light(ch0_reading);
            Serial.print("Integration Time: ");
            Serial.print(currentIntegrationTime);
            Serial.print(" Ch0: ");
            Serial.println(ch0_reading);
        }
        while (currentIntegrationTime > minIntegrationTime && ch0_reading > targetValue) {
            currentIntegrationTime--;
            setIntegrationTimePeriods(currentIntegrationTime);
            delay(150);
            readCh0Light(ch0_reading);
            Serial.print("Integration Time: ");
            Serial.print(currentIntegrationTime);
            Serial.print(" Ch0: ");
            Serial.println(ch0_reading);
        }
    } else if (ch0_reading < targetValue) {
        while (currentIntegrationTime > minIntegrationTime && ch0_reading < targetValue) {
            currentIntegrationTime--;
            setIntegrationTimePeriods(currentIntegrationTime);
            delay(150);
            readCh0Light(ch0_reading);
            Serial.print("Integration Time: ");
            Serial.print(currentIntegrationTime);
            Serial.print(" Ch0: ");
            Serial.println(ch0_reading);
        }
    }

    Serial.print("Optimized Gain: ");
    Serial.println(currentGain);
    Serial.print("Optimized Integration Time: ");
    Serial.println(currentIntegrationTime);
}

// ============================================
// New function: calculateAbsorbance
// ============================================

float calculateAbsorbance(uint16_t sampleReading) {
    if (zeroReading == 0) {
        Serial.println("Error: Zero reading not set!");
        return 0.0; // Or some other error value
    }

    float transmittance = (float)sampleReading / zeroReading;
    if (transmittance <= 0.0) {
        Serial.println("Error: Invalid transmittance value!");
        return 0.0; // Or some other error value
    }
    float absorbance = -log10(transmittance);
    return absorbance;
}

// ============================================
// helper functions end
// ============================================

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Client connected");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Client disconnected");
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String rxValueString = pCharacteristic->getValue();

        if (rxValueString.length() > 0) {
            Serial.print("Received: ");
            Serial.println(rxValueString);

            if (rxValueString == "READ_SENSOR") {
                // --- Read APDS Sensor ---
                if (readCh0Light(ch0_reading)) { // Only reading Ch0
                    Serial.print("Ch0: ");
                    Serial.println(ch0_reading);

                    // ***
                    // IMPORTANT: This is where you need to implement your
                    // absorbance calculation.
                    // ***
                    float absorbance = calculateAbsorbance(ch0_reading); // Calculate absorbance
                    Serial.print("Absorbance: ");
                    Serial.println(absorbance);

                    String dataString = "d:" + String(absorbance); // Send absorbance
                    pTxCharacteristic->setValue((uint8_t*)dataString.c_str(), dataString.length());
                    pTxCharacteristic->notify();
                } else {
                    String errorMessage = "Error reading APDS light data!";
                    pTxCharacteristic->setValue((uint8_t*)errorMessage.c_str(), errorMessage.length());
                    pTxCharacteristic->notify();
                    Serial.println(errorMessage);
                }
            } else if (rxValueString == "SET_ZERO") {
                // --- SET_ZERO Implementation ---

                Serial.println("SET_ZERO command received");

                // 1. Optimize Sensor Settings
                optimizeSensorSettings(); // Function to adjust gain/integration time

                // 2. Take Baseline Reading
                if (readCh0Light(ch0_reading)) {
                    zeroReading = ch0_reading; // Store the baseline
                    Serial.print("Zero reading set to: ");
                    Serial.println(zeroReading);

                    String logMessage = "Zero reading set";
                    pTxCharacteristic->setValue((uint8_t*)logMessage.c_str(), logMessage.length());
                    pTxCharacteristic->notify();
                } else {
                    String errorMessage = "Error reading sensor for zeroing!";
                    pTxCharacteristic->setValue((uint8_t*)errorMessage.c_str(), errorMessage.length());
                    pTxCharacteristic->notify();
                    Serial.println(errorMessage);
                }
            } else if (rxValueString == "LED_RED_ON") {
                digitalWrite(redLEDPin, HIGH);
                digitalWrite(greenLEDPin, LOW);
                digitalWrite(blueLEDPin, LOW);
                Serial.println("Red LED ON");
                String logMessage = "Red LED ON";
                pTxCharacteristic->setValue((uint8_t*)logMessage.c_str(), logMessage.length());
                pTxCharacteristic->notify();
            } else if (rxValueString == "LED_GREEN_ON") {
                digitalWrite(redLEDPin, LOW);
                digitalWrite(greenLEDPin, HIGH);
                digitalWrite(blueLEDPin, LOW);
                Serial.println("Green LED ON");
                String logMessage = "Green LED ON";
                pTxCharacteristic->setValue((uint8_t*)logMessage.c_str(), logMessage.length());
                pTxCharacteristic->notify();
            } else if (rxValueString == "LED_BLUE_ON") {
                digitalWrite(redLEDPin, LOW);
                digitalWrite(greenLEDPin, LOW);
                digitalWrite(blueLEDPin, HIGH);
                Serial.println("Blue LED ON");
                String logMessage = "Blue LED ON";
                pTxCharacteristic->setValue((uint8_t*)logMessage.c_str(), logMessage.length());
                pTxCharacteristic->notify();
            } else {
                String logMessage = "Received unknown command: " + rxValueString;
                pTxCharacteristic->setValue((uint8_t*)logMessage.c_str(), logMessage.length());
                pTxCharacteristic->notify();
            }
        }
    }
};

void setup() {
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
    // setup apds start (COPY CONTENT INTO YOUR setup())
    // ============================================

    // --- APDS Initialization ---
    if (!initAPD()) {
        Serial.println("APDS-9930 Initialization Failed! Halting.");
        while (1); // Stop execution if sensor fails
    } else {
        Serial.println("APDS-9930 Initialized Successfully.");
    }
    // Optional: Set initial non-default settings
    setAmbientLightGain(AGAIN_16X); // Example
    setIntegrationTimePeriods(200); // Example
    delay(120); // Allow time for settings and first integration

    // ============================================
    // setup apds end
    // ============================================

    BLEDevice::init("ESP32 Spectrophotometer");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService* pService = pServer->createService(SERVICE_UUID);
    pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_TX_UUID,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pTxCharacteristic->addDescriptor(new BLEDescriptor(BLEUUID((uint16_t)0x2902)));
    pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_RX_UUID,
                        BLECharacteristic::PROPERTY_WRITE
                      );
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->start();

    Serial.println("Waiting for a client connection to notify...");
}

void loop() {
    if (deviceConnected) {
        // You can add other tasks here
    }
    delay(10);
}
