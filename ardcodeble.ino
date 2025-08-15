#include <Wire.h>              // For I2C communication
#include <MPU6050.h>           // For MPU6050 sensor control
#include <BLEDevice.h>         // For BLE device definition
#include <BLEServer.h>         // For BLE server object
#include <BLEUtils.h>          // BLE utility functions
#include <BLE2902.h>           // Descriptor definition for Notify feature

MPU6050 mpu;                   // MPU6050 sensor object
bool startStreaming = false;   // Should sensor data be sent?

// BLE UUID definitions
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHAR_RX_UUID        "abcd1234-5678-90ab-cdef-1234567890ab" // computer to ESP32 (command)
#define CHAR_TX_UUID        "abcd1234-5678-90ab-cdef-1234567890ac" // ESP32 → computer

BLECharacteristic *pTxCharacteristic; // Characteristic used to send sensor data

// Class triggered when data is written by the computer
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // Using Arduino String instead of std::string:
    String cmd = String((char*)pCharacteristic->getValue().c_str());

    Serial.print("Command received: ");
    Serial.println(cmd);

    if (cmd == "start") {
      startStreaming = true;
      Serial.println("Data streaming STARTED.");
    } else if (cmd == "stop") {
      startStreaming = false;
      Serial.println("Data streaming STOPPED.");
    }
  }
};

void setup() {
  Serial.begin(115200);         // Start serial monitor
  Wire.begin();                 // Start I2C
  mpu.initialize();             // Initialize MPU6050
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 sensor is ready.");
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  // BLE configuration
  BLEDevice::init("ESP32_MPU6050");                 // Device name 
  BLEServer *pServer = BLEDevice::createServer();   // Create BLE server
  BLEService *pService = pServer->createService(SERVICE_UUID); // Create service

  // RX characteristic: receives commands from the phone
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHAR_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new MyCallbacks()); // Handle incoming commands

  // TX characteristic: sends sensor data
  pTxCharacteristic = pService->createCharacteristic(
    CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902()); // Required for Notify feature

  pService->start();                        // Start the service
  pServer->getAdvertising()->start();       // Start BLE advertising

  Serial.println("BLE started.");
}

void loop() {
  // If streaming is enabled, read from sensor and send data
  if (startStreaming) {
    int16_t ax, ay, az, gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read raw data from MPU6050

    // Format the data into a single string
    String data = "AX:" + String(ax) + " AY:" + String(ay) + " AZ:" + String(az) +
                  " GX:" + String(gx) + " GY:" + String(gy) + " GZ:" + String(gz);

    Serial.println(data); // Output to serial monitor

    pTxCharacteristic->setValue(data.c_str()); // Write data to the characteristic
    pTxCharacteristic->notify();               // Send via BLE Notify

    delay(200); // Send rate: 5Hz (200ms delay)
  }
}
