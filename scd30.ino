#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h> // Optional, for notifications if ever added
#include <Wire.h>
// Use Adafruit SCD30 Library
#include <Adafruit_SCD30.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Configuration ---
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // Or 0x3D depending on your display

// Sensor Read Interval (milliseconds)
#define SENSOR_READ_INTERVAL 5000 // How often the ESP32 checks if data is ready
#define SCD30_MEASUREMENT_INTERVAL_SECONDS 5 // SCD30 internal measurement interval

// BLE Service and Characteristic UUIDs
#define SERVICE_UUID             "e458cd1c-c71b-4f6b-9765-447dc468182c"
#define CHAR_UUID_SENSOR_DATA    "8b2f973c-0c9b-4d0a-87ae-5f29b8decbb9" // Read: CO2,Temp,Hum,UptimeS
#define CHAR_UUID_ASC_STATUS     "fe921c63-6745-43f5-b070-c9046a30ea10" // Read/Write: 1=on, 0=off
#define CHAR_UUID_FRC_COMMAND    "29caaaab-2588-4bdc-8e41-e364a9dbb202" // Write: FRC value (400-2000) as string
#define CHAR_UUID_STATS          "b47fc5f5-b357-4883-86b0-fcd107f9a475" // Read: Min/Max/Avg CO2,Temp,Hum
#define CHAR_UUID_DEVICE_COMMAND "3194473d-1536-4143-8116-9e805f341420" // Write: "RESET_STATS" or "RESET_DEVICE"

// --- Global Variables ---
Adafruit_SCD30 scd30;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

BLEServer* pServer = NULL;
BLECharacteristic* pSensorDataCharacteristic = NULL;
BLECharacteristic* pAscStatusCharacteristic = NULL;
BLECharacteristic* pFrcCommandCharacteristic = NULL;
BLECharacteristic* pStatsCharacteristic = NULL;
BLECharacteristic* pDeviceCommandCharacteristic = NULL; // New
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Sensor values (will be updated from scd30 object member variables)
float co2 = 0.0;
float temp = 0.0;
float humidity = 0.0;
bool ascEnabled = false;

// Timing
unsigned long startTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long uptimeSeconds = 0;

// Statistics
float minCO2 = 9999.0, maxCO2 = 0.0, sumCO2 = 0.0;
float minTemp = 200.0, maxTemp = -100.0, sumTemp = 0.0;
float minHumidity = 200.0, maxHumidity = -100.0, sumHumidity = 0.0;
unsigned long readCount = 0;

// --- Forward Declarations ---
void updateDisplay();
void resetStatistics(); // New

// --- BLE Server Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device Connected");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println("BLE Connected");
      display.display();
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      oldDeviceConnected = false; // Ensure this is reset too
      Serial.println("Device Disconnected");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println("BLE Disconnected");
      display.println("Advertising...");
      display.display();
      // Restart advertising after a short delay
      delay(500);
      BLEDevice::startAdvertising();
      Serial.println("Restarted Advertising");
    }
};

// --- BLE Characteristic Callbacks ---
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {

    // Handle Read requests
    void onRead(BLECharacteristic* pCharacteristic) {
        std::string uuid_str = pCharacteristic->getUUID().toString().c_str();
        // Serial.print("Read request for UUID: "); Serial.println(uuid_str.c_str()); // Verbose

        if (pCharacteristic == pSensorDataCharacteristic) {
            char dataStr[100];
            snprintf(dataStr, sizeof(dataStr), "%.1f,%.2f,%.2f,%lu", co2, temp, humidity, uptimeSeconds);
            pCharacteristic->setValue(dataStr);
            // Serial.print("Sent Sensor Data: "); Serial.println(dataStr); // Verbose
        }
        else if (pCharacteristic == pAscStatusCharacteristic) {
            if (scd30.selfCalibrationEnabled()) {
                ascEnabled = true;
                pCharacteristic->setValue("1");
                // Serial.println("Sent ASC Status: 1 (On)"); // Verbose
            } else {
                ascEnabled = false;
                pCharacteristic->setValue("0");
                // Serial.println("Sent ASC Status: 0 (Off)"); // Verbose
            }
        }
         else if (pCharacteristic == pStatsCharacteristic) {
            char statsStr[200];
            float avgCO2 = (readCount > 0) ? (sumCO2 / readCount) : 0.0;
            float avgTemp = (readCount > 0) ? (sumTemp / readCount) : 0.0;
            float avgHumidity = (readCount > 0) ? (sumHumidity / readCount) : 0.0;

            float displayMinCO2 = (readCount > 0) ? minCO2 : 0.0;
            float displayMaxCO2 = (readCount > 0) ? maxCO2 : 0.0;
            float displayMinTemp = (readCount > 0) ? minTemp : 0.0; // Show 0 if no reads yet
            float displayMaxTemp = (readCount > 0) ? maxTemp : 0.0;
            float displayMinHum = (readCount > 0) ? minHumidity : 0.0;
            float displayMaxHum = (readCount > 0) ? maxHumidity : 0.0;

            snprintf(statsStr, sizeof(statsStr),
                     "%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                     displayMinCO2, displayMaxCO2, avgCO2,
                     displayMinTemp, displayMaxTemp, avgTemp,
                     displayMinHum, displayMaxHum, avgHumidity);
            pCharacteristic->setValue(statsStr);
            // Serial.print("Sent Stats: "); Serial.println(statsStr); // Verbose
        }
    }

    // Handle Write requests
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string uuid_str = pCharacteristic->getUUID().toString().c_str();
        std::string value_str = pCharacteristic->getValue().c_str();

        Serial.print("Write request for UUID: ");
        Serial.print(uuid_str.c_str());
        Serial.print(" | Value: ");
        Serial.println(value_str.c_str());

        if (pCharacteristic == pAscStatusCharacteristic) {
            if (value_str == "1") {
                if (scd30.selfCalibrationEnabled(true)) {
                    ascEnabled = true;
                    Serial.println("ASC Enabled via BLE");
                    updateDisplay();
                } else { Serial.println("Error enabling ASC"); }
            } else if (value_str == "0") {
                 if (scd30.selfCalibrationEnabled(false)) {
                    ascEnabled = false;
                    Serial.println("ASC Disabled via BLE");
                    updateDisplay();
                 } else { Serial.println("Error disabling ASC"); }
            } else { Serial.println("Invalid ASC value received"); }
        }
        else if (pCharacteristic == pFrcCommandCharacteristic) {
            int frcValue = atoi(value_str.c_str());
            if (frcValue >= 400 && frcValue <= 2000) {
                Serial.print("Performing FRC with value: "); Serial.println(frcValue);
                display.clearDisplay();
                display.setCursor(0, 10);
                display.println("FRC in progress...");
                display.print(frcValue); display.println(" ppm");
                display.display();

                if (scd30.forceRecalibrationWithReference(frcValue)) {
                    Serial.println("FRC command successful.");
                    delay(500);
                    lastSensorReadTime = 0; // Force immediate read
                } else { Serial.println("FRC command failed."); }
                updateDisplay();
            } else { Serial.println("Invalid FRC value received (must be 400-2000)"); }
        }
        else if (pCharacteristic == pDeviceCommandCharacteristic) { // New
            if (value_str == "RESET_STATS") {
                resetStatistics();
            } else if (value_str == "RESET_DEVICE") {
                Serial.println("Received RESET_DEVICE command. Restarting ESP32...");
                delay(500); // Give BLE stack a moment
                ESP.restart();
                // Execution stops here
            } else { Serial.println("Unknown device command received."); }
        }
    }
};


// --- Helper Functions ---

void resetStatistics() {
    Serial.println("Resetting statistics...");
    minCO2 = 9999.0; maxCO2 = 0.0; sumCO2 = 0.0;
    minTemp = 200.0; maxTemp = -100.0; sumTemp = 0.0;
    minHumidity = 200.0; maxHumidity = -100.0; sumHumidity = 0.0;
    readCount = 0;

    // Immediately update the stats characteristic if connected
    if (pStatsCharacteristic != NULL && deviceConnected) {
         char statsStr[200];
         snprintf(statsStr, sizeof(statsStr),
                 "0.0,0.0,0.0,0.00,0.00,0.00,0.00,0.00,0.00"); // Reset values
         pStatsCharacteristic->setValue(statsStr);
         // Optional: notify if characteristic supports it
         // pStatsCharacteristic->notify();
         Serial.println("Updated stats characteristic after reset.");
    }
}

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    if (!deviceConnected && oldDeviceConnected) {
         display.println("BLE Disconnected");
         display.println("Advertising...");
    } else if (deviceConnected && !oldDeviceConnected) {
         display.println("BLE Connected");
    } else if (deviceConnected) {
         display.println("BLE Connected");
    } else {
        display.println("BLE Advertising");
    }

    display.setCursor(0, 10);
    display.print("CO2: "); display.print(co2, 1); display.println(" ppm");
    display.setCursor(0, 20);
    display.print("Tmp: "); display.print(temp, 2); display.println(" C");
    display.setCursor(0, 30);
    display.print("Hum: "); display.print(humidity, 2); display.println(" %");
    display.setCursor(0, 40);
    display.print("ASC: "); display.println(ascEnabled ? "ON" : "OFF");
    display.setCursor(0, 50);
    display.print("Up: ");
    unsigned long secs = uptimeSeconds % 60;
    unsigned long mins = (uptimeSeconds / 60) % 60;
    unsigned long hours = (uptimeSeconds / 3600);
    char uptimeStr[20];
    snprintf(uptimeStr, sizeof(uptimeStr), "%lu:%02lu:%02lu", hours, mins, secs);
    display.println(uptimeStr);

    display.display();
}

void updateStats() {
    // Use Adafruit library's typical valid ranges and member variables
    if (scd30.CO2 > 0 && scd30.temperature > -40 && scd30.relative_humidity >= 0 && scd30.relative_humidity <= 100) {
        if (readCount == 0) { // First valid reading initializes min/max
            minCO2 = maxCO2 = scd30.CO2;
            minTemp = maxTemp = scd30.temperature;
            minHumidity = maxHumidity = scd30.relative_humidity;
        } else {
            if (scd30.CO2 < minCO2) minCO2 = scd30.CO2;
            if (scd30.CO2 > maxCO2) maxCO2 = scd30.CO2;
            if (scd30.temperature < minTemp) minTemp = scd30.temperature;
            if (scd30.temperature > maxTemp) maxTemp = scd30.temperature;
            if (scd30.relative_humidity < minHumidity) minHumidity = scd30.relative_humidity;
            if (scd30.relative_humidity > maxHumidity) maxHumidity = scd30.relative_humidity;
        }
        sumCO2 += scd30.CO2;
        sumTemp += scd30.temperature;
        sumHumidity += scd30.relative_humidity;
        readCount++;
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000); // Wait for Serial Monitor briefly
    Serial.println("\nStarting ESP32 SCD30 Web BLE Sensor (Adafruit Lib)...");

    startTime = millis();
    Wire.begin(); // Default SDA=21, SCL=22

    // Initialize OLED Display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Initializing...");
    display.display();
    delay(500); // Shorter delay

    // Initialize SCD30 Sensor
    if (!scd30.begin()) {
        Serial.println("Failed to find SCD30 chip");
        display.clearDisplay(); display.setCursor(0,0);
        display.println("SCD30 Error!"); display.println("Check Wiring.");
        display.display();
        while (1) { delay(10); }
    }
    Serial.println("SCD30 Found!");

    // Set Measurement Interval
    if (!scd30.setMeasurementInterval(SCD30_MEASUREMENT_INTERVAL_SECONDS)) {
        Serial.print("Failed to set measurement interval to "); Serial.println(SCD30_MEASUREMENT_INTERVAL_SECONDS);
    } else {
        Serial.print("Measurement interval set to "); Serial.print(SCD30_MEASUREMENT_INTERVAL_SECONDS); Serial.println(" seconds.");
    }

    // Read Initial ASC Status
    ascEnabled = scd30.selfCalibrationEnabled();
    Serial.print("Initial ASC Status: "); Serial.println(ascEnabled ? "ON" : "OFF");


    // --- Initialize BLE ---
    Serial.println("Initializing BLE...");
    BLEDevice::init("ESP32_SCD30_Sensor");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Sensor Data Characteristic (Read)
    pSensorDataCharacteristic = pService->createCharacteristic(
                                CHAR_UUID_SENSOR_DATA,
                                BLECharacteristic::PROPERTY_READ);
    pSensorDataCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    // ASC Status Characteristic (Read/Write)
    pAscStatusCharacteristic = pService->createCharacteristic(
                                CHAR_UUID_ASC_STATUS,
                                BLECharacteristic::PROPERTY_READ |
                                BLECharacteristic::PROPERTY_WRITE);
    pAscStatusCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pAscStatusCharacteristic->setValue(ascEnabled ? "1" : "0");

    // FRC Command Characteristic (Write)
    pFrcCommandCharacteristic = pService->createCharacteristic(
                                CHAR_UUID_FRC_COMMAND,
                                BLECharacteristic::PROPERTY_WRITE);
    pFrcCommandCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

     // Statistics Characteristic (Read)
     pStatsCharacteristic = pService->createCharacteristic(
                                 CHAR_UUID_STATS,
                                 BLECharacteristic::PROPERTY_READ);
     pStatsCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
     pStatsCharacteristic->setValue("0.0,0.0,0.0,0.00,0.00,0.00,0.00,0.00,0.00");

    // Device Command Characteristic (Write) - New
    pDeviceCommandCharacteristic = pService->createCharacteristic(
                                 CHAR_UUID_DEVICE_COMMAND,
                                 BLECharacteristic::PROPERTY_WRITE);
    pDeviceCommandCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("BLE Service Started & Advertising!");
    Serial.println("Waiting for client connection...");

    updateDisplay(); // Show initial screen
}

// --- Loop ---
void loop() {
    unsigned long currentTime = millis();
    uptimeSeconds = (currentTime - startTime) / 1000;

    // --- Sensor Reading ---
    if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
        if (scd30.dataReady()) {
            lastSensorReadTime = currentTime;
            // Serial.print("Data Available! Reading SCD30..."); // Verbose

            if (!scd30.read()) {
                Serial.println("Error reading sensor data");
                // Keep previous values or indicate error? Let's keep previous for now.
                // co2 = -1.0; temp = -99.0; humidity = -1.0;
            } else {
                co2 = scd30.CO2;
                temp = scd30.temperature;
                humidity = scd30.relative_humidity;
                // Serial.printf(" CO2: %.1f ppm, Temp: %.2f C, Hum: %.2f %% \n", co2, temp, humidity); // Verbose
                updateStats();
            }
            updateDisplay();
        }
    }

     // --- Handle BLE Connection Status Change ---
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = true; // Set this flag *after* handling connection
        updateDisplay();
        Serial.println("Handling new connection...");
        // Refresh ASC status for client on connect
        ascEnabled = scd30.selfCalibrationEnabled();
        pAscStatusCharacteristic->setValue(ascEnabled ? "1" : "0");
        Serial.print("Refreshed ASC status for client: "); Serial.println(ascEnabled ? "ON" : "OFF");
    }
    // Disconnect logic moved mostly to onDisconnect callback

    // Small delay to yield to other tasks if needed
    delay(10);
}
