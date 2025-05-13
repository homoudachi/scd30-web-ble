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
// Include Esp.h to ensure we use the correct definitions (usually included by Arduino.h for ESP32)
#include <Esp.h>

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
#define CHAR_UUID_SYS_STATS      "6ac41580-04f6-4d66-9c73-723a10440fe3" // Read: FreeHeap,TotalHeap,MinHeap,SketchUse,SketchTotal,CPUIdle%


// --- Global Variables ---
Adafruit_SCD30 scd30;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

BLEServer* pServer = NULL;
BLECharacteristic* pSensorDataCharacteristic = NULL;
BLECharacteristic* pAscStatusCharacteristic = NULL;
BLECharacteristic* pFrcCommandCharacteristic = NULL;
BLECharacteristic* pStatsCharacteristic = NULL;
BLECharacteristic* pDeviceCommandCharacteristic = NULL;
BLECharacteristic* pSysStatsCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Sensor values (updated from scd30 object)
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

// For CPU Idle Approximation
#define CPU_SAMPLE_PERIOD_MS 10000 // Sample CPU approx every 10 seconds
unsigned long lastCpuSampleTime = 0;
unsigned long loopIdleTime = 0; // Accumulated "idle" time in main loop during sample period
float cpuIdlePercent = 100.0; // Start at 100% idle

// --- Forward Declarations ---
void updateDisplay();
void resetStatistics();

// --- BLE Server Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true; // Set connected flag
      Serial.println("Device Connected");
      // Update display immediately on connect
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println("BLE Connected");
      display.display();
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false; // Clear connected flag
      oldDeviceConnected = false; // Reset this flag too
      Serial.println("Device Disconnected");
      // Update display immediately on disconnect
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println("BLE Disconnected");
      display.println("Advertising...");
      display.display();
      // Restart advertising after a short delay to allow stack cleanup
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
        // Serial.print("Read req: "); Serial.println(uuid_str.c_str()); // Verbose for debugging

        if (pCharacteristic == pSensorDataCharacteristic) {
            char dataStr[100];
            snprintf(dataStr, sizeof(dataStr), "%.1f,%.2f,%.2f,%lu", co2, temp, humidity, uptimeSeconds);
            pCharacteristic->setValue(dataStr);
        }
        else if (pCharacteristic == pAscStatusCharacteristic) {
             // Read current status directly from sensor for accuracy
            bool currentAscStatus = scd30.selfCalibrationEnabled();
            ascEnabled = currentAscStatus; // Update global variable to match
            pCharacteristic->setValue(currentAscStatus ? "1" : "0");
        }
         else if (pCharacteristic == pStatsCharacteristic) {
            char statsStr[200];
            // Calculate averages safely
            float avgCO2 = (readCount > 0) ? (sumCO2 / readCount) : 0.0;
            float avgTemp = (readCount > 0) ? (sumTemp / readCount) : 0.0;
            float avgHumidity = (readCount > 0) ? (sumHumidity / readCount) : 0.0;
            // Use 0 for min/max if no readings yet, otherwise use stored values
            float displayMinCO2 = (readCount > 0) ? minCO2 : 0.0;
            float displayMaxCO2 = (readCount > 0) ? maxCO2 : 0.0;
            float displayMinTemp = (readCount > 0) ? minTemp : 0.0;
            float displayMaxTemp = (readCount > 0) ? maxTemp : 0.0;
            float displayMinHum = (readCount > 0) ? minHumidity : 0.0;
            float displayMaxHum = (readCount > 0) ? maxHumidity : 0.0;
            // Format the string
            snprintf(statsStr, sizeof(statsStr), "%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                     displayMinCO2, displayMaxCO2, avgCO2, displayMinTemp, displayMaxTemp, avgTemp,
                     displayMinHum, displayMaxHum, avgHumidity);
            pCharacteristic->setValue(statsStr);
         }
         else if (pCharacteristic == pSysStatsCharacteristic) {
             char sysStatsStr[150];
             uint32_t freeHeap = ESP.getFreeHeap();
             uint32_t totalHeap = ESP.getHeapSize();
             uint32_t minFreeHeap = ESP.getMinFreeHeap(); // Lowest level since boot
             uint32_t sketchUsed = ESP.getSketchSize();
             // Correct way to get total partition size using Esp.h functions
             uint32_t sketchTotal = ESP.getSketchSize() + ESP.getFreeSketchSpace();

             // Format the system stats string
             snprintf(sysStatsStr, sizeof(sysStatsStr), "%lu,%lu,%lu,%lu,%lu,%.1f",
                      freeHeap, totalHeap, minFreeHeap,
                      sketchUsed, sketchTotal, cpuIdlePercent); // cpuIdlePercent updated in loop
             pCharacteristic->setValue(sysStatsStr);
         }
    }

    // Handle Write requests
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string uuid_str = pCharacteristic->getUUID().toString().c_str();
        std::string value_str = pCharacteristic->getValue().c_str(); // Convert Arduino String

        Serial.print("Write request for UUID: ");
        Serial.print(uuid_str.c_str());
        Serial.print(" | Value: ");
        Serial.println(value_str.c_str());

        if (pCharacteristic == pAscStatusCharacteristic) {
            bool enableASC = (value_str == "1"); // Determine desired state
            if (scd30.selfCalibrationEnabled(enableASC)) { // Attempt to set on sensor
                ascEnabled = enableASC; // Update global state only on success
                Serial.print("ASC "); Serial.println(enableASC ? "Enabled via BLE" : "Disabled via BLE");
                updateDisplay(); // Update OLED
            } else {
                // Log error if setting failed
                Serial.print("Error "); Serial.println(enableASC ? "enabling ASC" : "disabling ASC");
                // Optionally: read back the actual state and update the characteristic if needed
            }
        }
        else if (pCharacteristic == pFrcCommandCharacteristic) {
            int frcValue = atoi(value_str.c_str()); // Convert string value to integer
            // Validate the range
            if (frcValue >= 400 && frcValue <= 2000) {
                Serial.print("Performing FRC with value: "); Serial.println(frcValue);
                // Update display to show progress
                display.clearDisplay(); display.setCursor(0, 10);
                display.println("FRC in progress..."); display.print(frcValue); display.println(" ppm");
                display.display();

                // Send command to sensor
                if (scd30.forceRecalibrationWithReference(frcValue)) {
                    Serial.println("FRC command successful.");
                    delay(500); // Give sensor time to process and store the value
                    lastSensorReadTime = 0; // Force immediate sensor read on next loop iteration
                } else {
                    Serial.println("FRC command failed.");
                }
                updateDisplay(); // Refresh display after FRC attempt
            } else {
                Serial.println("Invalid FRC value received (must be 400-2000)");
            }
        }
        else if (pCharacteristic == pDeviceCommandCharacteristic) {
            if (value_str == "RESET_STATS") {
                resetStatistics(); // Call the helper function
            } else if (value_str == "RESET_DEVICE") {
                Serial.println("Received RESET_DEVICE command. Restarting ESP32...");
                delay(500); // Short delay to allow BLE stack to potentially acknowledge write
                ESP.restart(); // Restart the ESP32
                // Execution effectively stops here
            } else {
                 Serial.println("Unknown device command received.");
            }
        }
    }
};


// --- Helper Functions ---

// Reset runtime statistics variables
void resetStatistics() {
    Serial.println("Resetting statistics...");
    // Reset runtime variables to initial state
    minCO2 = 9999.0; maxCO2 = 0.0; sumCO2 = 0.0;
    minTemp = 200.0; maxTemp = -100.0; sumTemp = 0.0;
    minHumidity = 200.0; maxHumidity = -100.0; sumHumidity = 0.0;
    readCount = 0;

    // Immediately update the stats characteristic to reflect the reset if connected
    if (pStatsCharacteristic != NULL && deviceConnected) {
         char statsStr[200];
         // Format string with reset values (0.0 or 0.00)
         snprintf(statsStr, sizeof(statsStr),
                 "0.0,0.0,0.0,0.00,0.00,0.00,0.00,0.00,0.00");
         pStatsCharacteristic->setValue(statsStr);
         Serial.println("Updated stats characteristic after reset.");
         // Optional: notify if characteristic supports it
         // if (pStatsCharacteristic->getProperties() & BLECharacteristic::PROPERTY_NOTIFY) {
         //     pStatsCharacteristic->notify();
         // }
    }
}

// Update the OLED display content
void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    // Show connection status based on current flags
    if (deviceConnected) {
         display.println("BLE Connected");
    } else {
        // Show advertising only if not previously connected or after disconnect message displayed once
         if (!oldDeviceConnected) { // Check if it was *previously* connected
            display.println("BLE Advertising");
         } else {
            // If it was previously connected, briefly show disconnected
            display.println("BLE Disconnected");
            // Note: oldDeviceConnected flag is reset in onDisconnect callback
         }
    }

    // Display sensor readings
    display.setCursor(0, 10); display.print("CO2: "); display.print(co2, 1); display.println(" ppm");
    display.setCursor(0, 20); display.print("Tmp: "); display.print(temp, 2); display.println(" C");
    display.setCursor(0, 30); display.print("Hum: "); display.print(humidity, 2); display.println(" %");
    display.setCursor(0, 40); display.print("ASC: "); display.println(ascEnabled ? "ON" : "OFF"); // Show ASC status

    // Display uptime
    display.setCursor(0, 50); display.print("Up: ");
    unsigned long secs = uptimeSeconds % 60;
    unsigned long mins = (uptimeSeconds / 60) % 60;
    unsigned long hours = (uptimeSeconds / 3600);
    char uptimeStr[20];
    snprintf(uptimeStr, sizeof(uptimeStr), "%lu:%02lu:%02lu", hours, mins, secs);
    display.println(uptimeStr);

    display.display(); // Push buffer to OLED
}

// Update running statistics with a new valid sensor reading
void updateStats() {
    // Check if the reading from the sensor object is valid before updating stats
    if (scd30.CO2 > 0 && scd30.temperature > -40 && scd30.relative_humidity >= 0 && scd30.relative_humidity <= 100) {
        if (readCount == 0) { // First valid reading initializes min/max
            minCO2 = maxCO2 = scd30.CO2;
            minTemp = maxTemp = scd30.temperature;
            minHumidity = maxHumidity = scd30.relative_humidity;
        } else {
            // Update min/max values
            if (scd30.CO2 < minCO2) minCO2 = scd30.CO2;
            if (scd30.CO2 > maxCO2) maxCO2 = scd30.CO2;
            if (scd30.temperature < minTemp) minTemp = scd30.temperature;
            if (scd30.temperature > maxTemp) maxTemp = scd30.temperature;
            if (scd30.relative_humidity < minHumidity) minHumidity = scd30.relative_humidity;
            if (scd30.relative_humidity > maxHumidity) maxHumidity = scd30.relative_humidity;
        }
        // Accumulate sums for average calculation
        sumCO2 += scd30.CO2;
        sumTemp += scd30.temperature;
        sumHumidity += scd30.relative_humidity;
        readCount++; // Increment the count of valid readings
    } else {
         // Optional: Log invalid reading if needed for debugging
         // Serial.println("Skipping stats update due to invalid sensor reading.");
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    // Wait briefly for serial monitor connection, but don't hang indefinitely
    while (!Serial && millis() < 2000);
    Serial.println("\nStarting ESP32 SCD30 Web BLE Sensor (Adafruit Lib)...");

    startTime = millis(); // Record boot time
    lastCpuSampleTime = startTime; // Initialize CPU sample timer
    Wire.begin(); // Initialize I2C (Default pins: SDA=21, SCL=22 for most ESP32 boards)

    // Initialize OLED Display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Halt execution if display fails
    }
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0); display.println("Initializing..."); display.display();
    delay(500); // Short delay for display init

    // Initialize SCD30 Sensor
    if (!scd30.begin()) {
        Serial.println("Failed to find SCD30 chip");
        display.clearDisplay(); display.setCursor(0,0);
        display.println("SCD30 Error!"); display.println("Check Wiring."); display.display();
        while (1) { delay(10); } // Halt execution if sensor fails
    }
    Serial.println("SCD30 Found!");

    // Set SCD30 Measurement Interval
    if (!scd30.setMeasurementInterval(SCD30_MEASUREMENT_INTERVAL_SECONDS)) {
        Serial.print("Failed to set measurement interval to "); Serial.println(SCD30_MEASUREMENT_INTERVAL_SECONDS);
    } else {
        Serial.print("Measurement interval set to "); Serial.print(SCD30_MEASUREMENT_INTERVAL_SECONDS); Serial.println(" seconds.");
    }

    // Read Initial ASC Status from sensor
    ascEnabled = scd30.selfCalibrationEnabled();
    Serial.print("Initial ASC Status: "); Serial.println(ascEnabled ? "ON" : "OFF");


    // --- Initialize BLE ---
    Serial.println("Initializing BLE...");
    BLEDevice::init("ESP32_SCD30_Sensor"); // Set BLE device name
    pServer = BLEDevice::createServer(); // Create the BLE Server
    pServer->setCallbacks(new MyServerCallbacks()); // Set server callbacks for connection/disconnection

    // Create the BLE Service using defined UUID
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create Sensor Data Characteristic (Read-Only)
    pSensorDataCharacteristic = pService->createCharacteristic(
                                CHAR_UUID_SENSOR_DATA,
                                BLECharacteristic::PROPERTY_READ
                              );
    pSensorDataCharacteristic->setCallbacks(new MyCharacteristicCallbacks()); // Set read callback

    // Create ASC Status Characteristic (Read/Write)
    pAscStatusCharacteristic = pService->createCharacteristic(
                                CHAR_UUID_ASC_STATUS,
                                BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
                              );
    pAscStatusCharacteristic->setCallbacks(new MyCharacteristicCallbacks()); // Set read/write callbacks
    pAscStatusCharacteristic->setValue(ascEnabled ? "1" : "0"); // Set initial value based on sensor state

    // Create FRC Command Characteristic (Write-Only)
    pFrcCommandCharacteristic = pService->createCharacteristic(
                                CHAR_UUID_FRC_COMMAND,
                                BLECharacteristic::PROPERTY_WRITE
                              );
    pFrcCommandCharacteristic->setCallbacks(new MyCharacteristicCallbacks()); // Set write callback

     // Create Statistics Characteristic (Read-Only)
     pStatsCharacteristic = pService->createCharacteristic(
                                 CHAR_UUID_STATS,
                                 BLECharacteristic::PROPERTY_READ
                               );
     pStatsCharacteristic->setCallbacks(new MyCharacteristicCallbacks()); // Set read callback
     pStatsCharacteristic->setValue("0.0,0.0,0.0,0.00,0.00,0.00,0.00,0.00,0.00"); // Initial empty stats string

    // Create Device Command Characteristic (Write-Only)
    pDeviceCommandCharacteristic = pService->createCharacteristic(
                                 CHAR_UUID_DEVICE_COMMAND,
                                 BLECharacteristic::PROPERTY_WRITE
                               );
    pDeviceCommandCharacteristic->setCallbacks(new MyCharacteristicCallbacks()); // Set write callback

    // Create System Stats Characteristic (Read-Only)
    pSysStatsCharacteristic = pService->createCharacteristic(
                                 CHAR_UUID_SYS_STATS,
                                 BLECharacteristic::PROPERTY_READ
                               );
    pSysStatsCharacteristic->setCallbacks(new MyCharacteristicCallbacks()); // Set read callback
    // Initial value will be fetched on the first client read request

    // Start the BLE service
    pService->start();

    // Configure and Start Advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID); // Advertise our main service UUID
    pAdvertising->setScanResponse(true); // Allow response data in scan results
    // Set connection interval preferences (helps with iOS compatibility)
    pAdvertising->setMinPreferred(0x06); // Min interval: 0x06 * 1.25ms = 7.5ms
    pAdvertising->setMinPreferred(0x12); // Max interval: 0x12 * 1.25ms = 22.5ms
    BLEDevice::startAdvertising(); // Start advertising the device
    Serial.println("BLE Service Started & Advertising!");
    Serial.println("Waiting for client connection...");

    updateDisplay(); // Show initial screen state on OLED
}

// --- Loop ---
void loop() {
    unsigned long loopStartTime = millis(); // Record loop start time for timing/calculations
    unsigned long timeSinceLastCpuSample = loopStartTime - lastCpuSampleTime;

    // Update uptime counter
    uptimeSeconds = (loopStartTime - startTime) / 1000;

    // --- Sensor Reading Logic ---
    // Check if enough time has passed since the last *attempted* read
    if (loopStartTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
        // Check if the sensor *actually* has new data available
        if (scd30.dataReady()) {
            lastSensorReadTime = loopStartTime; // Update last read time *after* confirmation

            // Attempt to read sensor data
            if (!scd30.read()) {
                Serial.println("Error reading sensor data");
                // Decide how to handle read errors - keep old values, set to error values, etc.
                // Keeping old values for now to avoid sudden drops on charts/display.
            } else {
                // Successfully read data, update global variables from sensor object members
                co2 = scd30.CO2;
                temp = scd30.temperature;
                humidity = scd30.relative_humidity;
                updateStats(); // Update statistics with the new valid reading
            }
            // Update the display regardless of read success to show latest values or persisted old ones
            updateDisplay();
        }
        // else: Sensor wasn't ready yet, will check again next interval
    }

     // --- Handle BLE Connection State Changes (for display and logic) ---
    if (deviceConnected && !oldDeviceConnected) {
        // Device just connected in this loop iteration
        oldDeviceConnected = true; // Set flag *after* handling the transition
        updateDisplay(); // Update display immediately to show "Connected"
        Serial.println("Handling new connection actions...");

        // Refresh ASC status characteristic for the newly connected client
        bool currentAsc = scd30.selfCalibrationEnabled(); // Read current state
        ascEnabled = currentAsc; // Update global var
        if (pAscStatusCharacteristic != NULL) { // Check if characteristic pointer is valid
             pAscStatusCharacteristic->setValue(currentAsc ? "1" : "0"); // Set characteristic value
        }
        Serial.print("Refreshed ASC status for client: "); Serial.println(currentAsc ? "ON" : "OFF");

    } else if (!deviceConnected && oldDeviceConnected) {
        // Device just disconnected in this loop iteration
        // Note: Most disconnect logic (like restarting advertising) is in the onDisconnect callback
        oldDeviceConnected = false; // Reset flag *after* handling transition
        updateDisplay(); // Update display immediately to show "Disconnected"
        Serial.println("Handling disconnection actions in loop (display updated)...");
    }


    // --- CPU Idle Approximation Calculation ---
    // Accumulate known idle time (the delay at the end of the loop)
    unsigned long currentLoopKnownIdle = 10; // Milliseconds spent in the delay() below
    loopIdleTime += currentLoopKnownIdle;

    // Check if the sample period has elapsed
    if (timeSinceLastCpuSample >= CPU_SAMPLE_PERIOD_MS) {
        if (timeSinceLastCpuSample > 0) { // Prevent division by zero
             // Calculate percentage of time spent in known delays over the sample period
             cpuIdlePercent = ( (float)loopIdleTime / timeSinceLastCpuSample) * 100.0;
             // Cap the value at 100% in case of timing inaccuracies or calculation overshoot
             if (cpuIdlePercent > 100.0) cpuIdlePercent = 100.0;
             // Optional: Print the calculated value for debugging
             // Serial.printf("CPU Idle Approx: %.1f%% (Idle: %lu ms, Period: %lu ms)\n",
             //               cpuIdlePercent, loopIdleTime, timeSinceLastCpuSample);
        }
        // Reset accumulator and timestamp for the next sample period
        loopIdleTime = 0;
        lastCpuSampleTime = loopStartTime; // Reset sample timer to the start of this loop
    }

    // Small delay to prevent watchdog timeouts and allow other tasks (like BLE stack) to run
    delay(10);
}
