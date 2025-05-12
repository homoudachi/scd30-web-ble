#include <Wire.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h> // For creating JSON stats string

// --- Display Settings ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- SCD30 Sensor ---
SCD30 airSensor;
bool scd30_init_success = false;

// --- BLE Settings ---
BLEServer* pServer = NULL;
bool deviceConnected = false;

// --- Service UUID ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // Keep existing

// --- Existing Data Characteristics ---
BLECharacteristic* pCO2Characteristic = NULL;
#define CO2_CHARACTERISTIC_UUID "a1b5d7c1-0304-4c17-b7f4-3f3d5f5e9f0a"
BLECharacteristic* pTempCharacteristic = NULL;
#define TEMP_CHARACTERISTIC_UUID "b2c6e8d2-1415-5d28-c8a5-4a4e6a6f0b1b"
BLECharacteristic* pHumidityCharacteristic = NULL;
#define HUMIDITY_CHARACTERISTIC_UUID "c3d7f9e3-2526-6e39-d9b6-5b5f7b7a1c2c"

// --- NEW Control/Command Characteristics (Writable) ---
BLECharacteristic* pFRCCommandCharacteristic = NULL;
#define FRC_COMMAND_CHARACTERISTIC_UUID "d4e5f6a1-b2c3-4d5e-8f9a-0b1c2d3e4f50" // Replace with YOUR generated UUID
BLECharacteristic* pResetCommandCharacteristic = NULL;
#define RESET_COMMAND_CHARACTERISTIC_UUID "e5f6a1b2-c3d4-5e8f-9a0b-1c2d3e4f5061" // Replace with YOUR generated UUID

// --- NEW Status/Data Characteristics (Notify) ---
BLECharacteristic* pUptimeCharacteristic = NULL;
#define UPTIME_CHARACTERISTIC_UUID "f6a1b2c3-d4e5-8f9a-0b1c-2d3e4f506172" // Replace with YOUR generated UUID
BLECharacteristic* pStatsCharacteristic = NULL;
#define STATS_CHARACTERISTIC_UUID "a1b2c3d4-e5f6-9a0b-1c2d-3e4f50617283" // Replace with YOUR generated UUID
// Optional Status Characteristic
BLECharacteristic* pCommandStatusCharacteristic = NULL;
#define COMMAND_STATUS_CHARACTERISTIC_UUID "b2c3d4e5-f6a1-0b1c-2d3e-4f5061728394" // Replace with YOUR generated UUID


// --- Global Sensor Data ---
float co2_ppm = 0.0;
float temperature_c = 0.0;
float humidity_percent = 0.0;
bool sensorReady = false;

// --- Statistics Variables ---
float min_co2 = 99999.0, max_co2 = 0.0, sum_co2 = 0.0;
float min_temp = 999.0, max_temp = -999.0, sum_temp = 0.0;
float min_hum = 999.0, max_hum = -1.0, sum_hum = 0.0;
unsigned long reading_count = 0;

// --- Timers ---
unsigned long lastSensorReadMillis = 0;
unsigned long lastDisplayUpdateMillis = 0;
unsigned long lastStatsUpdateMillis = 0; // Timer for updating stats/uptime BLE
const long sensorReadInterval = 2500;
const long displayUpdateInterval = 1000;
const long statsUpdateInterval = 5000; // Update stats/uptime BLE every 5 seconds

// --- Forward declarations ---
void updateStatsAndUptimeBLE();
void sendCommandStatus(const char* statusMsg);

// --- BLE Server Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Client Connected");
      // Optionally send initial stats/uptime on connect
      delay(100); // Give client time to subscribe
      updateStatsAndUptimeBLE();
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Client Disconnected");
      delay(500);
      BLEDevice::startAdvertising();
      Serial.println("Restarting advertising...");
    }
};

// --- BLE Characteristic Callbacks for Commands ---
class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        const char* uuid = pCharacteristic->getUUID().toString().c_str();

        Serial.printf("Received Write on UUID: %s, Value: %s\n", uuid, rxValue.c_str());

        // --- FRC Command Handling ---
        if (pCharacteristic == pFRCCommandCharacteristic) {
            // Attempt to convert received value to integer
            uint16_t targetPPM = (uint16_t)atoi(rxValue.c_str()); // Simple conversion
            Serial.printf("FRC Command Received. Target PPM: %d\n", targetPPM);

            // Validate PPM range (SCD30 typically 400-2000 for FRC)
            if (targetPPM >= 400 && targetPPM <= 2000) { // Adjust range if needed
                if (scd30_init_success) {
                    Serial.printf("Attempting FRC with reference value: %d ppm\n", targetPPM);
                    sendCommandStatus("FRC command processing..."); // Send immediate feedback
                    if (airSensor.forceRecalibrationWithReference(targetPPM)) {
                        Serial.println("FRC command successful!");
                        sendCommandStatus("FRC OK");
                        // Note: FRC takes time, sensor readings might be unstable briefly
                        // Maybe reset stats after FRC?
                        // min_co2 = 99999.0; max_co2 = 0.0; sum_co2 = 0.0; reading_count = 0; // etc.
                    } else {
                        Serial.println("FRC command failed!");
                        sendCommandStatus("FRC Failed (Sensor Error)");
                    }
                } else {
                     Serial.println("FRC failed: Sensor not initialized.");
                     sendCommandStatus("FRC Failed (Sensor Init)");
                }
            } else {
                Serial.println("FRC failed: Invalid PPM value.");
                sendCommandStatus("FRC Failed (Invalid PPM)");
            }
        }
        // --- Reset Command Handling ---
        else if (pCharacteristic == pResetCommandCharacteristic) {
            // Check if received value is "1" (or any specific trigger)
            if (rxValue == "1") {
                Serial.println("Reset Command Received. Restarting ESP32...");
                sendCommandStatus("Resetting ESP32...");
                delay(1000); // Give BLE time to send status
                ESP.restart();
            } else {
                 Serial.println("Invalid Reset Command value received.");
                 sendCommandStatus("Reset Failed (Invalid Cmd)");
            }
        }
    }
};

// --- Function Declarations ---
void initDisplay();
void initSCD30();
void initBLE();
void updateDisplay();
void readSensor();
void updateSensorDataBLE(); // Renamed from updateBLECharacteristics
void checkI2CDevices();

// --- Setup ---
void setup() {
  Serial.begin(115200);
  unsigned long setupStartTime = millis();
   while (!Serial && (millis() - setupStartTime < 5000)) { delay(10); }
  Serial.println("\n\n[DEBUG] ESP32 SCD30/SSD1306/WebBLE Enhanced - STARTING SETUP");

  Wire.begin();
  Serial.println("[DEBUG] I2C Initialized.");
  checkI2CDevices();

  initDisplay();
  initSCD30();
  initBLE(); // Initializes all characteristics

  lastSensorReadMillis = millis();
  lastDisplayUpdateMillis = millis();
  lastStatsUpdateMillis = millis();
  Serial.println("[DEBUG] Setup Complete.");
}

// --- Main Loop ---
void loop() {
  unsigned long currentMillis = millis();

  // --- Read Sensor Data ---
  if (scd30_init_success && (currentMillis - lastSensorReadMillis >= sensorReadInterval)) {
    readSensor(); // Updates global vars and stats vars
    lastSensorReadMillis = currentMillis;

    if (deviceConnected && sensorReady) {
        updateSensorDataBLE(); // Send current sensor data
    }
  }
  // ... (Skip sensor read if init failed logic) ...


  // --- Update Stats and Uptime BLE Periodically ---
  if (deviceConnected && (currentMillis - lastStatsUpdateMillis >= statsUpdateInterval)) {
     updateStatsAndUptimeBLE();
     lastStatsUpdateMillis = currentMillis;
  }


  // --- Update Display ---
  if (currentMillis - lastDisplayUpdateMillis >= displayUpdateInterval) {
     updateDisplay();
     lastDisplayUpdateMillis = currentMillis;
  }

   delay(10);
}

// --- Initialization Functions ---

void initDisplay() {
  Serial.println("[DEBUG] initDisplay(): Initializing Display...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("[ERROR] initDisplay(): SSD1306 allocation failed!");
    // Try displaying error
  } else {
      Serial.println("[DEBUG] initDisplay(): Display Initialized OK.");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println("Display OK");
      display.display();
      delay(500);
  }
}

void initSCD30() {
  Serial.println("[DEBUG] initSCD30(): Initializing SCD30 Sensor...");
  scd30_init_success = false;
  sensorReady = false;

  display.clearDisplay();
  display.setCursor(0,10);
  display.println("Sensor Init...");
  display.display();

  if (!airSensor.begin(Wire, true)) { // Enable library debug
    Serial.println("[ERROR] initSCD30(): airSensor.begin() failed.");
    display.clearDisplay(); display.setCursor(0,0);
    display.println("!! SCD30 ERROR !!"); display.display();
    return; // Exit init
  }

  Serial.println("[DEBUG] initSCD30(): airSensor.begin() successful.");
  display.setCursor(0,20); display.println("Sensor Found"); display.display();
  delay(100);

  uint16_t pressure_mbar = 0;
  Serial.printf("[DEBUG] initSCD30(): Starting continuous measurement...\n");
  if (!airSensor.startContinuousMeasurement(pressure_mbar)) {
    Serial.println("[ERROR] initSCD30(): Failed to start continuous measurement!");
    display.setCursor(0,30); display.println("Meas. Start FAIL"); display.display();
  } else {
     Serial.println("[DEBUG] initSCD30(): Continuous measurement started.");
     display.setCursor(0,30); display.println("Meas. Started"); display.display();
  }
  delay(100);

  uint16_t interval_sec = 0;
  if (airSensor.getMeasurementInterval(interval_sec)) {
     Serial.printf("[DEBUG] initSCD30(): Measurement interval = %d seconds.\n", interval_sec);
  } else {
     Serial.println("[WARN] initSCD30(): Failed to get measurement interval.");
  }
  delay(100);

  Serial.println("[DEBUG] initSCD30(): Waiting 5s for first reading...");
  display.setCursor(0,40); display.println("Sensor Warming..."); display.display();
  delay(5000);

  scd30_init_success = true; // Mark init as done
  Serial.println("[DEBUG] initSCD30(): Initialization steps finished.");
}


void initBLE() {
  Serial.println("[DEBUG] initBLE(): Initializing BLE...");
  display.setCursor(0,50); display.println("BLE Init..."); display.display(); // Adjust if needed

  BLEDevice::init("ESP32-SCD30-Enhanced"); // New name optional

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // --- Create Existing Data Characteristics ---
  pCO2Characteristic = pService->createCharacteristic(CO2_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCO2Characteristic->addDescriptor(new BLE2902());
  pTempCharacteristic = pService->createCharacteristic(TEMP_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTempCharacteristic->addDescriptor(new BLE2902());
  pHumidityCharacteristic = pService->createCharacteristic(HUMIDITY_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pHumidityCharacteristic->addDescriptor(new BLE2902());

  // --- Create NEW Control/Command Characteristics ---
  pFRCCommandCharacteristic = pService->createCharacteristic(FRC_COMMAND_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
  pFRCCommandCharacteristic->setCallbacks(new CommandCallbacks()); // Assign callback

  pResetCommandCharacteristic = pService->createCharacteristic(RESET_COMMAND_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
  pResetCommandCharacteristic->setCallbacks(new CommandCallbacks()); // Assign callback

  // --- Create NEW Status/Data Characteristics ---
  pUptimeCharacteristic = pService->createCharacteristic(UPTIME_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pUptimeCharacteristic->addDescriptor(new BLE2902());

  pStatsCharacteristic = pService->createCharacteristic(STATS_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pStatsCharacteristic->addDescriptor(new BLE2902());

  // Optional: Command Status Characteristic
  pCommandStatusCharacteristic = pService->createCharacteristic(COMMAND_STATUS_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCommandStatusCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[DEBUG] initBLE(): BLE Initialized & Advertising.");
  display.setCursor(0,50); display.println("BLE Advertising"); display.display();
  delay(500);
}

// --- Data Handling Functions ---

void readSensor() {
   // Serial.println("[DEBUG] readSensor(): Entered function."); // Can be noisy
   bool dataRdy = airSensor.dataAvailable();
   // Serial.printf("[DEBUG] readSensor(): airSensor.dataAvailable() returned: %s\n", dataRdy ? "true" : "false"); // Noisy

   if (dataRdy) {
    // Read values
    co2_ppm = airSensor.getCO2();
    temperature_c = airSensor.getTemperature();
    humidity_percent = airSensor.getHumidity();

    // Basic sanity check
    if (co2_ppm > 0 && co2_ppm < 10000 && temperature_c > -40 && temperature_c < 125) {
        sensorReady = true; // Mark as ready

        // --- Update Statistics ---
        reading_count++;
        sum_co2 += co2_ppm;
        sum_temp += temperature_c;
        sum_hum += humidity_percent;

        if (co2_ppm < min_co2) min_co2 = co2_ppm;
        if (co2_ppm > max_co2) max_co2 = co2_ppm;

        if (temperature_c < min_temp) min_temp = temperature_c;
        if (temperature_c > max_temp) max_temp = temperature_c;

        if (humidity_percent < min_hum) min_hum = humidity_percent;
        if (humidity_percent > max_hum) max_hum = humidity_percent;

        // Serial.printf("[DEBUG] readSensor(): Readings valid. Count: %lu\n", reading_count); // Noisy
    } else {
        sensorReady = false; // Mark as not ready if values are weird
        Serial.println("[WARN] readSensor(): Readings look suspect. sensorReady kept false.");
    }
  } else {
    // Serial.println("[DEBUG] readSensor(): No new data available from SCD30."); // Noisy
    // Keep sensorReady as it was - don't mark not ready just because data isn't available yet
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Line 0-2: Sensor Data or Status
  if (sensorReady) {
    display.print("CO2: "); display.print((int)co2_ppm); display.println(" ppm");
    display.print("Tmp: "); display.print(temperature_c, 1); display.println(" C");
    display.print("Hum: "); display.print(humidity_percent, 1); display.println(" %");
  } else if (!scd30_init_success) {
    display.println("SCD30 Init Fail");
    display.println("Check Wiring/Pwr");
    display.println(); // Blank line
  }
   else {
     display.println("Reading sensor...");
     display.println("(Waiting...)");
     display.println(); // Blank line
  }

  // Line 3: Uptime (Approximate)
  unsigned long seconds = millis() / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  seconds %= 60;
  minutes %= 60;
  char uptimeBuf[20];
  snprintf(uptimeBuf, sizeof(uptimeBuf), "Up: %lu:%02lu:%02lu", hours, minutes, seconds);
  display.setCursor(0, 30);
  display.println(uptimeBuf);


  // Line 5: BLE Status
  display.setCursor(0, 50); // Bottom line
  display.print("BLE: ");
  display.println(deviceConnected ? "Connected" : "Advertising");

  display.display();
}

// Send just the current sensor readings
void updateSensorDataBLE() {
    char buffer[10];

    snprintf(buffer, sizeof(buffer), "%.0f", co2_ppm);
    pCO2Characteristic->setValue(buffer);
    pCO2Characteristic->notify();
    delay(2); // Small delay between notifies

    snprintf(buffer, sizeof(buffer), "%.1f", temperature_c);
    pTempCharacteristic->setValue(buffer);
    pTempCharacteristic->notify();
    delay(2);

    snprintf(buffer, sizeof(buffer), "%.1f", humidity_percent);
    pHumidityCharacteristic->setValue(buffer);
    pHumidityCharacteristic->notify();
}

// Send Uptime and Stats Bundle
void updateStatsAndUptimeBLE() {
     Serial.println("[DEBUG] Updating Stats/Uptime BLE characteristics.");

     // --- Update Uptime ---
     unsigned long seconds = millis() / 1000;
     unsigned long minutes = seconds / 60;
     unsigned long hours = minutes / 60;
     seconds %= 60;
     minutes %= 60;
     char uptimeBuf[20];
     snprintf(uptimeBuf, sizeof(uptimeBuf), "%lu:%02lu:%02lu", hours, minutes, seconds);
     pUptimeCharacteristic->setValue(uptimeBuf);
     pUptimeCharacteristic->notify();
     delay(2);

     // --- Update Stats ---
     // Use ArduinoJson - Calculate averages safely
     StaticJsonDocument<256> statsDoc; // Adjust size if needed

     statsDoc["count"] = reading_count;
     if (reading_count > 0) {
        statsDoc["co2_min"] = String(min_co2, 0);
        statsDoc["co2_max"] = String(max_co2, 0);
        statsDoc["co2_avg"] = String(sum_co2 / reading_count, 0);

        statsDoc["temp_min"] = String(min_temp, 1);
        statsDoc["temp_max"] = String(max_temp, 1);
        statsDoc["temp_avg"] = String(sum_temp / reading_count, 1);

        statsDoc["hum_min"] = String(min_hum, 1);
        statsDoc["hum_max"] = String(max_hum, 1);
        statsDoc["hum_avg"] = String(sum_hum / reading_count, 1);
     } else {
         // Send default values if no readings yet
        statsDoc["co2_min"] = "---"; statsDoc["co2_max"] = "---"; statsDoc["co2_avg"] = "---";
        statsDoc["temp_min"] = "---"; statsDoc["temp_max"] = "---"; statsDoc["temp_avg"] = "---";
        statsDoc["hum_min"] = "---"; statsDoc["hum_max"] = "---"; statsDoc["hum_avg"] = "---";
     }

     String statsString;
     serializeJson(statsDoc, statsString);

     pStatsCharacteristic->setValue(statsString.c_str());
     pStatsCharacteristic->notify();

     Serial.print("[DEBUG] Sent Stats: "); Serial.println(statsString);
}

// Helper to send status messages back to web page
void sendCommandStatus(const char* statusMsg) {
    if (deviceConnected && pCommandStatusCharacteristic != NULL) {
         Serial.printf("[DEBUG] Sending Command Status: %s\n", statusMsg);
         pCommandStatusCharacteristic->setValue(statusMsg);
         pCommandStatusCharacteristic->notify();
    } else {
         Serial.printf("[DEBUG] Cannot send status '%s' (Not connected or Char NULL)\n", statusMsg);
    }
}


// --- I2C Scanner Debug Function ---
void checkI2CDevices() {
  Serial.println("[DEBUG] checkI2CDevices(): Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("[DEBUG] I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if (error==4) {
      Serial.printf("[WARN] Unknown error at address 0x%02X\n", address);
    }
  }
  if (nDevices == 0) {
    Serial.println("[ERROR] No I2C devices found!");
  } else {
    Serial.printf("[DEBUG] Found %d I2C device(s).\n", nDevices);
  }
  Serial.println("------------------------------------");
}
