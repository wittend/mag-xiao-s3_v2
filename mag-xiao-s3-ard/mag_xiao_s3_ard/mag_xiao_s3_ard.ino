#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>
#include "rm3100.h"
#include "MCP9808.h"

// ========================================
// Pin Definitions for XIAO ESP32S3
// ========================================
// I2C pins (default Wire)
#define I2C_SDA 5  // GPIO5 (D4)
#define I2C_SCL 6  // GPIO6 (D5)

// GNSS UART pins
#define GNSS_RX 44  // GPIO44 (D7)
#define GNSS_TX 43  // GPIO43 (D6)

// GNSS PPS pin for interrupt
#define GNSS_PPS_PIN 2  // GPIO2 (D1)

// GNSS update interval (hourly check per REQ-004)
#define GNSS_UPDATE_INTERVAL 3600000  // 1 hour in milliseconds

// LoRa SX1262 pins (SPI)
#define LORA_NSS 3      // GPIO3 (D2) - Chip select
#define LORA_DIO1 4     // GPIO4 (D3) - DIO1 interrupt
#define LORA_NRST 1     // GPIO1 (D0) - Reset
#define LORA_BUSY 7     // GPIO7 (D8) - Busy
// SPI pins: MOSI=GPIO9 (D9), MISO=GPIO8 (D10), SCK=GPIO10 (D11) - default SPI

// Configuration button pin
#define CONFIG_BUTTON_PIN 42  // GPIO42 - Press to enter config mode

// ========================================
// Global Variables
// ========================================
volatile bool ppsTriggered = false;
unsigned long lastSampleTime = 0;
unsigned long lastGNSSUpdate = 0;

// Magnetometer raw readings
int32_t magX = 0, magY = 0, magZ = 0;
float magX_uT = 0.0, magY_uT = 0.0, magZ_uT = 0.0;

// Temperature reading
float temperature = 0.0;

// Gain value for RM3100 conversion
uint16_t currentGain = GAIN_75;  // Default gain

// GNSS time data
struct {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    bool valid;
} gnssTime = {2025, 1, 1, 0, 0, 0, false};

// UART for GNSS
HardwareSerial GNSSSerial(1);  // Use UART1

// LoRa radio instance
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

// LoRa configuration
struct {
    float frequency;      // MHz
    float bandwidth;      // kHz
    uint8_t spreadFactor; // 7-12
    uint8_t codingRate;   // 5-8
    int8_t txPower;       // dBm
    uint16_t preamble;    // symbols
} loraConfig = {915.0, 125.0, 9, 7, 17, 8};  // Default US915 settings

// Device configuration
struct DeviceConfig {
    char nodeId[32];
    char owner[64];
    float latitude;
    float longitude;
    float elevation;
    int magTranslateX;
    int magTranslateY;
    int magTranslateZ;
    bool usePps;        // whether to use external GNSS PPS for 1 Hz cadence
    bool configured;
} deviceConfig;

// Web server and preferences
AsyncWebServer server(80);
Preferences prefs;
bool configMode = false;

// ========================================
// Function Prototypes
// ========================================
void IRAM_ATTR ppsISR();
void IRAM_ATTR rtcTimerISR();
bool initRM3100();
bool readRM3100(int32_t &x, int32_t &y, int32_t &z);
void convertToMicroTesla(int32_t rawX, int32_t rawY, int32_t rawZ,
                         float &x_uT, float &y_uT, float &z_uT);
bool initMCP9808();
float readMCP9808();
void initGNSS();
bool updateTimeFromGNSS();
bool parseNMEA(const char* sentence);
void formatTimestamp(char* buffer, size_t bufferSize);
bool initLoRa();
bool transmitLoRa(const char* data);
void loadConfiguration();
void saveConfiguration();
void startConfigPortal();
void applyMagOrientation(float &x, float &y, float &z);
void takeSample();
void outputJSON();

// ========================================
// PPS Interrupt Service Routine
// ========================================
void IRAM_ATTR ppsISR() {
    ppsTriggered = true; // lastPpsMillis will be updated in loop context
}

// ========================================
// RTC 1 Hz Timer ISR (fallback cadence)
// ========================================
void IRAM_ATTR rtcTimerISR() {
    rtcTick = true;
}

// ========================================
// RM3100 Initialization
// ========================================
bool initRM3100() {
    uint8_t revid;

    // Read REVID register to verify device
    Wire.beginTransmission(RM3100_I2C_ADDRESS);
    Wire.write(RM3100I2C_REVID);
    if (Wire.endTransmission() != 0) {
        Serial.println("RM3100: No ACK from device");
        return false;
    }

    Wire.requestFrom(RM3100_I2C_ADDRESS, 1);
    if (Wire.available()) {
        revid = Wire.read();
        Serial.print("RM3100 REVID: 0x");
        Serial.println(revid, HEX);
        if (revid != RM3100_VER_EXPECTED) {
            Serial.println("RM3100: Unexpected version");
            return false;
        }
    } else {
        Serial.println("RM3100: Failed to read REVID");
        return false;
    }

    // Set Cycle Count to 200 for all axes (default, good balance)
    uint8_t ccHigh = (CC_200 >> 8) & 0xFF;
    uint8_t ccLow = CC_200 & 0xFF;

    // Set CCX
    Wire.beginTransmission(RM3100_I2C_ADDRESS);
    Wire.write(RM3100I2C_CCX_1);
    Wire.write(ccHigh);
    Wire.write(ccLow);
    Wire.endTransmission();

    // Set CCY
    Wire.beginTransmission(RM3100_I2C_ADDRESS);
    Wire.write(RM3100I2C_CCY_1);
    Wire.write(ccHigh);
    Wire.write(ccLow);
    Wire.endTransmission();

    // Set CCZ
    Wire.beginTransmission(RM3100_I2C_ADDRESS);
    Wire.write(RM3100I2C_CCZ_1);
    Wire.write(ccHigh);
    Wire.write(ccLow);
    Wire.endTransmission();

    Serial.println("RM3100: Initialized successfully");
    return true;
}

// ========================================
// RM3100 Read Function (Polled Mode)
// ========================================
bool readRM3100(int32_t &x, int32_t &y, int32_t &z) {
    // Poll for single measurement (all axes)
    Wire.beginTransmission(RM3100_I2C_ADDRESS);
    Wire.write(RM3100_MAG_POLL);
    Wire.write(PMMODE_ALL);
    if (Wire.endTransmission() != 0) {
        Serial.println("RM3100: Poll command failed");
        return false;
    }

    // Wait for data ready (polling STATUS register)
    unsigned long startTime = millis();
    bool dataReady = false;
    while (millis() - startTime < 50) {  // 50ms timeout
        Wire.beginTransmission(RM3100_I2C_ADDRESS);
        Wire.write(RM3100I2C_STATUS);
        Wire.endTransmission();

        Wire.requestFrom(RM3100_I2C_ADDRESS, 1);
        if (Wire.available()) {
            uint8_t status = Wire.read();
            if (status & 0x80) {  // DRDY bit
                dataReady = true;
                break;
            }
        }
        delay(1);
    }

    if (!dataReady) {
        Serial.println("RM3100: Timeout waiting for data");
        return false;
    }

    // Read X, Y, Z measurements (24-bit each, 9 bytes total)
    Wire.beginTransmission(RM3100_I2C_ADDRESS);
    Wire.write(RM3100I2C_MX_2);
    Wire.endTransmission();

    Wire.requestFrom(RM3100_I2C_ADDRESS, 9);
    if (Wire.available() >= 9) {
        // Read X (3 bytes, signed 24-bit)
        uint8_t xH = Wire.read();
        uint8_t xM = Wire.read();
        uint8_t xL = Wire.read();
        x = ((int32_t)xH << 16) | ((int32_t)xM << 8) | xL;
        if (x & 0x800000) x |= 0xFF000000;  // Sign extend

        // Read Y
        uint8_t yH = Wire.read();
        uint8_t yM = Wire.read();
        uint8_t yL = Wire.read();
        y = ((int32_t)yH << 16) | ((int32_t)yM << 8) | yL;
        if (y & 0x800000) y |= 0xFF000000;

        // Read Z
        uint8_t zH = Wire.read();
        uint8_t zM = Wire.read();
        uint8_t zL = Wire.read();
        z = ((int32_t)zH << 16) | ((int32_t)zM << 8) | zL;
        if (z & 0x800000) z |= 0xFF000000;

        return true;
    }

    Serial.println("RM3100: Failed to read measurement data");
    return false;
}

// ========================================
// Convert RM3100 Raw to MicroTesla
// ========================================
void convertToMicroTesla(int32_t rawX, int32_t rawY, int32_t rawZ,
                         float &x_uT, float &y_uT, float &z_uT) {
    // RM3100 conversion: raw_value / gain = uT
    x_uT = (float)rawX / currentGain;
    y_uT = (float)rawY / currentGain;
    z_uT = (float)rawZ / currentGain;
}

// ========================================
// MCP9808 Initialization
// ========================================
bool initMCP9808() {
    // Try to read Manufacturer ID
    Wire.beginTransmission(MCP9808_RMT_I2CADDR_DEFAULT);
    Wire.write(MCP9808_REG_MANUF_ID);
    if (Wire.endTransmission() != 0) {
        Serial.println("MCP9808: No ACK from device");
        return false;
    }

    Wire.requestFrom(MCP9808_RMT_I2CADDR_DEFAULT, 2);
    if (Wire.available() >= 2) {
        uint16_t manID = (Wire.read() << 8) | Wire.read();
        Serial.print("MCP9808 Manufacturer ID: 0x");
        Serial.println(manID, HEX);
        if (manID != MCP9808_MANID_EXPECTED) {
            Serial.println("MCP9808: Unexpected manufacturer ID");
            return false;
        }
    } else {
        Serial.println("MCP9808: Failed to read manufacturer ID");
        return false;
    }

    // Read Device ID
    Wire.beginTransmission(MCP9808_RMT_I2CADDR_DEFAULT);
    Wire.write(MCP9808_REG_DEVICE_ID);
    Wire.endTransmission();

    Wire.requestFrom(MCP9808_RMT_I2CADDR_DEFAULT, 2);
    if (Wire.available() >= 2) {
        uint16_t devID = (Wire.read() << 8) | Wire.read();
        Serial.print("MCP9808 Device ID: 0x");
        Serial.println(devID, HEX);
    }

    // Set resolution to 0.0625°C (highest)
    Wire.beginTransmission(MCP9808_RMT_I2CADDR_DEFAULT);
    Wire.write(MCP9808_REG_RESOLUTION);
    Wire.write(0x03);  // 0.0625°C resolution
    Wire.endTransmission();

    Serial.println("MCP9808: Initialized successfully");
    return true;
}

// ========================================
// MCP9808 Read Temperature
// ========================================
float readMCP9808() {
    Wire.beginTransmission(MCP9808_RMT_I2CADDR_DEFAULT);
    Wire.write(MCP9808_REG_AMBIENT_TEMP);
    if (Wire.endTransmission() != 0) {
        Serial.println("MCP9808: Read failed");
        return -273.15;  // Error value
    }

    Wire.requestFrom(MCP9808_RMT_I2CADDR_DEFAULT, 2);
    if (Wire.available() >= 2) {
        uint8_t upperByte = Wire.read();
        uint8_t lowerByte = Wire.read();

        // Clear flag bits
        upperByte &= 0x1F;

        // Calculate temperature
        float temp = (upperByte * 16.0) + (lowerByte / 16.0);

        // Check sign bit
        if (upperByte & 0x10) {
            temp = temp - 256.0;
        }

        return temp;
    }

    Serial.println("MCP9808: No data available");
    return -273.15;
}

// ========================================
// GNSS Initialization
// ========================================
void initGNSS() {
    GNSSSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
    delay(100);
    Serial.print("GNSS UART initialized @ ");
    Serial.println(GNSS_BAUD);
}

// ========================================
// Update Time from GNSS
// ========================================
bool updateTimeFromGNSS() {
    static char nmeaBuffer[128];
    static int bufferIndex = 0;
    unsigned long startTime = millis();
    bool timeUpdated = false;

    // Read NMEA sentences for up to 5 seconds
    while (millis() - startTime < 5000 && !timeUpdated) {
        while (GNSSSerial.available() && !timeUpdated) {
            char c = GNSSSerial.read();

            if (c == '$') {
                bufferIndex = 0;
                nmeaBuffer[bufferIndex++] = c;
            } else if (c == '\n' || c == '\r') {
                if (bufferIndex > 0) {
                    nmeaBuffer[bufferIndex] = '\0';
                    if (parseNMEA(nmeaBuffer)) {
                        timeUpdated = true;
                    }
                    bufferIndex = 0;
                }
            } else if (bufferIndex < sizeof(nmeaBuffer) - 1) {
                nmeaBuffer[bufferIndex++] = c;
            }
        }
        delay(10);
    }

    return timeUpdated;
}

// ========================================
// Parse NMEA Sentence (RMC or GGA)
// ========================================
bool parseNMEA(const char* sentence) {
    // Look for $GPRMC or $GNRMC sentences (Recommended Minimum)
    if (strncmp(sentence, "$GPRMC", 6) != 0 && strncmp(sentence, "$GNRMC", 6) != 0) {
        return false;
    }

    // Parse RMC: $GPRMC,hhmmss.ss,A,ddmm.mmmm,N,dddmm.mmmm,E,speed,course,ddmmyy,...
    char* token;
    char sentenceCopy[128];
    strncpy(sentenceCopy, sentence, sizeof(sentenceCopy) - 1);
    sentenceCopy[sizeof(sentenceCopy) - 1] = '\0';

    token = strtok(sentenceCopy, ",");  // $GPRMC
    if (!token) return false;

    token = strtok(NULL, ",");  // Time: hhmmss.ss
    if (!token || strlen(token) < 6) return false;

    int timeVal = atoi(token);
    gnssTime.hour = timeVal / 10000;
    gnssTime.minute = (timeVal / 100) % 100;
    gnssTime.second = timeVal % 100;

    token = strtok(NULL, ",");  // Status: A=active, V=void
    if (!token || token[0] != 'A') return false;

    // Skip latitude, N/S, longitude, E/W, speed, course
    for (int i = 0; i < 6; i++) {
        token = strtok(NULL, ",");
        if (!token) return false;
    }

    token = strtok(NULL, ",");  // Date: ddmmyy
    if (!token || strlen(token) < 6) return false;

    int dateVal = atoi(token);
    gnssTime.day = dateVal / 10000;
    gnssTime.month = (dateVal / 100) % 100;
    gnssTime.year = 2000 + (dateVal % 100);

    gnssTime.valid = true;
    Serial.println("GNSS time updated successfully");
    return true;
}

// ========================================
// Format Timestamp for JSON
// ========================================
void formatTimestamp(char* buffer, size_t bufferSize) {
    const char* months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    if (gnssTime.valid && gnssTime.month >= 1 && gnssTime.month <= 12) {
        snprintf(buffer, bufferSize, "%02d %s %04d %02d:%02d:%02d",
                 gnssTime.day, months[gnssTime.month - 1], gnssTime.year,
                 gnssTime.hour, gnssTime.minute, gnssTime.second);
    } else {
        snprintf(buffer, bufferSize, "01 Jan 2025 00:00:00");
    }
}

// ========================================
// LoRa Initialization
// ========================================
bool initLoRa() {
    Serial.print("Initializing LoRa SX1262... ");

    // Initialize SX1262
    int state = radio.begin(
        loraConfig.frequency,
        loraConfig.bandwidth,
        loraConfig.spreadFactor,
        loraConfig.codingRate,
        RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
        loraConfig.txPower,
        loraConfig.preamble,
        0  // TCXO voltage (0 = external TCXO)
    );

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("success!");
        Serial.print("  Frequency: "); Serial.print(loraConfig.frequency); Serial.println(" MHz");
        Serial.print("  Bandwidth: "); Serial.print(loraConfig.bandwidth); Serial.println(" kHz");
        Serial.print("  Spreading Factor: "); Serial.println(loraConfig.spreadFactor);
        Serial.print("  TX Power: "); Serial.print(loraConfig.txPower); Serial.println(" dBm");
        return true;
    } else {
        Serial.print("failed, code: ");
        Serial.println(state);
        return false;
    }
}

// ========================================
// LoRa Transmit Function
// ========================================
bool transmitLoRa(const char* data) {
    int state = radio.transmit((uint8_t*)data, strlen(data));

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa: Packet transmitted");
        return true;
    } else {
        Serial.print("LoRa: Transmit failed, code: ");
        Serial.println(state);
        return false;
    }
}

// ========================================
// Load Configuration from Flash
// ========================================
void loadConfiguration() {
    prefs.begin("magConfig", false);

    deviceConfig.configured = prefs.getBool("configured", false);

    if (deviceConfig.configured) {
        prefs.getString("nodeId", deviceConfig.nodeId, sizeof(deviceConfig.nodeId));
        prefs.getString("owner", deviceConfig.owner, sizeof(deviceConfig.owner));
        deviceConfig.latitude = prefs.getFloat("latitude", 0.0);
        deviceConfig.longitude = prefs.getFloat("longitude", 0.0);
        deviceConfig.elevation = prefs.getFloat("elevation", 0.0);
        deviceConfig.magTranslateX = prefs.getInt("magTransX", 0);
        deviceConfig.magTranslateY = prefs.getInt("magTransY", 0);
        deviceConfig.magTranslateZ = prefs.getInt("magTransZ", 0);

        loraConfig.frequency = prefs.getFloat("loraFreq", 915.0);
        loraConfig.bandwidth = prefs.getFloat("loraBW", 125.0);
        loraConfig.spreadFactor = prefs.getUChar("loraSF", 9);
        loraConfig.txPower = prefs.getChar("loraPwr", 17);

        Serial.println("Configuration loaded from flash");
    } else {
        // Set defaults
        strcpy(deviceConfig.nodeId, "MAG-UNCONFIGURED");
        strcpy(deviceConfig.owner, "Unknown");
        deviceConfig.latitude = 0.0;
        deviceConfig.longitude = 0.0;
        deviceConfig.elevation = 0.0;
        deviceConfig.magTranslateX = 0;
        deviceConfig.magTranslateY = 0;
        deviceConfig.magTranslateZ = 0;
        Serial.println("Using default configuration");
    }

    prefs.end();
}

// ========================================
// Save Configuration to Flash
// ========================================
void saveConfiguration() {
    prefs.begin("magConfig", false);

    prefs.putBool("configured", true);
    prefs.putString("nodeId", deviceConfig.nodeId);
    prefs.putString("owner", deviceConfig.owner);
    prefs.putFloat("latitude", deviceConfig.latitude);
    prefs.putFloat("longitude", deviceConfig.longitude);
    prefs.putFloat("elevation", deviceConfig.elevation);
    prefs.putInt("magTransX", deviceConfig.magTranslateX);
    prefs.putInt("magTransY", deviceConfig.magTranslateY);
    prefs.putInt("magTransZ", deviceConfig.magTranslateZ);

    prefs.putFloat("loraFreq", loraConfig.frequency);
    prefs.putFloat("loraBW", loraConfig.bandwidth);
    prefs.putUChar("loraSF", loraConfig.spreadFactor);
    prefs.putChar("loraPwr", loraConfig.txPower);

    prefs.end();

    deviceConfig.configured = true;
    Serial.println("Configuration saved to flash");
}

// ========================================
// Start WiFi Configuration Portal
// ========================================
void startConfigPortal() {
    Serial.println("Starting WiFi Configuration Portal...");

    // Create AP
    WiFi.softAP("MagNode-Config", "configure123");
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Serve configuration page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = R"(
<!DOCTYPE html>
<html>
<head><title>Magnetometer Configuration</title>
<style>body{font-family:Arial;margin:20px;}input,select{width:100%;padding:8px;margin:5px 0;}
button{background:#0066cc;color:white;padding:10px;border:none;cursor:pointer;}
button:hover{background:#0052a3;}</style>
</head>
<body>
<h1>Magnetometer Node Configuration</h1>
<form action="/save" method="POST">
<h3>Node Information</h3>
<label>Node ID:</label><input name="nodeId" value=")";
        html += deviceConfig.nodeId;
        html += R"("><br>
<label>Owner:</label><input name="owner" value=")";
        html += deviceConfig.owner;
        html += R"("><br>
<h3>Location</h3>
<label>Latitude:</label><input name="latitude" type="number" step="0.000001" value=")";
        html += String(deviceConfig.latitude, 6);
        html += R"("><br>
<label>Longitude:</label><input name="longitude" type="number" step="0.000001" value=")";
        html += String(deviceConfig.longitude, 6);
        html += R"("><br>
<label>Elevation (m):</label><input name="elevation" type="number" step="0.1" value=")";
        html += String(deviceConfig.elevation, 1);
        html += R"("><br>
<h3>Magnetometer Orientation (degrees)</h3>
<label>X-axis Rotation:</label><select name="magTransX">
<option value="-180">-180</option><option value="-90">-90</option>
<option value="0" selected>0</option><option value="90">90</option>
<option value="180">180</option></select><br>
<label>Y-axis Rotation:</label><select name="magTransY">
<option value="-180">-180</option><option value="-90">-90</option>
<option value="0" selected>0</option><option value="90">90</option>
<option value="180">180</option></select><br>
<label>Z-axis Rotation:</label><select name="magTransZ">
<option value="-180">-180</option><option value="-90">-90</option>
<option value="0" selected>0</option><option value="90">90</option>
<option value="180">180</option></select><br>
<h3>LoRa Settings</h3>
<label>Frequency (MHz):</label><input name="loraFreq" type="number" step="0.1" value=")";
        html += String(loraConfig.frequency, 1);
        html += R"("><br>
<label>Spreading Factor:</label><select name="loraSF">)";
        for(int i=7; i<=12; i++) {
            html += "<option value=\"" + String(i) + "\"";
            if(i == loraConfig.spreadFactor) html += " selected";
            html += ">" + String(i) + "</option>";
        }
        html += R"(</select><br>
<label>TX Power (dBm):</label><input name="loraPwr" type="number" min="-9" max="22" value=")";
        html += String(loraConfig.txPower);
        html += R"("><br><br>
<button type="submit">Save Configuration</button>
</form>
</body>
</html>
)";
        request->send(200, "text/html", html);
    });

    // Handle save
    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request){
        if(request->hasParam("nodeId", true))
            strcpy(deviceConfig.nodeId, request->getParam("nodeId", true)->value().c_str());
        if(request->hasParam("owner", true))
            strcpy(deviceConfig.owner, request->getParam("owner", true)->value().c_str());
        if(request->hasParam("latitude", true))
            deviceConfig.latitude = request->getParam("latitude", true)->value().toFloat();
        if(request->hasParam("longitude", true))
            deviceConfig.longitude = request->getParam("longitude", true)->value().toFloat();
        if(request->hasParam("elevation", true))
            deviceConfig.elevation = request->getParam("elevation", true)->value().toFloat();
        if(request->hasParam("magTransX", true))
            deviceConfig.magTranslateX = request->getParam("magTransX", true)->value().toInt();
        if(request->hasParam("magTransY", true))
            deviceConfig.magTranslateY = request->getParam("magTransY", true)->value().toInt();
        if(request->hasParam("magTransZ", true))
            deviceConfig.magTranslateZ = request->getParam("magTransZ", true)->value().toInt();
        if(request->hasParam("loraFreq", true))
            loraConfig.frequency = request->getParam("loraFreq", true)->value().toFloat();
        if(request->hasParam("loraSF", true))
            loraConfig.spreadFactor = request->getParam("loraSF", true)->value().toInt();
        if(request->hasParam("loraPwr", true))
            loraConfig.txPower = request->getParam("loraPwr", true)->value().toInt();

        saveConfiguration();

        request->send(200, "text/html",
            "<html><body><h1>Configuration Saved!</h1>"
            "<p>Device will restart in 3 seconds...</p></body></html>");

        delay(3000);
        ESP.restart();
    });

    server.begin();
    Serial.println("Web server started. Connect to 'MagNode-Config' and visit http://192.168.4.1");
}

// ========================================
// Apply Magnetometer Orientation Transform
// ========================================
void applyMagOrientation(float &x, float &y, float &z) {
    float tempX = x, tempY = y, tempZ = z;

    // Rotate around X axis
    switch(deviceConfig.magTranslateX) {
        case 90: case -270:
            y = -tempZ; z = tempY; break;
        case -90: case 270:
            y = tempZ; z = -tempY; break;
        case 180: case -180:
            y = -tempY; z = -tempZ; break;
    }
    tempY = y; tempZ = z;

    // Rotate around Y axis
    switch(deviceConfig.magTranslateY) {
        case 90: case -270:
            x = tempZ; z = -tempX; break;
        case -90: case 270:
            x = -tempZ; z = tempX; break;
        case 180: case -180:
            x = -tempX; z = -tempZ; break;
    }
    tempX = x; tempZ = z;

    // Rotate around Z axis
    switch(deviceConfig.magTranslateZ) {
        case 90: case -270:
            x = -tempY; y = tempX; break;
        case -90: case 270:
            x = tempY; y = -tempX; break;
        case 180: case -180:
            x = -tempX; y = -tempY; break;
    }
}

// ========================================
// Take Sample from Both Sensors
// ========================================
void takeSample() {
    // Read magnetometer
    if (readRM3100(magX, magY, magZ)) {
        convertToMicroTesla(magX, magY, magZ, magX_uT, magY_uT, magZ_uT);
        // Apply orientation transform
        applyMagOrientation(magX_uT, magY_uT, magZ_uT);
    } else {
        Serial.println("Failed to read RM3100");
    }

    // Read temperature
    temperature = readMCP9808();
    if (temperature < -200.0) {
        Serial.println("Failed to read MCP9808");
    }
}

// ========================================
// Output JSON Data
// ========================================
void outputJSON() {
    // Create JSON document
    JsonDocument doc;

    // Format timestamp from GNSS time
    char timestamp[32];
    formatTimestamp(timestamp, sizeof(timestamp));

    doc["ts"] = timestamp;
    doc["rt"] = serialized(String(temperature, 3));
    doc["x"] = serialized(String(magX_uT / 1000.0, 3));  // Convert to nT
    doc["y"] = serialized(String(magY_uT / 1000.0, 3));
    doc["z"] = serialized(String(magZ_uT / 1000.0, 3));

    // Serialize to string for transmission
    String jsonString;
    serializeJson(doc, jsonString);

    // Print to Serial
    Serial.println(jsonString);

    // Transmit via LoRa
    transmitLoRa(jsonString.c_str());
}

// ========================================
// Setup
// ========================================
void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial connection
    Serial.println("\n\nMagnetometer Data Logger - Initializing...");

    // Load configuration
    loadConfiguration();

    // Check config button for forced configuration mode
    pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
    delay(100);
    if (digitalRead(CONFIG_BUTTON_PIN) == LOW || !deviceConfig.configured) {
        Serial.println("Entering configuration mode...");
        configMode = true;
        startConfigPortal();
        // Stay in config mode
        while(true) {
            delay(100);
        }
    }

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_STANDARD);
    Serial.println("I2C initialized");

    // Initialize RM3100
    if (!initRM3100()) {
        Serial.println("ERROR: RM3100 initialization failed!");
        while(1) delay(1000);
    }

    // Initialize MCP9808
    if (!initMCP9808()) {
        Serial.println("ERROR: MCP9808 initialization failed!");
        while(1) delay(1000);
    }

    // Initialize GNSS
    initGNSS();

    // Try to get initial time from GNSS
    Serial.println("Attempting to get time from GNSS...");
    if (updateTimeFromGNSS()) {
        Serial.print("Time synchronized: ");
        char timestamp[32];
        formatTimestamp(timestamp, sizeof(timestamp));
        Serial.println(timestamp);
    } else {
        Serial.println("Warning: Could not get GNSS time, using default");
    }
    lastGNSSUpdate = millis();

    // Configure PPS pin for interrupt
    pinMode(GNSS_PPS_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), ppsISR, RISING);
    Serial.println("PPS interrupt configured");

    // Initialize LoRa
    if (!initLoRa()) {
        Serial.println("WARNING: LoRa initialization failed - continuing without radio");
    }

    Serial.println("Initialization complete. Waiting for PPS trigger...");
}

// ========================================
// Main Loop
// ========================================
void loop() {
    // Check if PPS interrupt fired
    if (ppsTriggered) {
        ppsTriggered = false;

        // Take samples from sensors
        takeSample();

        // Output JSON data
        outputJSON();

        // Increment time by 1 second (assuming 1 Hz PPS)
        gnssTime.second++;
        if (gnssTime.second >= 60) {
            gnssTime.second = 0;
            gnssTime.minute++;
            if (gnssTime.minute >= 60) {
                gnssTime.minute = 0;
                gnssTime.hour++;
                if (gnssTime.hour >= 24) {
                    gnssTime.hour = 0;
                    // Note: Date rollover not implemented for simplicity
                }
            }
        }

        lastSampleTime = millis();
    }

    // Update time from GNSS hourly (REQ-004)
    if (millis() - lastGNSSUpdate >= GNSS_UPDATE_INTERVAL) {
        Serial.println("Hourly GNSS time sync...");
        if (updateTimeFromGNSS()) {
            Serial.println("Time re-synchronized");
        } else {
            Serial.println("GNSS sync failed, continuing with local time");
        }
        lastGNSSUpdate = millis();
    }

    // Small delay to prevent tight loop
    delay(10);
}