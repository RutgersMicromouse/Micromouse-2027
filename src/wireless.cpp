#include "wireless.h"

// ==============================================================================
// BLUETOOTH LOW ENERGY
// ==============================================================================

#if ENABLE_BLE_DEBUG

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =============================================================
// NORDIC UART SERVICE
// =============================================================

#define SERVICE_UUID \
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

#define CHARACTERISTIC_UUID_RX \
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#define CHARACTERISTIC_UUID_TX \
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// =============================================================
// MICROMOUSE TELEMETRY SERVICE
// =============================================================

#define TELEMETRY_SERVICE_UUID \
    "7A100001-0000-4A5B-8C9D-123456789ABC"

#define BATTERY_UUID \
    "7A100002-0000-4A5B-8C9D-123456789ABC"

#define HEADING_UUID \
    "7A100003-0000-4A5B-8C9D-123456789ABC"

#define LEFT_ENCODER_UUID \
    "7A100004-0000-4A5B-8C9D-123456789ABC"

#define RIGHT_ENCODER_UUID \
    "7A100005-0000-4A5B-8C9D-123456789ABC"

// =============================================================
// BLE OBJECTS
// =============================================================

static BLEServer* s_server = nullptr;

// Nordic UART

static BLECharacteristic*
    s_tx_characteristic = nullptr;

static BLECharacteristic*
    s_rx_characteristic = nullptr;

// Telemetry

static BLECharacteristic*
    s_battery_characteristic = nullptr;

static BLECharacteristic*
    s_heading_characteristic = nullptr;

static BLECharacteristic*
    s_left_encoder_characteristic = nullptr;

static BLECharacteristic*
    s_right_encoder_characteristic = nullptr;

// =============================================================
// CONNECTION STATE
// =============================================================

static bool s_device_connected = false;

// =============================================================
// COMMAND STATE
//
// Written by the BLE host task, read by the navigation task, so
// every access goes through s_rx_mux. Fixed-size buffers keep
// heap allocation out of the critical sections.
// =============================================================

#define BLE_CMD_MAX_LEN 64

static portMUX_TYPE s_rx_mux = portMUX_INITIALIZER_UNLOCKED;

// Characters of the command currently being received
static char s_rx_partial[BLE_CMD_MAX_LEN];
static size_t s_rx_partial_len = 0;

// Last complete command, waiting to be read
static char s_rx_command[BLE_CMD_MAX_LEN];
static volatile bool s_command_ready = false;

// =============================================================
// SERVER CALLBACKS
// =============================================================

class ServerCallbacks : public BLEServerCallbacks {

    void onConnect(
        BLEServer* pServer
    ) {

        s_device_connected = true;

        Serial.println(
            "[BLE] Device connected"
        );
    }

    void onDisconnect(
        BLEServer* pServer
    ) {

        s_device_connected = false;

        Serial.println(
            "[BLE] Device disconnected"
        );

        // Allow phone to reconnect
        pServer->startAdvertising();
    }
};

// =============================================================
// RX CALLBACK
// =============================================================

class RxCallbacks :
    public BLECharacteristicCallbacks {

    void onWrite(
        BLECharacteristic* pCharacteristic
    ) {

        std::string rxValue =
            pCharacteristic->getValue();

        portENTER_CRITICAL(&s_rx_mux);

        for (size_t i = 0; i < rxValue.length(); i++) {

            char c = rxValue[i];

            if (c == '\r' || c == '\n') {

                if (s_rx_partial_len > 0) {

                    memcpy(s_rx_command, s_rx_partial, s_rx_partial_len);
                    s_rx_command[s_rx_partial_len] = '\0';
                    s_rx_partial_len = 0;
                    s_command_ready = true;
                }

            } else if (s_rx_partial_len < BLE_CMD_MAX_LEN - 1) {

                s_rx_partial[s_rx_partial_len++] = c;
            }
        }

        portEXIT_CRITICAL(&s_rx_mux);
    }
};

// =============================================================
// BLE BEGIN
// =============================================================

void BLEDebug::begin(
    const char* device_name
) {

    Serial.printf(
        "[BLE] Initializing Bluetooth Low Energy (%s)...\n",
        device_name
    );

    // ---------------------------------------------------------
    // Initialize BLE
    // ---------------------------------------------------------

    BLEDevice::init(
        device_name
    );

    // ---------------------------------------------------------
    // Create BLE server
    // ---------------------------------------------------------

    s_server =
        BLEDevice::createServer();

    s_server->setCallbacks(
        new ServerCallbacks()
    );

    // =========================================================
    // CREATE NORDIC UART SERVICE
    // =========================================================

    BLEService* uartService =
        s_server->createService(
            SERVICE_UUID
        );

    // ---------------------------------------------------------
    // UART TX
    // Robot -> Phone
    // ---------------------------------------------------------

    s_tx_characteristic =
        uartService->createCharacteristic(
            CHARACTERISTIC_UUID_TX,

            BLECharacteristic::PROPERTY_NOTIFY
        );

    s_tx_characteristic->addDescriptor(
        new BLE2902()
    );

    // ---------------------------------------------------------
    // UART RX
    // Phone -> Robot
    // ---------------------------------------------------------

    s_rx_characteristic =
        uartService->createCharacteristic(
            CHARACTERISTIC_UUID_RX,

            BLECharacteristic::PROPERTY_WRITE
            |
            BLECharacteristic::PROPERTY_WRITE_NR
        );

    s_rx_characteristic->setCallbacks(
        new RxCallbacks()
    );

    uartService->start();

    // =========================================================
    // CREATE MICROMOUSE TELEMETRY SERVICE
    // =========================================================

    BLEService* telemetryService =
        s_server->createService(
            TELEMETRY_SERVICE_UUID
        );

    // ---------------------------------------------------------
    // Battery
    // ---------------------------------------------------------

    s_battery_characteristic =
        telemetryService->createCharacteristic(
            BATTERY_UUID,

            BLECharacteristic::PROPERTY_READ
        );

    s_battery_characteristic->setValue(
        "0.00 V"
    );

    // ---------------------------------------------------------
    // Heading
    // ---------------------------------------------------------

    s_heading_characteristic =
        telemetryService->createCharacteristic(
            HEADING_UUID,

            BLECharacteristic::PROPERTY_READ
        );

    s_heading_characteristic->setValue(
        "0.0 deg"
    );

    // ---------------------------------------------------------
    // Left Encoder
    // ---------------------------------------------------------

    s_left_encoder_characteristic =
        telemetryService->createCharacteristic(
            LEFT_ENCODER_UUID,

            BLECharacteristic::PROPERTY_READ
        );

    s_left_encoder_characteristic->setValue(
        "0"
    );

    // ---------------------------------------------------------
    // Right Encoder
    // ---------------------------------------------------------

    s_right_encoder_characteristic =
        telemetryService->createCharacteristic(
            RIGHT_ENCODER_UUID,

            BLECharacteristic::PROPERTY_READ
        );

    s_right_encoder_characteristic->setValue(
        "0"
    );

    telemetryService->start();

    // =========================================================
    // START ADVERTISING
    // =========================================================

    BLEAdvertising* pAdvertising =
        BLEDevice::getAdvertising();

    // Advertise Nordic UART
    pAdvertising->addServiceUUID(
        SERVICE_UUID
    );

    // Advertise telemetry service
    pAdvertising->addServiceUUID(
        TELEMETRY_SERVICE_UUID
    );

    pAdvertising->setScanResponse(
        true
    );

    pAdvertising->setMinPreferred(
        0x06
    );

    pAdvertising->setMinPreferred(
        0x12
    );

    BLEDevice::startAdvertising();

    Serial.println(
        "[BLE] UART + Telemetry Services Ready!"
    );
}

// =============================================================
// CONNECTION STATUS
// =============================================================

bool BLEDebug::isConnected() {

    return s_device_connected;
}

// =============================================================
// UPDATE LIVE TELEMETRY VALUES
// =============================================================

void BLEDebug::updateTelemetry(
    float battery_voltage,
    float heading_deg,
    long left_encoder,
    long right_encoder
) {

    char buffer[32];

    // ---------------------------------------------------------
    // Battery
    // ---------------------------------------------------------

    snprintf(
        buffer,
        sizeof(buffer),
        "%.2f V",
        battery_voltage
    );

    if (s_battery_characteristic) {

        s_battery_characteristic->setValue(
            buffer
        );
    }

    // ---------------------------------------------------------
    // Heading
    // ---------------------------------------------------------

    snprintf(
        buffer,
        sizeof(buffer),
        "%.1f deg",
        heading_deg
    );

    if (s_heading_characteristic) {

        s_heading_characteristic->setValue(
            buffer
        );
    }

    // ---------------------------------------------------------
    // Left Encoder
    // ---------------------------------------------------------

    snprintf(
        buffer,
        sizeof(buffer),
        "%ld",
        left_encoder
    );

    if (s_left_encoder_characteristic) {

        s_left_encoder_characteristic->setValue(
            buffer
        );
    }

    // ---------------------------------------------------------
    // Right Encoder
    // ---------------------------------------------------------

    snprintf(
        buffer,
        sizeof(buffer),
        "%ld",
        right_encoder
    );

    if (s_right_encoder_characteristic) {

        s_right_encoder_characteristic->setValue(
            buffer
        );
    }
}

// =============================================================
// UART PRINT
// =============================================================

void BLEDebug::print(
    const char* str
) {

    if (
        !s_device_connected
        ||
        !s_tx_characteristic
    ) {

        return;
    }

    size_t len =
        strlen(str);

    size_t offset = 0;

    while (offset < len) {

        size_t chunk =
            len - offset;

        // Keep notification payload <= 20 bytes
        if (chunk > 20) {

            chunk = 20;
        }

        s_tx_characteristic->setValue(
            (uint8_t*)(str + offset),
            chunk
        );

        s_tx_characteristic->notify();

        offset += chunk;

        delay(2);
    }
}

// =============================================================
// UART PRINTLN
// =============================================================

void BLEDebug::println(
    const char* str
) {

    print(str);

    print("\r\n");
}

// =============================================================
// UART PRINTF
// =============================================================

void BLEDebug::printf(
    const char* fmt,
    ...
) {

    char buf[256];

    va_list args;

    va_start(
        args,
        fmt
    );

    vsnprintf(
        buf,
        sizeof(buf),
        fmt,
        args
    );

    va_end(args);

    print(buf);
}

// =============================================================
// COMMAND AVAILABLE
// =============================================================

bool BLEDebug::hasCommand() {

    return s_command_ready;
}

// =============================================================
// READ COMMAND
// =============================================================

String BLEDebug::readCommand() {

    char buffer[BLE_CMD_MAX_LEN];

    portENTER_CRITICAL(&s_rx_mux);

    memcpy(buffer, s_rx_command, sizeof(buffer));
    s_rx_command[0] = '\0';
    s_command_ready = false;

    portEXIT_CRITICAL(&s_rx_mux);

    buffer[BLE_CMD_MAX_LEN - 1] = '\0';

    String cmd = buffer;

    cmd.trim();

    return cmd;
}

#endif // ENABLE_BLE_DEBUG

// ==============================================================================
// WI-FI OTA & TELNET
// ==============================================================================

#if ENABLE_WIFI_OTA

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Update.h>

static WebServer  s_web_server(80);
static WiFiServer s_telnet_server(TELNET_PORT);
static WiFiClient s_telnet_client;

static String s_telnet_rx_buf = "";
static bool   s_telnet_cmd_ready = false;

// Web update page HTML
static const char UPDATE_INDEX_HTML[] PROGMEM =
    "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Antigrav-Mouse OTA</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:-apple-system,sans-serif;background:#121214;color:#eee;text-align:center;padding:40px 20px;}"
    ".card{background:#1e1e24;max-width:460px;margin:0 auto;padding:30px;border-radius:12px;box-shadow:0 8px 24px rgba(0,0,0,0.5);}"
    "h1{color:#4af;margin-bottom:8px;}p{color:#aaa;font-size:14px;}"
    "input[type='file']{display:block;margin:24px auto;color:#ccc;}"
    "input[type='submit']{background:#4af;color:#000;font-weight:700;border:none;padding:12px 28px;border-radius:6px;cursor:pointer;font-size:16px;}"
    "input[type='submit']:hover{background:#38d;}"
    "#progress{display:none;margin-top:20px;font-size:14px;color:#fa4;}"
    "</style></head><body>"
    "<div class='card'>"
    "<h1>🐭 Antigrav-Mouse</h1>"
    "<p>Over-The-Air Wireless Firmware Update</p>"
    "<form method='POST' action='/update' enctype='multipart/form-data' onsubmit='document.getElementById(\"progress\").style.display=\"block\";'>"
    "<input type='file' name='update' accept='.bin' required>"
    "<input type='submit' value='Upload & Flash Firmware'>"
    "</form>"
    "<div id='progress'>⚡ Flashing firmware to ESP32-S3... Robot will reboot in ~8 seconds.</div>"
    "</div></body></html>";

void WifiOTA::begin() {
    Serial.println("[WIFI] Initializing Wireless Network for OTA & Debugging...");

#if WIFI_AP_MODE
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    IPAddress myIP = WiFi.softAPIP();
    Serial.printf("[WIFI] Hotspot Started: SSID '%s' | IP: %s\n", WIFI_AP_SSID, myIP.toString().c_str());
#else
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    Serial.printf("[WIFI] Connecting to '%s'...\n", WIFI_STA_SSID);
    uint32_t start_ms = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start_ms < 6000)) {
        delay(200);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WIFI] STA timeout, falling back to SoftAP...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    }
#endif

    // 1. Configure ArduinoOTA (for PlatformIO CLI: pio run -t upload)
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setHostname("antigrav-mouse");

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("\n[OTA] ⚡ Wireless Firmware Update Started: " + type);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Update Complete! Rebooting ESP32-S3...");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.printf("[OTA] ArduinoOTA service listening on port %d\n", OTA_PORT);

    // 2. Configure WebServer for Browser-based OTA updates
    s_web_server.on("/", HTTP_GET, []() {
        s_web_server.send(200, "text/html", UPDATE_INDEX_HTML);
    });

    s_web_server.on("/update", HTTP_GET, []() {
        s_web_server.send(200, "text/html", UPDATE_INDEX_HTML);
    });

    s_web_server.on("/update", HTTP_POST, []() {
        s_web_server.sendHeader("Connection", "close");
        s_web_server.send(200, "text/plain", (Update.hasError()) ? "UPDATE FAIL" : "UPDATE SUCCESS! Rebooting...");
        delay(1000);
        ESP.restart();
    }, []() {
        HTTPUpload& upload = s_web_server.upload();
        if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("[WEB OTA] Upload Started: %s\n", upload.filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) {
                Serial.printf("[WEB OTA] Success: %u bytes written. Rebooting...\n", upload.totalSize);
            } else {
                Update.printError(Serial);
            }
        }
    });

    s_web_server.begin();
    Serial.println("[WEB] Browser OTA server listening on port 80 (http://192.168.4.1/update)");

    // 3. Configure Telnet Server for Wireless Terminal Monitoring
    s_telnet_server.begin();
    s_telnet_server.setNoDelay(true);
    Serial.printf("[TELNET] Wireless console listening on port %d (telnet 192.168.4.1)\n", TELNET_PORT);
}

void WifiOTA::handle() {
    ArduinoOTA.handle();
    s_web_server.handleClient();

    // Handle Telnet client connections
    if (s_telnet_server.hasClient()) {
        if (!s_telnet_client || !s_telnet_client.connected()) {
            if (s_telnet_client) s_telnet_client.stop();
            s_telnet_client = s_telnet_server.available();
            s_telnet_client.println("\n=== Connected to Antigrav-Mouse Telnet Console ===");
            s_telnet_client.println("Type 'help' for commands.\r\n");
            Serial.println("[TELNET] Client connected wirelessly.");
        } else {
            // Reject second client
            WiFiClient rejected = s_telnet_server.available();
            rejected.stop();
        }
    }

    // Read incoming commands from Telnet client
    if (s_telnet_client && s_telnet_client.connected() && s_telnet_client.available()) {
        while (s_telnet_client.available()) {
            char c = s_telnet_client.read();
            if (c == '\r' || c == '\n') {
                if (s_telnet_rx_buf.length() > 0) {
                    s_telnet_cmd_ready = true;
                    break;
                }
            } else {
                s_telnet_rx_buf += c;
            }
        }
    }
}

bool WifiOTA::isClientConnected() {
    return (s_telnet_client && s_telnet_client.connected());
}

void WifiOTA::print(const char* str) {
    if (s_telnet_client && s_telnet_client.connected()) {
        s_telnet_client.print(str);
    }
}

void WifiOTA::println(const char* str) {
    if (s_telnet_client && s_telnet_client.connected()) {
        s_telnet_client.println(str);
    }
}

void WifiOTA::printf(const char* fmt, ...) {
    if (!s_telnet_client || !s_telnet_client.connected()) return;
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    s_telnet_client.print(buf);
}

bool WifiOTA::hasCommand() {
    return s_telnet_cmd_ready;
}

String WifiOTA::readCommand() {
    String cmd = s_telnet_rx_buf;
    s_telnet_rx_buf = "";
    s_telnet_cmd_ready = false;
    cmd.trim();
    return cmd;
}

IPAddress WifiOTA::getIP() {
#if WIFI_AP_MODE
    return WiFi.softAPIP();
#else
    return (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : WiFi.softAPIP();
#endif
}

#endif // ENABLE_WIFI_OTA
