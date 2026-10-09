#include "wireless/ble_debug/ble_debug.h"

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

#define BLE_CMD_MAX_LEN 200

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
