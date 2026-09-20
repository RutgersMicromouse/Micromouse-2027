#include "ble_debug.h"

#if ENABLE_BLE_DEBUG

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // Nordic UART Service
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static BLEServer*          s_server = nullptr;
static BLECharacteristic*  s_tx_characteristic = nullptr;
static BLECharacteristic*  s_rx_characteristic = nullptr;

static bool   s_device_connected = false;
static String s_rx_command_buffer = "";
static bool   s_command_ready = false;

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        s_device_connected = true;
    }

    void onDisconnect(BLEServer* pServer) {
        s_device_connected = false;
        // Restart advertising so we can reconnect anytime
        pServer->startAdvertising();
    }
};

class RxCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            for (size_t i = 0; i < rxValue.length(); i++) {
                char c = rxValue[i];
                if (c == '\r' || c == '\n') {
                    if (s_rx_command_buffer.length() > 0) {
                        s_command_ready = true;
                    }
                } else {
                    s_rx_command_buffer += c;
                }
            }
        }
    }
};

void BLEDebug::begin(const char* device_name) {
    Serial.printf("[BLE] Initializing Bluetooth Low Energy (%s)...\n", device_name);
    BLEDevice::init(device_name);

    s_server = BLEDevice::createServer();
    s_server->setCallbacks(new ServerCallbacks());

    BLEService* pService = s_server->createService(SERVICE_UUID);

    // TX Characteristic (Notify robot telemetry to phone)
    s_tx_characteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    s_tx_characteristic->addDescriptor(new BLE2902());

    // RX Characteristic (Receive commands from phone)
    s_rx_characteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    s_rx_characteristic->setCallbacks(new RxCallbacks());

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] BLE UART Service Ready! Connect using 'Serial Bluetooth Terminal' or 'nRF Connect'.");
}

bool BLEDebug::isConnected() {
    return s_device_connected;
}

void BLEDebug::print(const char* str) {
    if (!s_device_connected || !s_tx_characteristic) return;

    size_t len = strlen(str);
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > 20) chunk = 20; // BLE MTU safe payload
        s_tx_characteristic->setValue((uint8_t*)(str + offset), chunk);
        s_tx_characteristic->notify();
        offset += chunk;
        delay(2);
    }
}

void BLEDebug::println(const char* str) {
    print(str);
    print("\r\n");
}

void BLEDebug::printf(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    print(buf);
}

bool BLEDebug::hasCommand() {
    return s_command_ready;
}

String BLEDebug::readCommand() {
    String cmd = s_rx_command_buffer;
    s_rx_command_buffer = "";
    s_command_ready = false;
    cmd.trim();
    return cmd;
}

#endif // ENABLE_BLE_DEBUG
