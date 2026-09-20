#include "wifi_ota.h"

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
