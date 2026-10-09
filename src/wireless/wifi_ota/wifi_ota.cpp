#include "wireless/wifi_ota/wifi_ota.h"

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
static uint32_t s_commands_taken = 0;   // Commands the console has finished with so far (the app's buttons watch this)

#include "wireless/wifi_ota/web_pages.h" // UPDATE_INDEX_HTML and APP_PAGE_HTML

// Recent robot output kept for the phone app, which asks for "everything since byte N"
static portMUX_TYPE s_web_log_mux = portMUX_INITIALIZER_UNLOCKED;
static char     s_web_log[4096];
static uint32_t s_web_log_total = 0; // Bytes ever logged; the newest byte is at (total - 1) % size

static String (*s_status_provider)() = nullptr;

void WifiOTA::setStatusProvider(String (*provider)()) {
    s_status_provider = provider;
}

void WifiOTA::appendWebLog(const char* text, size_t length) {
    portENTER_CRITICAL(&s_web_log_mux);
    for (size_t i = 0; i < length; ++i) {
        s_web_log[s_web_log_total % sizeof(s_web_log)] = text[i];
        s_web_log_total++;
    }
    portEXIT_CRITICAL(&s_web_log_mux);
}

// GET /data?since=N  ->  {"next":M,"done":K,"log":"...new output...","status":{...}}
// "done" counts the commands the console has taken; the page uses it to tell when a button's
// command has been carried out.
static void handleAppData() {
    static char slice[1025];
    uint32_t since = (uint32_t)s_web_server.arg("since").toInt();

    portENTER_CRITICAL(&s_web_log_mux);
    uint32_t total = s_web_log_total;
    uint32_t oldest = (total > sizeof(s_web_log)) ? total - sizeof(s_web_log) : 0;
    if (since > total || since < oldest) since = oldest; // Robot restarted, or the app fell too far behind
    uint32_t count = total - since;
    if (count > sizeof(slice) - 1) count = sizeof(slice) - 1;
    for (uint32_t i = 0; i < count; ++i) {
        slice[i] = s_web_log[(since + i) % sizeof(s_web_log)];
    }
    portEXIT_CRITICAL(&s_web_log_mux);

    String out;
    out.reserve(count + 2400);
    out += "{\"next\":";
    out += String(since + count);
    out += ",\"done\":";
    out += String(s_commands_taken);
    out += ",\"log\":\"";
    for (uint32_t i = 0; i < count; ++i) {
        char c = slice[i];
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c == '\n') out += "\\n";
        else if ((uint8_t)c >= 0x20) out += c; // Other control characters are dropped
    }
    out += "\",\"status\":";
    out += s_status_provider ? s_status_provider() : String("{}");
    out += "}";
    s_web_server.sendHeader("Cache-Control", "no-store");
    s_web_server.send(200, "application/json", out);
}

// GET /cmd?c=text  ->  hands the text to the debug console, as if it had been typed over Telnet
static void handleAppCommand() {
    String command = s_web_server.arg("c");
    command.trim();
    // (the limit was 64, which silently dropped any "fail <what happened>" longer than a few words)
    if (command.length() > 0 && command.length() < 300 && !s_telnet_cmd_ready) {
        s_telnet_rx_buf = command;
        s_telnet_cmd_ready = true;
    }
    s_web_server.send(200, "text/plain", "ok");
}

// GET /note?t=text  ->  printed as an [APP] line and nothing else. The page reports what was done
// on it that sends no command (page opened, slider moved, a box opened or closed), so that a
// recording of the robot's output shows the order things were pressed in. It does not go through
// the console, so a note can never get in the way of a command such as STOP.
static void handleAppNote() {
    String note = s_web_server.arg("t");
    note.trim();
    if (note.length() > 0 && note.length() < 200) {
        note.replace('\n', ' ');
        note.replace('\r', ' ');
        Serial.printf("[APP] %s\n", note.c_str());
    }
    s_web_server.send(200, "text/plain", "ok");
}

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
        s_web_server.send_P(200, "text/html", APP_PAGE_HTML);
    });
    s_web_server.on("/data", HTTP_GET, handleAppData);
    s_web_server.on("/cmd", HTTP_GET, handleAppCommand);
    s_web_server.on("/note", HTTP_GET, handleAppNote);

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
    Serial.println("[WEB] Phone app at http://192.168.4.1  (firmware upload at http://192.168.4.1/update)");

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
    // (not while a command is waiting to be collected: the navigation task is reading the buffer)
    if (!s_telnet_cmd_ready && s_telnet_client && s_telnet_client.connected() && s_telnet_client.available()) {
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

void WifiOTA::commandDone() {
    s_commands_taken++;
}

IPAddress WifiOTA::getIP() {
#if WIFI_AP_MODE
    return WiFi.softAPIP();
#else
    return (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : WiFi.softAPIP();
#endif
}

#endif // ENABLE_WIFI_OTA
