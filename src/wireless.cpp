#include "wireless.h"

// ==============================================================================
// DEBUG LOG (see config.h section 8)
// ==============================================================================

DebugLog g_debug_log;

static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;
static char   s_log_ring[DEBUG_LOG_BUFFER_BYTES];
static size_t s_log_start = 0; // Index of the oldest byte not yet sent
static size_t s_log_count = 0; // Bytes waiting

size_t DebugLog::write(const uint8_t* data, size_t length) {
    usbSerial().write(data, length);

#if ENABLE_BLE_DEBUG || ENABLE_WIFI_OTA
    portENTER_CRITICAL(&s_log_mux);
    for (size_t i = 0; i < length; ++i) {
        if (s_log_count == DEBUG_LOG_BUFFER_BYTES) { // Full: drop the oldest byte
            s_log_start = (s_log_start + 1) % DEBUG_LOG_BUFFER_BYTES;
            s_log_count--;
        }
        s_log_ring[(s_log_start + s_log_count) % DEBUG_LOG_BUFFER_BYTES] = (char)data[i];
        s_log_count++;
    }
    portEXIT_CRITICAL(&s_log_mux);
#endif
    return length;
}

size_t DebugLog::drain(char* out, size_t max_length) {
    portENTER_CRITICAL(&s_log_mux);
    size_t n = (s_log_count < max_length) ? s_log_count : max_length;
    for (size_t i = 0; i < n; ++i) {
        out[i] = s_log_ring[(s_log_start + i) % DEBUG_LOG_BUFFER_BYTES];
    }
    s_log_start = (s_log_start + n) % DEBUG_LOG_BUFFER_BYTES;
    s_log_count -= n;
    portEXIT_CRITICAL(&s_log_mux);
    return n;
}

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


// The phone app: one page served by the robot itself at http://192.168.4.1
// (join the robot's Wi-Fi first). It asks /data four times a second and sends button presses to /cmd.
static const char APP_PAGE_HTML[] PROGMEM = R"PAGE(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-title" content="Antigrav">
<meta name="theme-color" content="#0b0d10">
<title>Antigrav-Mouse</title>
<style>
*{box-sizing:border-box}
body{margin:0;padding:16px;padding-top:max(16px,env(safe-area-inset-top));background:#0b0d10;color:#f5f7fa;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
h1{font-size:24px;margin:0}
.top{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#6d7480;margin-right:6px}
.live .dot{background:#52d273;box-shadow:0 0 12px #52d273}
.dim{color:#8f9aaa;font-size:13px}
.card{background:#171c24;border:1px solid #2c3440;border-radius:16px;padding:14px;margin-bottom:12px}
#state{font-size:17px;font-weight:700;margin:2px 0 0}
.row{display:grid;grid-template-columns:1fr 1fr;gap:10px}
button{appearance:none;border:0;border-radius:14px;padding:16px 10px;font-size:17px;font-weight:700;color:#0b0d10;background:#f5f7fa;width:100%}
button:active{opacity:.6}
#start{background:#52d273;font-size:22px;padding:22px 10px}
#stop{background:#ff5d5d;color:#fff;font-size:22px;padding:22px 10px}
button.plain{background:#232a34;color:#eef2f7;border:1px solid #303844;font-size:15px;padding:13px 8px}
.bars{display:grid;grid-template-columns:repeat(6,1fr);gap:6px}
.bar{display:flex;flex-direction:column;align-items:center;gap:4px}
.bar b{font:600 13px ui-monospace,Consolas,monospace}
.bar span{color:#8f9aaa;font-size:11px}
.track{width:100%;height:110px;background:#07090c;border:1px solid #232a34;border-radius:8px;display:flex;align-items:flex-end;overflow:hidden}
.fill{width:100%;height:0;background:linear-gradient(#7cf29a,#2f9e52);transition:height .15s linear}
.vals{display:grid;grid-template-columns:1fr 1fr;gap:6px 14px;font-size:14px}
.vals b{font-family:ui-monospace,Consolas,monospace}
#log{margin:0;height:220px;overflow:auto;background:#07090c;border:1px solid #232a34;border-radius:10px;padding:10px;font:12px/1.4 ui-monospace,Consolas,monospace;color:#cfe3d4;white-space:pre-wrap;word-break:break-word}
form{display:flex;gap:8px;margin-top:10px}
input{flex:1;min-width:0;background:#07090c;border:1px solid #303844;border-radius:12px;padding:12px;color:#f5f7fa;font:14px ui-monospace,Consolas,monospace}
form button{width:auto;padding:12px 18px}
a{color:#7fb6ff}
</style></head><body>
<div class="top"><h1>Antigrav-Mouse</h1><div id="link" class="dim"><span class="dot"></span><span id="linkText">connecting</span></div></div>

<div class="card"><div class="dim">The robot is</div><div id="state">...</div><div class="dim" id="mode"></div>
<div style="margin-top:8px">It thinks it is in cell <b id="cell">--</b></div><div class="dim">Cells explored so far: <b id="visited">--</b> &nbsp;(the start cell is (0, 0); first number counts right, second counts forward)</div></div>

<div class="row" style="margin-bottom:12px">
  <button id="start">START</button>
  <button id="stop">STOP</button>
</div>
<div class="row" style="margin-bottom:12px">
  <button class="plain" data-cmd="speedrun">Speed run</button>
  <button class="plain" data-cmd="calib">Calibrate sensors</button>
  <button class="plain" data-cmd="clear">Forget the maze</button>
  <button class="plain" data-cmd="health">Health check</button>
  <button class="plain" data-cmd="resetall">Reset sensors</button>
  <button class="plain" data-cmd="ir">Raw IR readings</button>
</div>

<div class="card">
  <div class="top" style="margin-bottom:8px"><span class="dim">IR wall sensors</span><span class="dim">Walls: <b id="walls">- - -</b></span></div>
  <div class="bars">
    <div class="bar"><div class="track"><div class="fill" id="f0"></div></div><b id="v0">--</b><span>L 90</span></div>
    <div class="bar"><div class="track"><div class="fill" id="f1"></div></div><b id="v1">--</b><span>L 45</span></div>
    <div class="bar"><div class="track"><div class="fill" id="f2"></div></div><b id="v2">--</b><span>Front L</span></div>
    <div class="bar"><div class="track"><div class="fill" id="f3"></div></div><b id="v3">--</b><span>Front R</span></div>
    <div class="bar"><div class="track"><div class="fill" id="f4"></div></div><b id="v4">--</b><span>R 45</span></div>
    <div class="bar"><div class="track"><div class="fill" id="f5"></div></div><b id="v5">--</b><span>R 90</span></div>
  </div>
</div>

<div class="card"><div class="vals">
  <div>Heading <b id="heading">--</b>&deg;</div><div>Battery <b id="vbat">--</b> V</div>
  <div>Left wheel <b id="encL">--</b></div><div>Right wheel <b id="encR">--</b></div>
  <div>Motor driver <b id="motor">--</b></div><div>IMU <b id="imu">--</b></div>
  <div>Motor supply <b id="supply">--</b> V</div><div>Loop <b id="loop">--</b> &micro;s</div>
</div></div>

<div class="card">
  <div class="top" style="margin-bottom:8px"><span class="dim">Robot output (same as the serial monitor)</span><button class="plain" id="clear" style="width:auto;padding:6px 12px;font-size:13px">Clear</button></div>
  <pre id="log"></pre>
  <form id="form"><input id="cmd" autocomplete="off" autocapitalize="off" spellcheck="false" placeholder="command, e.g. status"><button type="submit">Send</button></form>
</div>
<p class="dim" style="text-align:center"><a href="/update">Upload new firmware</a></p>

<script>
const $ = id => document.getElementById(id);
let next = 0, misses = 0;

function send(command) {
  $("log").textContent += "> " + command + "\n";
  $("log").scrollTop = $("log").scrollHeight;
  fetch("/cmd?c=" + encodeURIComponent(command)).catch(() => {});
}

$("start").onclick = () => send("start");
$("stop").onclick = () => send("stop");
document.querySelectorAll("button[data-cmd]").forEach(b => b.onclick = () => send(b.dataset.cmd));
$("clear").onclick = () => { $("log").textContent = ""; };
$("form").onsubmit = e => {
  e.preventDefault();
  const c = $("cmd").value.trim();
  if (c) send(c);
  $("cmd").value = "";
};

function show(d) {
  const s = d.status || {};
  $("state").textContent = s.state || "...";
  $("mode").textContent = s.mode ? "Mode: " + s.mode + (s.tier ? ", speed tier " + s.tier : "") : "";
  if (s.ir) for (let i = 0; i < 6; i++) {
    $("v" + i).textContent = s.ir[i];
    $("f" + i).style.height = Math.min(100, s.ir[i] / 4095 * 100) + "%";
  }
  if (s.walls) $("walls").textContent =
    (s.walls[0] === "L" ? "LEFT " : "- ") + (s.walls[1] === "F" ? "FRONT " : "- ") + (s.walls[2] === "R" ? "RIGHT" : "-");
  for (const k of ["heading", "vbat", "encL", "encR", "motor", "imu", "supply", "loop", "cell", "visited"])
    if (s[k] !== undefined) $(k).textContent = s[k];
  if (d.log) {
    const log = $("log");
    const atBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 40;
    log.textContent = (log.textContent + d.log).slice(-30000);
    if (atBottom) log.scrollTop = log.scrollHeight;
  }
  next = d.next;
}

async function poll() {
  try {
    const r = await fetch("/data?since=" + next, { cache: "no-store" });
    show(await r.json());
    misses = 0;
  } catch (e) {
    misses++;
  }
  $("link").className = misses < 4 ? "dim live" : "dim";
  $("linkText").textContent = misses < 4 ? "live" : "no answer from the robot";
  setTimeout(poll, 250);
}
poll();
</script>
</body></html>)PAGE";

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

// GET /data?since=N  ->  {"next":M,"log":"...new output...","status":{...}}
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
    out.reserve(count + 600);
    out += "{\"next\":";
    out += String(since + count);
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
    if (command.length() > 0 && command.length() < 64 && !s_telnet_cmd_ready) {
        s_telnet_rx_buf = command;
        s_telnet_cmd_ready = true;
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
