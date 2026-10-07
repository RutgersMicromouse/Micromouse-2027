// =============================================================
// ANTIGRAV-MOUSE WEB BLUETOOTH DASHBOARD
// =============================================================


// =============================================================
// BLE UUIDs
// =============================================================

// Nordic UART Service
const UART_SERVICE =
  "6e400001-b5a3-f393-e0a9-e50e24dcca9e";

const UART_RX =
  "6e400002-b5a3-f393-e0a9-e50e24dcca9e";


// Custom telemetry service
const TELEMETRY_SERVICE =
  "7a100001-0000-4a5b-8c9d-123456789abc";

const BATTERY =
  "7a100002-0000-4a5b-8c9d-123456789abc";

const HEADING =
  "7a100003-0000-4a5b-8c9d-123456789abc";

const LEFT_ENCODER =
  "7a100004-0000-4a5b-8c9d-123456789abc";

const RIGHT_ENCODER =
  "7a100005-0000-4a5b-8c9d-123456789abc";


// =============================================================
// BLE OBJECTS
// =============================================================

let device = null;
let server = null;
let telemetry = null;
let rx = null;


// Store telemetry characteristics here
const chars = {};


// =============================================================
// TEXT ENCODERS / DECODERS
// =============================================================

const decoder = new TextDecoder();
const encoder = new TextEncoder();


// =============================================================
// AUTO REFRESH SETTINGS
// =============================================================

// How often we want to update the dashboard.
//
// 250 ms = 4 updates per second
const REFRESH_INTERVAL_MS = 250;


// Prevent multiple BLE reads from happening at the same time.
let isRefreshing = false;


// Timer used for automatic refresh
let refreshTimer = null;


// =============================================================
// HTML HELPER
// =============================================================

const $ = (id) => {

  return document.getElementById(id);
};


// =============================================================
// MESSAGE
// =============================================================

function msg(text) {

  $("message").textContent = text;
}


// =============================================================
// CONNECTION DISPLAY
// =============================================================

function setConnected(connected) {

  const status = $("status");


  if (connected) {

    status.className =
      "status online";

    status.innerHTML =
      "<span></span>Connected";


    $("connectBtn").textContent =
      "Disconnect";


    $("refreshBtn").disabled =
      false;


    $("resetHeading").disabled =
      false;


    $("resetEncoders").disabled =
      false;

  } else {

    status.className =
      "status offline";

    status.innerHTML =
      "<span></span>Disconnected";


    $("connectBtn").textContent =
      "Connect to Mouse";


    $("refreshBtn").disabled =
      true;


    $("resetHeading").disabled =
      true;


    $("resetEncoders").disabled =
      true;
  }
}


// =============================================================
// CONVERT BLE DATA TO TEXT
// =============================================================

function parseValue(view) {

  return decoder
    .decode(
      view.buffer.slice(
        view.byteOffset,
        view.byteOffset + view.byteLength
      )
    )
    .trim();
}


// =============================================================
// READ ONE CHARACTERISTIC
// =============================================================

async function readOne(
  key,
  elementID,
  textToRemove = null
) {

  // Make sure characteristic exists
  if (!chars[key]) {

    return;
  }


  // Ask ESP32 for latest value
  const data =
    await chars[key].readValue();


  // Convert BLE bytes into text
  let value =
    parseValue(data);


  // Remove units because HTML already displays them.
  //
  // Example:
  //
  // "3.77 V" -> "3.77"
  // "-44.5 deg" -> "-44.5"

  if (textToRemove) {

    value =
      value
        .replace(textToRemove, "")
        .trim();
  }


  // Update dashboard
  $(elementID).textContent =
    value;
}


// =============================================================
// REFRESH ALL TELEMETRY
// =============================================================

async function refresh() {

  // Don't start another refresh if one is still running
  if (isRefreshing) {

    return;
  }


  // Don't attempt BLE operations when disconnected
  if (
    !device ||
    !device.gatt ||
    !device.gatt.connected
  ) {

    return;
  }


  isRefreshing = true;


  try {

    // IMPORTANT:
    //
    // Read ONE characteristic at a time.
    //
    // Doing four Web Bluetooth GATT operations simultaneously
    // can cause problems.


    // ---------------------------------------------------------
    // Battery
    // ---------------------------------------------------------

    await readOne(
      "battery",
      "battery",
      "V"
    );


    // ---------------------------------------------------------
    // Heading
    // ---------------------------------------------------------

    await readOne(
      "heading",
      "heading",
      "deg"
    );


    // ---------------------------------------------------------
    // Left Encoder
    // ---------------------------------------------------------

    await readOne(
      "left",
      "left"
    );


    // ---------------------------------------------------------
    // Right Encoder
    // ---------------------------------------------------------

    await readOne(
      "right",
      "right"
    );


  } catch (error) {

    console.error(
      "Telemetry refresh failed:",
      error
    );

  } finally {

    isRefreshing = false;
  }
}


// =============================================================
// START AUTO REFRESH
// =============================================================

function startAutoRefresh() {

  // Make sure we don't accidentally create two timers
  stopAutoRefresh();


  console.log(
    "Starting automatic telemetry refresh"
  );


  // Refresh immediately
  refresh();


  // Then continue refreshing
  refreshTimer =
    setInterval(
      refresh,
      REFRESH_INTERVAL_MS
    );
}


// =============================================================
// STOP AUTO REFRESH
// =============================================================

function stopAutoRefresh() {

  if (refreshTimer !== null) {

    clearInterval(
      refreshTimer
    );


    refreshTimer = null;
  }
}


// =============================================================
// DISCONNECTED EVENT
// =============================================================

function onDisconnected() {

  console.log(
    "Antigrav-Mouse disconnected"
  );


  stopAutoRefresh();


  setConnected(
    false
  );


  msg(
    "Mouse disconnected."
  );


  // Reset displayed values

  $("battery").textContent =
    "--";

  $("heading").textContent =
    "--";

  $("left").textContent =
    "--";

  $("right").textContent =
    "--";
}


// =============================================================
// CONNECT / DISCONNECT
// =============================================================

async function connect() {

  // -----------------------------------------------------------
  // Check Web Bluetooth support
  // -----------------------------------------------------------

  if (!navigator.bluetooth) {

    msg(
      "Web Bluetooth is not available in this browser. Use Google Chrome."
    );

    return;
  }


  // -----------------------------------------------------------
  // If already connected, disconnect
  // -----------------------------------------------------------

  if (
    device &&
    device.gatt &&
    device.gatt.connected
  ) {

    stopAutoRefresh();


    device.gatt.disconnect();


    return;
  }


  try {

    msg(
      "Looking for Antigrav-Mouse..."
    );


    // =========================================================
    // ASK USER TO SELECT MOUSE
    // =========================================================

    device =
      await navigator.bluetooth.requestDevice({

        filters: [

          {
            name: "Antigrav-Mouse"
          }

        ],

        optionalServices: [

          TELEMETRY_SERVICE,

          UART_SERVICE

        ]

      });


    // =========================================================
    // DISCONNECT EVENT
    // =========================================================

    device.addEventListener(
      "gattserverdisconnected",
      onDisconnected
    );


    msg(
      "Connecting..."
    );


    // =========================================================
    // CONNECT TO ESP32
    // =========================================================

    server =
      await device.gatt.connect();


    // =========================================================
    // GET TELEMETRY SERVICE
    // =========================================================

    telemetry =
      await server.getPrimaryService(
        TELEMETRY_SERVICE
      );


    // =========================================================
    // GET BATTERY CHARACTERISTIC
    // =========================================================

    chars.battery =
      await telemetry.getCharacteristic(
        BATTERY
      );


    // =========================================================
    // GET HEADING CHARACTERISTIC
    // =========================================================

    chars.heading =
      await telemetry.getCharacteristic(
        HEADING
      );


    // =========================================================
    // GET LEFT ENCODER CHARACTERISTIC
    // =========================================================

    chars.left =
      await telemetry.getCharacteristic(
        LEFT_ENCODER
      );


    // =========================================================
    // GET RIGHT ENCODER CHARACTERISTIC
    // =========================================================

    chars.right =
      await telemetry.getCharacteristic(
        RIGHT_ENCODER
      );


    // =========================================================
    // GET UART RX CHARACTERISTIC
    //
    // Used for:
    //
    // resetheading
    // resetenc
    // =========================================================

    try {

      const uart =
        await server.getPrimaryService(
          UART_SERVICE
        );


      rx =
        await uart.getCharacteristic(
          UART_RX
        );

      // Robot output (everything it prints) arrives here
      await attachBluetoothLog(uart);


    } catch (error) {

      console.warn(
        "UART command service unavailable:",
        error
      );


      rx = null;
    }


    // =========================================================
    // CONNECTED
    // =========================================================

    setConnected(
      true
    );


    msg(
      "Live telemetry connected."
    );


    // =========================================================
    // START AUTOMATIC REFRESH
    // =========================================================

    startAutoRefresh();


  } catch (error) {

    console.error(
      "Connection failed:",
      error
    );


    msg(
      "Connection failed: " +
      error.message
    );


    stopAutoRefresh();


    setConnected(
      false
    );
  }
}


// =============================================================
// SEND COMMAND TO ESP32
// =============================================================

async function sendCommand(
  command
) {

  // Make sure UART RX exists
  if (!rx) {

    msg(
      "Command channel is unavailable."
    );

    return;
  }


  try {

    // Temporarily pause reads while writing a command.
    stopAutoRefresh();


    const commandData =
      encoder.encode(
        command + "\n"
      );


    await rx.writeValue(
      commandData
    );


    console.log(
      "Command sent:",
      command
    );


    msg(
      'Sent "' +
      command +
      '" to the mouse.'
    );


    // Give ESP32 a moment to process reset
    await new Promise(
      resolve =>
        setTimeout(
          resolve,
          150
        )
    );


    // Read the new values
    await refresh();


    // Resume live telemetry
    startAutoRefresh();


  } catch (error) {

    console.error(
      "Command failed:",
      error
    );


    msg(
      "Command failed: " +
      error.message
    );


    startAutoRefresh();
  }
}


// =============================================================
// BUTTON EVENTS
// =============================================================


// Connect / Disconnect
$("connectBtn").addEventListener(

  "click",

  connect

);


// Manual refresh still works if you want it
$("refreshBtn").addEventListener(

  "click",

  refresh

);


// Reset heading
$("resetHeading").addEventListener(

  "click",

  () => {

    sendCommand(
      "resetheading"
    );
  }

);


// Reset encoders
$("resetEncoders").addEventListener(

  "click",

  () => {

    sendCommand(
      "resetenc"
    );
  }

);


// =============================================================
// INITIAL PAGE STATE
// =============================================================

setConnected(
  false
);


msg(
  "Connect to Antigrav-Mouse to begin live telemetry."
);


// =============================================================
// ROBOT OUTPUT PANEL
//
// Shows everything the robot prints, exactly as the serial
// monitor would, and sends typed commands to it.
//
// The text can arrive two ways:
//   Bluetooth - the robot copies its output to the UART TX
//               characteristic (needs no cable)
//   USB       - read straight from the serial port (close the
//               serial monitor first: only one program can
//               have the port)
// =============================================================

const UART_TX =
  "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

// Keep the panel from growing without limit
const MAX_LOG_CHARS = 60000;

let usbPort = null;
let usbWriter = null;

function appendLog(text) {
  const log = $("log");
  if (log.dataset.started !== "yes") {
    log.textContent = "";
    log.dataset.started = "yes";
  }
  // Stay scrolled to the bottom unless the user has scrolled up to read
  const atBottom =
    log.scrollHeight - log.scrollTop - log.clientHeight < 40;
  log.textContent += text.replace(/\r/g, "");
  if (log.textContent.length > MAX_LOG_CHARS) {
    log.textContent = log.textContent.slice(-MAX_LOG_CHARS);
  }
  if (atBottom) {
    log.scrollTop = log.scrollHeight;
  }
}

// ---------------------------------------------------------
// Bluetooth: listen to the robot's output
// ---------------------------------------------------------
async function attachBluetoothLog(uart) {
  try {
    const tx = await uart.getCharacteristic(UART_TX);
    const textDecoder = new TextDecoder();
    tx.addEventListener("characteristicvaluechanged", (event) => {
      handleRobotText(textDecoder.decode(event.target.value, { stream: true }));
    });
    await tx.startNotifications();
    appendLog("[dashboard] Listening to the robot over Bluetooth.\n");
    startSensorStream();
  } catch (error) {
    console.warn("Robot output over Bluetooth unavailable:", error);
    appendLog("[dashboard] Could not listen over Bluetooth: " + error.message + "\n");
  }
}

// ---------------------------------------------------------
// USB: read the serial port directly
// ---------------------------------------------------------
async function connectUsb() {
  if (!navigator.serial) {
    msg("This browser cannot open serial ports. Use Google Chrome or Edge.");
    return;
  }
  if (usbPort) {
    msg("USB is already connected.");
    return;
  }
  try {
    usbPort = await navigator.serial.requestPort();
    await usbPort.open({ baudRate: 115200 });
    // Leave the reset lines alone so opening the port does not restart the robot
    try {
      await usbPort.setSignals({ dataTerminalReady: false, requestToSend: false });
    } catch (error) {
      console.warn("Could not set serial signals:", error);
    }
    usbWriter = usbPort.writable.getWriter();
    $("usbBtn").textContent = "USB connected";
    $("usbBtn").disabled = true;
    appendLog("[dashboard] Listening to the robot over USB.\n");
    msg("USB connected. The robot's output appears below.");
    startSensorStream();

    const reader = usbPort.readable.getReader();
    const textDecoder = new TextDecoder();
    try {
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        handleRobotText(textDecoder.decode(value, { stream: true }));
      }
    } finally {
      reader.releaseLock();
    }
  } catch (error) {
    console.error("USB connection failed:", error);
    msg("USB connection failed: " + error.message +
        " (is the serial monitor still open? Only one program can use the port.)");
    usbPort = null;
    usbWriter = null;
    $("usbBtn").textContent = "Connect by USB";
    $("usbBtn").disabled = false;
  }
}

// ---------------------------------------------------------
// Send a typed command by whichever link is connected
// ---------------------------------------------------------
async function sendTypedCommand(command) {
  if (usbWriter) {
    await usbWriter.write(encoder.encode(command + "\n"));
    return;
  }
  if (rx) {
    await rx.writeValue(encoder.encode(command + "\n"));
    return;
  }
  msg("Not connected. Use Connect to Mouse (Bluetooth) or Connect by USB first.");
}

$("usbBtn").addEventListener("click", connectUsb);

$("clearLog").addEventListener("click", () => {
  $("log").textContent = "";
  $("log").dataset.started = "yes";
});

$("commandForm").addEventListener("submit", async (event) => {
  event.preventDefault();
  const input = $("commandInput");
  const command = input.value.trim();
  if (!command) return;
  input.value = "";
  appendLog("> " + command + "\n");
  try {
    await sendTypedCommand(command);
  } catch (error) {
    msg("Command failed: " + error.message);
  }
});


// =============================================================
// LIVE SENSOR READINGS
//
// The robot prints a line starting "[TEL]" five times a second
// once it has been sent the command "stream on". Those lines are
// taken out of the output panel and shown as the bars instead.
// =============================================================

// Reading that fills a bar completely: the top of the sensors' range
const SENSOR_FULL_SCALE = 4095;

let robotTextBuffer = "";
let lastSensorTime = 0;

function startSensorStream() {
  // Give the link a moment, then ask the robot to start sending readings
  setTimeout(() => {
    sendTypedCommand("stream on").catch((error) => console.warn(error));
  }, 600);
}

function showSensors(line) {
  const ir = line.match(
    /L90=\s*(\d+)\s+L45=\s*(\d+)\s+FL=\s*(\d+)\s+FR=\s*(\d+)\s+R45=\s*(\d+)\s+R90=\s*(\d+)/
  );
  if (ir) {
    for (let i = 0; i < 6; i++) {
      const value = Number(ir[i + 1]);
      $("ir" + i).textContent = value;
      $("bar" + i).style.height =
        Math.min(100, (value / SENSOR_FULL_SCALE) * 100) + "%";
    }
    lastSensorTime = Date.now();
  }
  const walls = line.match(/Walls: \[(.)(.)(.)\]/);
  if (walls) {
    $("walls").textContent =
      (walls[1] === "L" ? "LEFT " : "- ") +
      (walls[2] === "F" ? "FRONT " : "- ") +
      (walls[3] === "R" ? "RIGHT" : "-");
  }
  const loop = line.match(/Loop:\s*(\d+)us/);
  if (loop) {
    $("loopTime").textContent = loop[1];
  }
}

// Splits incoming text into whole lines; sensor lines feed the bars, the rest goes to the panel
function handleRobotText(text) {
  robotTextBuffer += text.replace(/\r/g, "");
  let newline;
  while ((newline = robotTextBuffer.indexOf("\n")) >= 0) {
    const line = robotTextBuffer.slice(0, newline);
    robotTextBuffer = robotTextBuffer.slice(newline + 1);
    if (line.startsWith("[TEL]")) {
      showSensors(line);
    } else {
      appendLog(line + "\n");
    }
  }
}

// Say how fresh the readings are, so stale numbers are not mistaken for live ones
setInterval(() => {
  if (!lastSensorTime) return;
  const seconds = (Date.now() - lastSensorTime) / 1000;
  $("sensorAge").textContent =
    seconds < 1.5 ? "(live)" : "(last update " + seconds.toFixed(0) + " s ago)";
}, 500);

