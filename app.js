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