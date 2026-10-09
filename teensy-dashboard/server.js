
const express = require("express");
const http = require("http");
const WebSocket = require("ws");
const { SerialPort } = require("serialport");

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

const HTTP_PORT = 3000;
const SERIAL_PORT = "/dev/cu.HC-05";
const BAUD_RATE = 9600;
const graphNodes = new Map();
const graphEdges = new Map();

function cacheGraphMessage(message) {
    const fields = message.split(",");

    if (fields[0] === "GRAPH_RESET" && fields.length === 1) {
        graphNodes.clear();
        graphEdges.clear();
    } else if (fields[0] === "GRAPH_NODE" && fields.length === 4) {
        const [id, x, y] = fields.slice(1).map(Number);
        if (Number.isInteger(id) && Number.isFinite(x) && Number.isFinite(y)) {
            graphNodes.set(id, message);
        }
    } else if (fields[0] === "GRAPH_EDGE" && (fields.length === 3 || fields.length === 4)) {
        const [from, to] = fields.slice(1, 3).map(Number);
        const cost = fields.length === 4 ? Number(fields[3]) : null;
        if (Number.isInteger(from) && Number.isInteger(to) &&
            (cost === null || Number.isFinite(cost))) {
            graphEdges.set(`${from},${to}`, message);
        }
    }
}

// Serve the frontend files from public/
app.use(express.static("public"));

// Start the website
server.listen(HTTP_PORT, () => {
    console.log(`Dashboard: http://localhost:${HTTP_PORT}`);
});

// Send messages to every connected browser
function broadcast(message) {
    for (const client of wss.clients) {
        if (client.readyState === WebSocket.OPEN) {
            client.send(message);
        }
    }
}

// Connect to the HC-05
const serial = new SerialPort({
    path: SERIAL_PORT,
    baudRate: BAUD_RATE,
    autoOpen: false
});

serial.open((err) => {
    if (err) {
        console.error("Could not open HC-05:", err.message);
        console.log("The website will still run without Bluetooth.");
        return;
    }

    console.log(`Connected to ${SERIAL_PORT} at ${BAUD_RATE} baud`);
    console.log("Waiting for Teensy data...");
});

// Forward received serial data to the website
let serialBuffer = "";

serial.on("data", (data) => {
    serialBuffer += data.toString();

    // Process only complete lines, delimited by '\n'
    const lines = serialBuffer.split("\n");

    // Keep the final, potentially incomplete line in the buffer
    serialBuffer = lines.pop();

    for (const line of lines) {
        const message = line.trim();

        if (message.length > 0) {
            cacheGraphMessage(message);
            //console.log("Teensy:", message);
            broadcast(message);
        }
    }
});

serial.on("error", (err) => {
    console.error("Serial error:", err.message);
});

// Show browser connections
wss.on("connection", (socket) => {
console.log("Dashboard browser connected.");

for (const message of graphNodes.values()) {
    socket.send(message);
}
for (const message of graphEdges.values()) {
    socket.send(message);
}

// Receive commands from the website and send them to the Teensy
socket.on("message", (data) => {
    const command = data.toString().trim();

    // Only allow this first test command
    const match = command.match(/^SET,(Kp_dist, Ki_dist, Kd_dist, Kp_angle, Ki_angle, Kd_angle, Kp_lat, Kd_lat, Kp_wall, Kd_wall, max_speed),(\d+(?:\.\d+)?)$/);

    if (!match) {
        socket.send("ERROR,Invalid command");
        return;
    }

    const value = Number(match[1]);

    if (!Number.isFinite(value) || value < 0 || value > 15) {
        socket.send("ERROR,K must be between 0 and 15");
        return;
    }

    if (!serial.isOpen) {
        socket.send("ERROR,Bluetooth serial is not connected");
        return;
    }

    serial.write(`${command}\n`, (err) => {
        if (err) {
            console.error("Command write failed:", err.message);
            socket.send("ERROR,Command write failed");
        }
    });
});

socket.on("close", () => {
    console.log("Dashboard browser disconnected.");
});


});


// Clean up on Ctrl+C
process.on("SIGINT", () => {
    console.log("\nShutting down...");

    if (serial.isOpen) {
        serial.close();
    }

    wss.close();
    server.close(() => process.exit(0));
});
