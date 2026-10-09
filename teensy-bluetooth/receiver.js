const { SerialPort } = require("serialport");

const port = new SerialPort({
path: "/dev/cu.HC-05",
baudRate: 9600,
autoOpen: false,
});

console.log("Connecting to HC-05...");

port.on("open", () => {
console.log("Serial port opened.");
console.log("Waiting for data...");
});

port.on("data", (data) => {
console.log("Received:", data.toString());
});

port.on("close", () => {
console.log("Serial port closed.");
});

port.on("error", (err) => {
console.error("Serial error:", err.message);
});

port.open((err) => {
if (err) {
console.error("Could not open serial port:", err.message);
}
});

process.on("SIGINT", () => {
console.log("\nStopping receiver...");

```
if (port.isOpen) {
    port.close((err) => {
        if (err) {
            console.error("Error closing serial port:", err.message);
        } else {
            console.log("Serial port closed cleanly.");
        }

        process.exit(0);
    });
} else {
    process.exit(0);
}
```

});
