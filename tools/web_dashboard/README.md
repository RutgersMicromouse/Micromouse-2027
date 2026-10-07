# Antigrav-Mouse Web Telemetry

A simple browser dashboard for the custom BLE telemetry service already added to the ESP32 firmware.

## What it displays
- Battery voltage
- Heading
- Left encoder ticks
- Right encoder ticks

It also sends the existing Nordic UART commands `resetheading` and `resetenc`.

## Run locally on your Mac
From this folder:

    python3 -m http.server 8000

Then open:

    http://localhost:8000

in a Web Bluetooth-capable desktop browser.

## Important iPhone limitation
Standard iPhone Safari and Chrome do not expose the standard Web Bluetooth API used by this page. If your goal is specifically "open a normal URL in Safari on my iPhone", the better architecture is to have the ESP32 serve this dashboard over its existing Wi-Fi access point instead of connecting from JavaScript over BLE.

The current page is useful for testing the BLE service on browsers/platforms that expose Web Bluetooth.
