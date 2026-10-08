#include <functional>
#include "robot.h"
#include "ui/status_led/status_led.h"
#include "ui/actions/actions.h"
#include "ui/gestures/gestures.h"
#include "ui/console/console.h"
#include "wireless/ble_debug/ble_debug.h"
#include "wireless/wifi_ota/wifi_ota.h"

// ==============================================================================
// STATUS LED
// ==============================================================================

namespace StatusLED {

void begin() {
    pinMode(PIN_ESP32_RGB_LED, OUTPUT);
    set(OFF);
}

void set(Color color) {
    const uint8_t level = RGB_BRIGHTNESS_LEVEL;
    const uint8_t r = color.red   ? level : 0;
    const uint8_t g = color.green ? level : 0;
    const uint8_t b = color.blue  ? level : 0;
    neopixelWrite(PIN_ESP32_RGB_LED, r, g, b);
    neopixelWrite(PIN_ESP32_RGB_LED_ALT, r, g, b); // The same LED on the other board revision
}

void flash(Color color, int count, int delay_ms) {
    for (int i = 0; i < count; ++i) {
        set(color);
        delay(delay_ms);
        set(OFF);
        delay(delay_ms);
    }
}

} // namespace StatusLED
