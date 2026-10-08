#include "config.h"

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
