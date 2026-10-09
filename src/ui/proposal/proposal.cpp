#include "config.h"
#include "ui/proposal/proposal.h"

// ==============================================================================
// PROPOSAL
// ==============================================================================

namespace Proposal {

static char s_text[701] = "";   // The question being put together or on show
static uint16_t s_count = 0;    // Questions shown since power-on
static bool s_shown = false;

void startNew() {
    s_text[0] = '\0';
    s_shown = false;
}

void append(const char* text) {
    size_t used = strlen(s_text);
    if (used > 0 && used < sizeof(s_text) - 1) s_text[used++] = ' '; // The pieces were cut at spaces
    for (; *text != '\0' && used < sizeof(s_text) - 1; ++text) {
        // The text travels inside the app's status reply, so keep it to what is safe there
        const char c = *text;
        s_text[used++] = (c == '"' || c == '\\' || c < ' ') ? '\'' : c;
    }
    s_text[used] = '\0';
}

void show() {
    if (s_text[0] == '\0') return;
    s_shown = true;
    s_count++;
    Serial.printf("[ASK] #%u on the app: %s\n", (unsigned int)s_count, s_text);
}

void clear() {
    s_shown = false;
    s_text[0] = '\0';
}

uint16_t shownId() {
    return s_shown ? s_count : 0;
}

const char* text() {
    return s_text;
}

bool answer(bool yes) {
    if (!s_shown) return false;
    Serial.printf("[ANSWER] #%u %s | %.80s\n", (unsigned int)s_count, yes ? "YES" : "NO", s_text);
    clear();
    return true;
}

} // namespace Proposal
