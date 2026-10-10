#include <Preferences.h>
#include "robot.h"
#include "ui/stages/stages.h"

// ==============================================================================
// STAGES
// ==============================================================================

namespace Stages {

static const char* const kNames[COUNT] = {
    "dimensions of the robot",
    "encoder counts (10 turns of each wheel)",
    "basic movement (forward is forward, turns are 90 degrees the right way)",
    "search turns on the spot",
    "search straights",
    "speed-run turns",
    "speed-run straights",
};

static int current_ = 0;
static uint8_t done_ = 0;

static void save() {
    Preferences prefs;
    prefs.begin("stages", false);
    prefs.putInt("cur", current_);
    prefs.putUChar("done", done_);
    prefs.end();
}

void begin() {
    Preferences prefs;
    prefs.begin("stages", true);
    current_ = constrain(prefs.getInt("cur", 0), 0, COUNT - 1);
    done_ = prefs.getUChar("done", 0) & ((1 << COUNT) - 1);
    prefs.end();
}

int current() { return current_; }
uint8_t doneMask() { return done_; }

void report() {
    char finished[COUNT + 1];
    int n = 0;
    for (int i = 0; i < COUNT; ++i) {
        if (done_ & (1 << i)) finished[n++] = (char)('A' + i);
    }
    finished[n] = '\0';
    Serial.printf("[STAGE] Working on stage %c: %s. Finished: %s\n",
                  'A' + current_, kNames[current_], n ? finished : "none");
}

void goTo(int stage) {
    if (stage < 0 || stage >= COUNT) return;
    // Going back to a stage that was finished means it is not finished after all, and neither
    // is anything that was built on it
    if (done_ & (1 << stage)) {
        done_ &= (uint8_t)((1 << stage) - 1);
    }
    current_ = stage;
    save();
    report();
}

void markDone(int stage) {
    if (stage < 0 || stage >= COUNT) return;
    done_ |= (uint8_t)(1 << stage);
    Serial.printf("[STAGE] Stage %c marked FINISHED by the owner.\n", 'A' + stage);
    if (stage + 1 < COUNT) current_ = stage + 1;
    save();
    report();
}

} // namespace Stages
