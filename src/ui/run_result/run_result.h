#pragma once

// ==============================================================================
// RUN RESULT
// ==============================================================================

// The operator's verdict on what the robot just did ("success" / "fail" on the console, the two
// buttons in the phone app). Prints [RESULT] lines with the verdict and every setting that was in
// force, so that a recording of the robot's output (tools/run_recorder, over Bluetooth) shows
// which settings went with runs that worked and which with runs that did not.
namespace RunResult {

// `note` is whatever the operator typed after the word, and may be empty
void report(bool success, const char* note);

} // namespace RunResult
