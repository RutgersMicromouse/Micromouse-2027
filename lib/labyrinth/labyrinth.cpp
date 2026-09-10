#include "labyrinth.h"
#include "Robot.h"
#include "MazeLogic.h"
#include "tof.h"  // For your ToF sensor functions
#include "pidrotate.h"
#include "pidstraight.h"
#include "firefighter.h" // For CHECK_BEACON()

// Initialize the "Brain" with the maze dimension (e.g., 16x16)
Robot mouse(16);

void labyrinthLoop() {
    // Small delay to ensure sensors are ready
    delay(500);

    while(true) {
        // 1. COLLECT SENSORS
        // Convert distance readings to 1 (Path) or 0 (Wall)
        // Adjust the '100' threshold to your maze's cell size
        int sensors[3];
        sensors[0] = (left() > 150) ? 1 : 0;   // Left
        sensors[1] = (front() > 150) ? 1 : 0;  // Front
        sensors[2] = (right() > 150) ? 1 : 0;  // Right

        // 2. CHECK FOR FIRE BEACON
        bool beacon = CHECK_BEACON();

        // 3. THINK
        // next_move updates internal maps and returns {rotation, movement}
        std::pair<int, int> command = mouse.next_move(sensors, beacon);

        int turnAngle = command.first;
        int moveDistance = command.second;

        // 4. ACT: ROTATE
        if (turnAngle != 0) {
            turnTo((double)turnAngle);
            delay(100); // Settle time
        }

        // 5. ACT: MOVE
        if (moveDistance > 0) {
            // Move one cell length (usually 180mm for standard Micromouse)
            pidForward(180); 
        }

        // 6. MODE CHECK
        if (mouse.currentMode == Robot::SEARCH) {
            // Once the robot returns to (0,0) after finding the goal, 
            // it switches to SEARCH to calculate the BFS path.
            Serial.println("Exploration complete! Ready for Race Mode.");
            break; 
        }
    }
}