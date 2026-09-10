#include "Robot.h"
#include "MazeLogic.h"

static const int WALL_THRESHOLD = 100; // mm — tightened from 150

// ─── Constructor ──────────────────────────────────────────────────────────────
Robot::Robot(int dim) {
    mazeDim     = dim;
    posX        = 0;
    posY        = 0;
    lastX       = 0;
    lastY       = 0;
    heading     = "up";
    isBeginning = true;
    isReversing = false;
    goalFound   = false;
    goalCoords  = {-1, -1};
    currentMode = EXPLORE;

    mazeMap   .assign(dim, std::vector<int>(dim, 0));
    pathMap   .assign(dim, std::vector<Cell>(dim));
    policyGrid.assign(dim, std::vector<std::string>(dim, ""));
    passageMap.assign(dim, std::vector<std::map<std::string,int>>(dim));
}

// ─── Wall Mapping ─────────────────────────────────────────────────────────────
void Robot::updateMap(int sensors[3]) {
    int& cell = mazeMap[posX][posY];

    Serial.println("=== updateMap ===");
    Serial.print("  pos: ("); Serial.print(posX); Serial.print(","); Serial.print(posY); Serial.println(")");
    blinkPosition(posX, posY);
    Serial.print("  heading: "); Serial.println(heading.c_str());
    Serial.print("  sensors L|F|R: ");
    Serial.print(sensors[0]); Serial.print(" | ");
    Serial.print(sensors[1]); Serial.print(" | ");
    Serial.println(sensors[2]);

    if (sensors[0] > 0 && sensors[0] < WALL_THRESHOLD) {
        cell |= MazeLogic::getWallBit(heading, "left");
        Serial.println("  -> wall LEFT");
    }
    if (sensors[1] > 0 && sensors[1] < WALL_THRESHOLD) {
        cell |= MazeLogic::getWallBit(heading, "front");
        Serial.println("  -> wall FRONT");
    }
    if (sensors[2] > 0 && sensors[2] < WALL_THRESHOLD) {
        cell |= MazeLogic::getWallBit(heading, "right");
        Serial.println("  -> wall RIGHT");
    }

    Serial.print("  cell bits: "); Serial.println(cell, BIN);
    propagateWalls(posX, posY);
}

void Robot::propagateWalls(int x, int y) {
    const std::string dirs[] = {"up", "right", "down", "left"};
    const int dx[]           = {  0,    1,       0,     -1 };
    const int dy[]           = {  1,    0,      -1,      0 };

    for (int i = 0; i < 4; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (nx < 0 || nx >= mazeDim || ny < 0 || ny >= mazeDim) continue;

        int bit = MazeLogic::getWallBit(dirs[i], "front");
        if (mazeMap[x][y] & bit) {
            mazeMap[nx][ny] |= MazeLogic::getWallBit(
                MazeLogic::getOpposite(dirs[i]), "front");
        }
    }
}

// ─── Trémaux Exploration ──────────────────────────────────────────────────────
void Robot::explore(int sensors[3]) {
    updateMap(sensors);

    if (!isBeginning) {
        std::string arrivedFrom = MazeLogic::getOpposite(heading);
        passageMap[posX][posY][arrivedFrom]++;
    }

    if (pathMap[posX][posY].value == 0)
        pathMap[posX][posY].value = 1;

    const std::string dirs[] = {"up", "right", "down", "left"};
    std::string backDir = MazeLogic::getOpposite(heading);

    std::string bestDir   = "";
    int         bestCount = 9999;

    Serial.println("=== explore: evaluating directions ===");
    for (const std::string& dir : dirs) {
        bool passable = MazeLogic::isPermissible(mazeMap[posX][posY], dir);
        Serial.print("  dir: "); Serial.print(dir.c_str());
        Serial.print("  passable: "); Serial.print(passable);

        if (!passable) { Serial.println(" -> WALL"); continue; }

        int nx = posX, ny = posY;
        if      (dir == "up")    ny++;
        else if (dir == "right") nx++;
        else if (dir == "down")  ny--;
        else if (dir == "left")  nx--;

        if (nx < 0 || nx >= mazeDim || ny < 0 || ny >= mazeDim) {
            Serial.println(" -> OUT OF BOUNDS");
            continue;
        }

        int count = passageMap[posX][posY][dir];
        Serial.print("  count: "); Serial.println(count);

        if (count >= 2) continue;

        if (count == 0 && dir != backDir) {
            bestDir   = dir;
            bestCount = 0;
            break;
        }

        if (count < bestCount) {
            bestCount = count;
            bestDir   = dir;
        }
    }

    if (bestDir == "") {
        bestDir = backDir;
        pathMap[posX][posY].value = 2;
        Serial.println("  -> dead end, backtracking");
    }

    Serial.print("  CHOSEN: "); Serial.println(bestDir.c_str());

    passageMap[posX][posY][bestDir]++;

    int nx = posX, ny = posY;
    if      (bestDir == "up")    ny++;
    else if (bestDir == "right") nx++;
    else if (bestDir == "down")  ny--;
    else if (bestDir == "left")  nx--;

    if (nx < 0 || nx >= mazeDim || ny < 0 || ny >= mazeDim) {
        Serial.println("ERROR: explore() tried to step out of bounds!");
        return;
    }

    lastX   = posX;
    lastY   = posY;
    heading = bestDir;
    posX    = nx;
    posY    = ny;
}

// ─── Public Interface ─────────────────────────────────────────────────────────
std::pair<int, int> Robot::next_move(int sensors[3], bool beaconDetected) {
    checkBeacon(beaconDetected);

    std::string oldHeading = heading;
    int oldX = posX, oldY = posY;

    if (currentMode == EXPLORE) {
        explore(sensors);

        if (!isBeginning && posX == 0 && posY == 0) {
            currentMode = SEARCH;
            findShortestPath();
        }
        isBeginning = false;
    }

    int rotation  = calculateRotation(oldHeading, heading);
    int moveCells = (posX != oldX || posY != oldY) ? 1 : 0;

    Serial.print("next_move: rotation="); Serial.print(rotation);
    Serial.print(" moveCells="); Serial.println(moveCells);

    if (rotation == 180 || rotation == -180)
        return {rotation, moveCells};

    return {rotation, moveCells};
}

// ─── Helpers ──────────────────────────────────────────────────────────────────
void Robot::checkBeacon(bool detected) {
    if (detected && !goalFound) {
        goalFound  = true;
        goalCoords = {posX, posY};
    }
}

bool Robot::isUnvisited(std::string direction) {
    int nx = posX, ny = posY;
    if      (direction == "up")    ny++;
    else if (direction == "right") nx++;
    else if (direction == "down")  ny--;
    else if (direction == "left")  nx--;
    if (nx < 0 || nx >= mazeDim || ny < 0 || ny >= mazeDim) return false;
    return pathMap[nx][ny].value == 0;
}

int Robot::calculateRotation(std::string current, std::string target) {
    if (current == target) return 0;
    const std::string dirs[] = {"up", "right", "down", "left"};
    int curIdx = 0, tarIdx = 0;
    for (int i = 0; i < 4; i++) {
        if (dirs[i] == current) curIdx = i;
        if (dirs[i] == target)  tarIdx = i;
    }
    int diff = tarIdx - curIdx;
    if (diff ==  1 || diff == -3) return  90;
    if (diff == -1 || diff ==  3) return -90;
    return 180;
}

void Robot::findShortestPath() {
    // TODO: flood-fill or A*
}

void Robot::blinkPosition(int x, int y) {
    const int shortPulse = 200;
    const int longPulse = 1000;
    const int gap = 200;

    // Blink X (posX)
    // If posX is 0, we'll do one very fast flicker or stay dark
    for (int i = 0; i < x + 1; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(shortPulse);
        digitalWrite(LED_BUILTIN, LOW);  delay(gap);
    }

    // Long Light (Separator)
    delay(400); // Small pause before dash
    digitalWrite(LED_BUILTIN, HIGH); delay(longPulse);
    digitalWrite(LED_BUILTIN, LOW);  delay(800); // Pause before Y

    // Blink Y (posY)
    for (int i = 0; i < y + 1; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(shortPulse);
        digitalWrite(LED_BUILTIN, LOW);  delay(gap);
    }
}
