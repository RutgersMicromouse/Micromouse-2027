#ifndef ROBOT_H
#define ROBOT_H
#include <Arduino.h>
#include <vector>
#include <string>
#include <map>

struct Coord {
    int x, y;
    bool operator==(const Coord& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Coord& o) const { return !(*this == o); }
};

class Robot {
public:
    struct Cell {
        std::string previous = "";
        int value = 0; // 0: Unvisited, 1: Visited, 2: Double
    };

    enum Mode { EXPLORE, SEARCH, RACE };
    Mode currentMode = EXPLORE;

    Robot(int dim);
    std::pair<int, int> next_move(int sensors[3], bool beaconDetected = false);

private:
    int posX, posY;
    int lastX, lastY;
    std::string heading;
    bool isBeginning;
    bool isReversing;
    int mazeDim;
    bool goalFound;
    Coord goalCoords;

    std::vector<std::vector<int>>                        mazeMap;
    std::vector<std::vector<Cell>>                       pathMap;
    std::vector<std::vector<std::string>>                policyGrid;
    std::vector<std::vector<std::map<std::string, int>>> passageMap; // NEW

    void explore(int sensors[3]);
    void updateMap(int sensors[3]);
    void propagateWalls(int x, int y);                              // NEW
    void checkBeacon(bool detected);
    bool isUnvisited(std::string direction);
    int  calculateRotation(std::string current, std::string target);
    void findShortestPath();
    void blinkPosition(int posX, int posY);
};




#endif
