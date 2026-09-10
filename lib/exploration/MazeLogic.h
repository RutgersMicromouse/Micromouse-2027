#ifndef MAZELOGIC_H
#define MAZELOGIC_H
#include <Arduino.h>
#include <string>

class MazeLogic {
public:
    // Bits: 1=Up/North, 2=Right/East, 4=Down/South, 8=Left/West
    // A set bit means a WALL EXISTS in that direction.

    static bool isPermissible(int wallValue, std::string direction) {
        if (direction == "up")    return !(wallValue & 1); // passable if NO wall
        if (direction == "right") return !(wallValue & 2);
        if (direction == "down")  return !(wallValue & 4);
        if (direction == "left")  return !(wallValue & 8);
        return false;
    }

    static std::string getOpposite(std::string dir) {
        if (dir == "up")    return "down";
        if (dir == "down")  return "up";
        if (dir == "left")  return "right";
        if (dir == "right") return "left";
        return "up";
    }

    static int getWallBit(std::string heading, std::string side) {
        if (heading == "up") {
            if (side == "front") return 1;
            if (side == "right") return 2;
            if (side == "back")  return 4;
            if (side == "left")  return 8;
        } else if (heading == "right") {
            if (side == "front") return 2;
            if (side == "right") return 4;
            if (side == "back")  return 8;
            if (side == "left")  return 1;
        } else if (heading == "down") {
            if (side == "front") return 4;
            if (side == "right") return 8;
            if (side == "back")  return 1;
            if (side == "left")  return 2;
        } else if (heading == "left") {
            if (side == "front") return 8;
            if (side == "right") return 1;
            if (side == "back")  return 2;
            if (side == "left")  return 4;
        }
        return 0;
    }
};
#endif
