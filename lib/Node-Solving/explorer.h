#pragma once
#include "types.h"
#include "graph.h"
#include <unordered_map>



const int CELL_SIZE_MM = 180;
const int WALL_THRESHOLD_MM = 120;
void smart_delay(int ms);

class Explorer {
public:
    MazeGraph& graph;
    RobotState  state;

    // Hand Passed Values
    bool isEncoder = false;


    // (x,y) -> node_id for loop detection
    std::unordered_map<int, int> positionMap; // key = x*100+y
    int beacon_node_id = 0; 
    int center_node_id = 0;
    Direction beacon_heading; 


    Explorer(MazeGraph& graph)
        : graph(graph), state({graph.addNode(0, 0), Direction::NORTH, 0, 0}) {
        // register start node position
        positionMap[0] = state.current_node_id;
    }


    void explore();


private:
    void exploreFromNode(int node_id, Direction heading_to_parent);


    // returns available exits based on sensors
    std::vector<Direction> readExits();


    // move through corridor until junction, build edge
    // returns node_id at the other end
    int traverseCorridor(Direction dir, std::vector<Instruction>& out_instructions, float& out_cost, bool& out_discoveredNewNode);


    // check if position is already a known node
    int positionKey(int x, int y) { return x * 100 + y; }
    bool isKnownPosition(int x, int y);

    // heading helpers
    void faceDirection(Direction d);
    void updatePosition(int steps);
    Direction oppositeDirection(Direction d);
    void executeInstructions(const std::vector<Instruction>& instructions);
    int directionToDegrees(Direction d);

    //checking if beacon is there
    void tripBeacon(bool& out_isTripped, Direction& out_facing);

    // solving the fastest
    std::vector<int> fastestRunAStar(int start_node, int goal_node);
    std::vector<Instruction> buildFastestInstructions(const std::vector<int>& path);
    std::vector<Instruction> getInstructionsBetween(int from_node, int to_node, int& heading);
    std::vector<Instruction> condenseInstructions(std::vector<Instruction> slow_instructions);
};



