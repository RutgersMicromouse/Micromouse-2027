#pragma once
#include "types.h"
#include <unordered_map>


struct Edge {
    int to_node_id;
    Direction departure_direction;
    std::vector<Instruction> instructions;
    float cost; // time
};

struct Node {
    int id;
    int x;
    int y;
    std::vector<Edge> edges;
};

class MazeGraph {
public:
    std::unordered_map<int, Node> nodes;
    int next_id = 0;

    int addNode(int x, int y) {
        nodes[next_id] = Node{next_id, x, y, {}};
        return next_id++;
    }

    void addEdge(int from, int to, Direction dir, std::vector<Instruction> instructions, float cost) {
        nodes[from].edges.push_back({to, dir, instructions, cost});
    }
};
