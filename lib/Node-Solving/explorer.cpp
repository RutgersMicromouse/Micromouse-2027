#include "explorer.h"
#include "tof.h"
#include "pidstraight.h"
#include "pidrotate.h"
#include <queue>
#include <limits>
#include <algorithm>
// #include "BeaconSensor.h"


void Explorer::explore() {
    // beacon.begin();
    // beacon.setAutoTransmit(true);

    // Stage 1: Explore Maze
    exploreFromNode(state.current_node_id, Direction::NORTH);
    faceDirection(oppositeDirection(state.heading));

    // Calculations
    std::vector<int> bestPath = fastestRunAStar(0, center_node_id); // std::vector<int> bestPath = fastestRunAStar(0, beacon_node_id);
    //smart_delay(10000);
    Serial.print("---------------");
    Serial.print("Shortest Path: ");
    for(int i = 0; i < bestPath.size(); i++) {
        Serial.print(bestPath[i]);
        Serial.print(" -> ");
    }
    Serial.println("");
    std::vector<Instruction> instructionsToFinal = buildFastestInstructions(bestPath);
    
    Serial.print("---------------");
    std::vector<Instruction> reverseInstructionsToFinal;
    for(auto it = instructionsToFinal.rbegin(); it != instructionsToFinal.rend(); ++it) {
        Instruction instr = *it;

        switch (instr.type) {
            case InstructionType::ROTATE_RELATIVE:
                reverseInstructionsToFinal.push_back({
                    InstructionType::ROTATE_RELATIVE,
                    (instr.value * -1)
                });
                Serial.print("Type: Relative | Value: "); Serial.println(instr.value);
                break;
            
            case InstructionType::FORWARD:
                reverseInstructionsToFinal.push_back(instr);
                Serial.print("Type: Forward | Value: "); Serial.println(instr.value);
                break;
        }
    }

    std::vector<Instruction> backToFinal = condenseInstructions(instructionsToFinal);
    std::vector<Instruction> backToStart = condenseInstructions(reverseInstructionsToFinal);
    
    while(true) {
        digitalWrite(LED_BUILTIN, HIGH);
        smart_delay(250);
        bool hand = false;
        while(true) {
            if(front() < 60 && front() > 0) {
                hand = true;
                Serial.println("Hand Found");
                digitalWrite(LED_BUILTIN, LOW);
            }
            if ((front() > 100) && (hand == true)) {
                digitalWrite(LED_BUILTIN, HIGH);
                break;
            }
        }

        //STAGE 2: Solving
        executeInstructions(instructionsToFinal);  // EDIT HERE instructionsToFinal

        // beacon
        //smart_delay(1000);
        Direction original_heading = state.heading;
        //smart_delay(1000);
        if (front() >= 10) {
            pidForward(50, isEncoder);
            smart_delay(25);
            pidReverse(40);
        }
        faceDirection(original_heading);
        
        //smart_delay(1000);


        //return back to start
        faceDirection(oppositeDirection(state.heading));  // EDIT HERE
        executeInstructions(reverseInstructionsToFinal);
        
        faceDirection(oppositeDirection(state.heading));

        delay(25);
    }
    //STAGE 3: Solving Faster
    // faceDirection(oppositeDirection(state.heading));
    // delay(25);
    // executeInstructions(instructionsToFinal);  // EDIT HERE instructionsToFinal

    // beacon
    // //smart_delay(1000);
    // Direction original2_heading = state.heading;
    // //smart_delay(1000);
    // if (front() >= 10) {
    //     pidForward(50, isEncoder);
    //     delay(3000);
    //     pidReverse(40);
    // }
    // faceDirection(original2_heading);
    
    // //smart_delay(1000);


    //return back to start
    // faceDirection(oppositeDirection(state.heading));  // EDIT HERE
    // executeInstructions(backToStart);


}

void smart_delay(int ms) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)ms) {
        //distancePrint();
        left();
        front();
        right();

        delay(1);
    }
}

void Explorer::exploreFromNode(int node_id, Direction heading_to_parent) {
    //smart_delay(25);
    Serial.print("----- Node: ");
    Serial.print(node_id);
    std::vector<Direction> exits = readExits();
    Serial.print(" [Size: ");
    Serial.print(exits.size());
    if(exits.size() > 1) {
        Serial.print("] [First Checked: ");
        Serial.print(static_cast<int>(exits[0]));
        Serial.print("] [Second Checked: ");
        Serial.print(static_cast<int>(exits[1]));
        Serial.print("]");
        Serial.println(" -----");
    }
    //smart_delay(25);
    for (Direction dir : exits) {
        Serial.print("Test Facing: ");
        Serial.println(static_cast<int>(dir));
        // skip if we already have an edge in this direction
        bool alreadyExplored = false;
        for (auto& [id, node] : graph.nodes) {
            Serial.print("Node "); Serial.println(id);
            for (auto& edge : node.edges) {
                Serial.print("  -> Node "); Serial.print(edge.to_node_id);
                Serial.print(" ("); Serial.print(static_cast<int>(edge.departure_direction)); Serial.println(")");
                Serial.print(" cost: "); Serial.println(edge.cost);
            }
        }
        for (auto& edge : graph.nodes[node_id].edges) {
            if (edge.departure_direction == dir) {
                alreadyExplored = true;
                break;
            }
        }
        if (alreadyExplored) continue;

        // face the exit direction
        Serial.print("Actual Facing: ");
        Serial.println(static_cast<int>(exits[0]));
        faceDirection(dir);
        // Serial.println(static_cast<int>(dir));

        //Gets instructions to next node
        std::vector<Instruction> instructions;
        float cost = 0;
        bool discoveredNewNode = false;
        int next_node_id = traverseCorridor(dir, instructions, cost, discoveredNewNode);


        // build reverse instructions for backtracking
        std::vector<Instruction> reverseInstructions;
        for(auto it = instructions.rbegin(); it != instructions.rend(); ++it) {
            Instruction instr = *it;

            switch (instr.type) {
                case InstructionType::ROTATE_RELATIVE:
                    reverseInstructions.push_back({
                        InstructionType::ROTATE_RELATIVE,
                        (instr.value * -1)
                    });
                    break;
                
                case InstructionType::FORWARD:
                    reverseInstructions.push_back(instr);
                    break;
            }
        }


        // add edges in both directions
        Serial.print("Adding Edges Home Node: ");
        Serial.print(node_id);
        Serial.print("| HEADING: ");
        Serial.println(static_cast<int>(oppositeDirection(state.heading)));
        graph.addEdge(node_id, next_node_id, dir, instructions, cost);
        Direction parent_heading = oppositeDirection(state.heading);
        graph.addEdge(next_node_id, node_id, oppositeDirection(state.heading), reverseInstructions, cost);
        Serial.print("Discovered New Node?: ");
        Serial.println(discoveredNewNode);
        if (discoveredNewNode) {
            // recurse into new node
            if (center_node_id == 0) {
                Serial.print("Home Node: ");
                Serial.print(node_id);
                Serial.print("| HEADING: ");
                Serial.println(static_cast<int>(parent_heading));
                exploreFromNode(next_node_id, parent_heading);
            }
            
            
            Serial.print("Leaving That Node: ");
            Serial.print(node_id);
            Serial.print("| HEADING: ");
            Serial.println(static_cast<int>(parent_heading));
            faceDirection(parent_heading);
        } else {
            faceDirection(oppositeDirection(state.heading));
        }
        Serial.println("");
        Serial.print("----- Final Position: (");
        Serial.print(state.x);
        Serial.print(", ");
        Serial.print(state.y);
        Serial.print(") HEADING: ");
        Serial.print(static_cast<int>(state.heading));
        Serial.println(" -----");


        // backtrack to current node
        executeInstructions(reverseInstructions);
        state.current_node_id = node_id;
    }
    Serial.print("Escaping Node: ");
    Serial.println(node_id);
}


std::vector<Direction> Explorer::readExits() {
    std::vector<Direction> exits;

    // Serial.print("* Left: ");
    // Serial.print(left());
    // Serial.print("| Front: ");
    // Serial.print(front());
    // Serial.print("| Right: ");
    // Serial.println(right());

    // front is current heading
    if (front() > WALL_THRESHOLD_MM) {
        // Serial.println(static_cast<int>(state.heading));
        exits.push_back(state.heading);
    }


    // left is 90 degrees CCW
    Direction leftDir = static_cast<Direction>((static_cast<int>(state.heading) + 3) % 4);
    if (left() > WALL_THRESHOLD_MM) {
        // Serial.println(static_cast<int>(leftDir));
        exits.push_back(leftDir);
    }


    // right is 90 degrees CW
    Direction rightDir = static_cast<Direction>((static_cast<int>(state.heading) + 1) % 4);
    if (right() > WALL_THRESHOLD_MM) {
        // Serial.println(static_cast<int>(rightDir));
        exits.push_back(rightDir);
    }


    return exits;
}


void Explorer::executeInstructions(const std::vector<Instruction>& instructions) {
    for (const auto& instr : instructions) {
        switch (instr.type) {
            case InstructionType::FORWARD:
                Serial.println("Reverse (Forward)");
                pidForward(instr.value * CELL_SIZE_MM, isEncoder);
                updatePosition(instr.value);
                break;
            
            case InstructionType::ROTATE_RELATIVE:
                pidRotate(instr.value);

                if (instr.value == 90) {
                    state.heading = static_cast<Direction>((static_cast<int>(state.heading) + 3) % 4);
                } 
                else if (instr.value == -90) {
                    state.heading = static_cast<Direction>((static_cast<int>(state.heading) + 1) % 4);
                } 
                break;
        } 
        Serial.println("");
        Serial.print("----- Position: (");
        Serial.print(state.x);
        Serial.print(", ");
        Serial.print(state.y);
        Serial.print(") HEADING: ");
        Serial.print(static_cast<int>(state.heading));
        Serial.println(" -----");
        //smart_delay(250);
    }
}

std::vector<Instruction> Explorer::condenseInstructions(std::vector<Instruction> slow_instructions) {
    std::vector<Instruction> faster_instructions;
    int move;
    int value = 0;
    for(auto it = slow_instructions.rbegin(); it != slow_instructions.rend(); ++it) {
        Instruction instr = *it;

        switch (instr.type) {
            case InstructionType::ROTATE_RELATIVE:
                if (move == 1) {
                    faster_instructions.push_back({
                        InstructionType::FORWARD,
                        value
                    });
                }
                faster_instructions.push_back({
                    InstructionType::ROTATE_RELATIVE,
                    (instr.value * 1)
                });
                move = 0;
                value = 0;
                break;
            
            case InstructionType::FORWARD:
                value += instr.value;
                move = 1;
                break;
        }
    }
    if (move == 1) {
        faster_instructions.push_back({
            InstructionType::FORWARD,
            value
        });
    }
    return faster_instructions;
}

int Explorer::traverseCorridor(Direction dir, std::vector<Instruction>& out_instructions, float& out_cost, bool& out_discoveredNewNode) {
    int steps = 0;

    while (true) {
        // move one cell forward
        Serial.println("");
        Serial.print("----- Position: (");
        Serial.print(state.x);
        Serial.print(", ");
        Serial.print(state.y);
        Serial.print(") HEADING: ");
        Serial.print(static_cast<int>(state.heading));
        Serial.println(" -----");
        // Serial.println("Before Moves (1): ");
        // readExits();
        //smart_delay(500);
        // Serial.println("Before Moves (2): ");
        // readExits(1000);
        
        bool isCentered = false;

        if ((state.x == 7 || state.y == 8) && (state.x == 7 || state.y == 8)) {
            isCentered == true;
        } else {
            Serial.print("Left: "); Serial.print(left()); Serial.print(" | Front: "); Serial.print(front()); Serial.print(" | Right: "); Serial.println(right());
            if (front() > WALL_THRESHOLD_MM) {
                Serial.println("Move forward");
                pidForward(CELL_SIZE_MM, isEncoder);
                updatePosition(1);
                steps++;
                out_instructions.push_back({InstructionType::FORWARD, 1});
            } else if (right() > WALL_THRESHOLD_MM) {
                Serial.println("Move right");
                pidRotate(-90);
                state.heading = static_cast<Direction>((static_cast<int>(state.heading) + 1) % 4);
                smart_delay(25);
                pidForward(CELL_SIZE_MM, isEncoder);
                updatePosition(1);
                steps+=2;
                out_instructions.push_back({InstructionType::ROTATE_RELATIVE, -90});
                out_instructions.push_back({InstructionType::FORWARD, 1});
            } else {
                Serial.println("Move left");
                pidRotate(90);
                state.heading = static_cast<Direction>((static_cast<int>(state.heading) + 3) % 4);
                smart_delay(25);
                pidForward(CELL_SIZE_MM, isEncoder);
                updatePosition(1);
                steps+=2;
                out_instructions.push_back({InstructionType::ROTATE_RELATIVE, 90});
                out_instructions.push_back({InstructionType::FORWARD, 1});
            }
        }

        

        // check if this position is already known;
        //smart_delay(25);
        // Serial.print("After Moves: ");
        std::vector<Direction> exits = readExits();
        if (isKnownPosition(state.x, state.y)) {
            state.current_node_id = positionMap[positionKey(state.x, state.y)];
            Serial.println("Known Position");
            break;
        }


        bool isTripped = false;
        // if (beacon_node_id == 0) {
        //     Direction facing;
        //     tripBeacon(isTripped, facing);
        //     if (isTripped) {
        //         beacon_heading = facing;
        //     }
        // }
        


        // check if we've reached a junction (more than one exit)
        Serial.print("Left: "); Serial.print(left()); Serial.print("Front: "); Serial.print(front()); Serial.print("Right: "); Serial.println(right()); 
        bool isDeadEnd = exits.empty();
        Serial.print("Exit Sizes: ");
        Serial.println(exits.size());
        bool isJunction = exits.size() > 1;
        
        // bool beaconDetected = false;
        // if (isDeadEnd && beacon_node_id == 0) {
        //     //smart_delay(1000);
        //     if (front() >= 60) {
        //         pidForward(50, isEncoder);
        //         beaconDetected = true;
        //         digitalWrite(LED_BUILTIN, HIGH);
        //         //smart_delay(3000);
        //         pidReverse(40);
        //     }

        // }

        if (isJunction || isDeadEnd || isTripped || isCentered) {
            int key = positionKey(state.x, state.y);
            Serial.println(key);
            if (positionMap.count(key) > 0) {
                state.current_node_id = positionMap[key];
                out_discoveredNewNode = false;
            }
            else {
                int new_id = graph.addNode(state.x, state.y);
                positionMap[positionKey(state.x, state.y)] = new_id;
                state.current_node_id = new_id;
                // if (beaconDetected) {
                //     beacon_node_id = new_id;
                // }
                if (isCentered) {
                    center_node_id = new_id;
                }
                out_discoveredNewNode = true;
                // if (isTripped) {
                //     beacon_node_id = new_id;
                    
                //     digitalWrite(LED_BUILTIN, LOW);
                //     for(int i = 0; i < beacon_node_id; i++) {
                //         digitalWrite(LED_BUILTIN, HIGH); delay(500);
                //         digitalWrite(LED_BUILTIN, LOW); delay(500);
                //     }
                // }
            }
            break;
        }
    }

    out_cost = steps; // in cells; multiply by CELL_SIZE_MM for mm

    //smart_delay(50);
    return state.current_node_id;
}

bool Explorer::isKnownPosition(int x, int y) {
    return positionMap.count(positionKey(x, y)) > 0;
}

void Explorer::faceDirection(Direction d) {
    turnTo(directionToDegrees(d));
    state.heading = d;
}


void Explorer::updatePosition(int steps) {
    switch (state.heading) {
        case Direction::NORTH: state.y += steps; break;
        case Direction::SOUTH: state.y -= steps; break;
        case Direction::EAST:  state.x += steps; break;
        case Direction::WEST:  state.x -= steps; break;
    }
}


Direction Explorer::oppositeDirection(Direction d) {
    return static_cast<Direction>((static_cast<int>(d) + 2) % 4);
}

//works
int Explorer::directionToDegrees(Direction d) {
    return static_cast<int>(d) * -90;
}

float heuristic(const Node& a, const Node& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

std::vector<int> Explorer::fastestRunAStar(int start_node, int goal_node) {

    // gScore = actual cost from start
    std::unordered_map<int, float> gScore;

    // fScore = gScore + heuristic
    std::unordered_map<int, float> fScore;

    // path reconstruction
    std::unordered_map<int, int> previous;

    // initialize scores
    for (auto& [id, node] : graph.nodes) {
        gScore[id] = std::numeric_limits<float>::infinity();
        fScore[id] = std::numeric_limits<float>::infinity();
    }

    gScore[start_node] = 0;

    fScore[start_node] =
        heuristic(graph.nodes[start_node], graph.nodes[goal_node]);

    // priority queue stores:
    // (fScore, node_id)

    using PQNode = std::pair<float, int>;

    auto cmp = [](PQNode a, PQNode b) {
        return a.first > b.first;
    };

    std::priority_queue<
        PQNode,
        std::vector<PQNode>,
        decltype(cmp)
    > openSet(cmp);

    openSet.push({fScore[start_node], start_node});

    while (!openSet.empty()) {

        int current = openSet.top().second;
        openSet.pop();

        // reached target
        if (current == goal_node)
            break;

        for (const auto& edge : graph.nodes[current].edges) {

            int neighbor = edge.to_node_id;

            float tentative_g =
                gScore[current] + edge.cost;

            if (tentative_g < gScore[neighbor]) {

                previous[neighbor] = current;

                gScore[neighbor] = tentative_g;

                fScore[neighbor] =
                    tentative_g +
                    heuristic(graph.nodes[neighbor], graph.nodes[goal_node]);

                openSet.push({
                    fScore[neighbor],
                    neighbor
                });
            }
        }
    }

    // reconstruct path
    std::vector<int> path;

    if (gScore[goal_node] ==
        std::numeric_limits<float>::infinity()) {
        return path;
    }

    int current = goal_node;

    while (current != start_node) {
        path.push_back(current);
        current = previous[current];
    }

    path.push_back(start_node);

    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<Instruction> Explorer::getInstructionsBetween(
    int from_node,
    int to_node,
    int& out_heading
) {
    for (const auto& edge : graph.nodes[from_node].edges) {

        if (edge.to_node_id == to_node) {
            std::vector<Instruction> route = edge.instructions;
            if (static_cast<int>(edge.departure_direction) == ((out_heading + 1) % 4)) {
                route.insert(route.begin(), {
                    InstructionType::ROTATE_RELATIVE,
                    (-90)
                });
            } else if (static_cast<int>(edge.departure_direction) == ((out_heading + 3) % 4)) {
                route.insert(route.begin(), {
                    InstructionType::ROTATE_RELATIVE,
                    (90)
                });
            }
            for (const auto& edge_reverse : graph.nodes[to_node].edges) {
                if (edge_reverse.to_node_id == from_node) {
                    out_heading = static_cast<int>(oppositeDirection(edge_reverse.departure_direction));
                }
            }
            
            return route;
        }
    }

    // no connection found
    return {};
}

std::vector<Instruction> Explorer::buildFastestInstructions(const std::vector<int>& path) {
    std::vector<Instruction> fastInstruction;
    int heading = static_cast<int>(state.heading);
    for (size_t i = 0; i < path.size() - 1; i++) {

        int from = path[i];
        int to = path[i + 1];
        std::vector<Instruction> segmentInstruction = getInstructionsBetween(from, to, heading);

        fastInstruction.insert(fastInstruction.end(), segmentInstruction.begin(), segmentInstruction.end());     
    }
    return fastInstruction;
}

// void Explorer::tripBeacon(bool& out_isTripped, Direction& out_facing) {
    
//     Direction left = static_cast<Direction>((static_cast<int>(state.heading) + 3 ) % 4);
//     Direction right = static_cast<Direction>((static_cast<int>(state.heading) + 1 ) % 4);
//     Direction front = state.heading;

//     faceDirection(left); // left
//     bool beaconDetected = beacon.detectBeacon();
//     if (beaconDetected) {
//         out_facing = left;
//         out_isTripped = true;
//     }


//     faceDirection(right); // right

//     beaconDetected = beacon.detectBeacon();
//     if (beaconDetected) {
//         out_facing = right;
//         out_isTripped = true;
//     }

//     faceDirection(front);
//     beaconDetected = beacon.detectBeacon();
//     if (beaconDetected) {
//         out_facing = front;
//         out_isTripped = true;
//     }
// }
