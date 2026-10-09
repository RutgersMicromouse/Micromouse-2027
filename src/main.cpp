#include <Arduino.h>
#include "ioexpander.h"
#include "imu.h"
#include "tof.h"
#include "motors.h"
#include "pidstraight.h"
#include "pidrotate.h"
#include "graph.h"
#include "explorer.h"


MazeGraph graph;
Explorer explorer(graph);

int maxHandStates = 3;
int maxHandDistance = 100;


void distancePrint() {
    Serial1.print("Left");
    Serial1.print(left());
    Serial1.print("Front");
    Serial1.print(front());
    Serial1.print("Right");
    Serial1.print(right());
}

// Non-blocking delay that keeps printing sensor readings

void handleBluetoothCommand(String command) {
    command.trim();

    if (!command.startsWith("SET,")) {
        Serial1.println("ERROR,Unknown command");
        return;
    }

    int separator = command.indexOf(',', 4);
    if (separator < 0) {
        Serial1.println("ERROR,Invalid format");
        return;
    }

    String name = command.substring(4, separator);
    String valueString = command.substring(separator + 1);

    if (valueString.length() == 0) {
        Serial1.println("ERROR,Missing value");
        return;
    }

    char* endPtr;
    double value = strtod(valueString.c_str(), &endPtr);

    if (*endPtr != '\0' || !isfinite(value)) {
        Serial1.println("ERROR,Invalid number");
        return;
    }

    if (setPidParameter(name.c_str(), value)) {
        Serial1.print("ACK,");
        Serial1.print(name);
        Serial1.print(",");
        Serial1.println(value, 2);
    } else {
        Serial1.println("ERROR,Unknown parameter or value out of range");
    }
}

void pollBluetoothCommands() {
    static String commandBuffer;

    while (Serial1.available()) {
        char c = Serial1.read();

        if (c == '\n') {
            handleBluetoothCommand(commandBuffer);
            commandBuffer = "";
        } else if (c != '\r') {
            if (commandBuffer.length() < 80) {
                commandBuffer += c;
            } else {
                commandBuffer = "";
                Serial1.println("ERROR,Command too long");
            }
        }
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial1.begin(9600);
    delay(1000);

    for (int i = 0; i < 4; i++) {
        digitalWrite(LED_BUILTIN, LOW);  delay(250);
        digitalWrite(LED_BUILTIN, HIGH); delay(250);
    }
    digitalWrite(LED_BUILTIN, LOW);
    Serial1.println("LED TEST DONE");

    Serial1.println("BOOT OK");
    Serial1.println("GRAPH_RESET");
    Serial1.print("GRAPH_NODE,");
    Serial1.print(explorer.state.current_node_id);
    Serial1.print(",");
    Serial1.print(explorer.state.x);
    Serial1.print(",");
    Serial1.println(explorer.state.y);

    

    Wire.begin();
    Wire.setClock(400000);
    tofSetup();
    motorSetup();
    imuSetup();
    explorer.publishState();
    Serial1.println("Done");
    // Wait to Start
    digitalWrite(LED_BUILTIN, HIGH);
    smart_delay(500);
    int handState = 0;
    digitalWrite(LED_BUILTIN, LOW);
    double t_start = micros();
    double t_buffer = micros();


    while(true) {
        pollBluetoothCommands();
        double t_current = micros();
        int front_dist = front();
        if(front_dist < maxHandDistance && front_dist > 0 && t_current > t_buffer + 300000UL) {
            t_buffer = micros();
            handState = front_dist / (maxHandDistance / maxHandStates) + 1;
            Serial1.print("Hand Found: "); Serial1.println(handState);
        }

        if ((front_dist > maxHandDistance * 1.5) && handState > 0) {
            digitalWrite(LED_BUILTIN, HIGH);

            delay (250);
            if (handState == maxHandStates) {
                // Serial.println("Checking HC-05...");
                // Serial1.print("AT+UART=9600,0,0\r\n");
                // delay(5000);
                // Serial1.print("AT+UART?\r\n");
                pidForward(100);
                return;
            } else if (handState == 2) {
                explorer.isTof = true;
                pidForward(90, true);
                delay(2000);
                Serial1.println("Tof Mode Selected");

            } else if (handState == 1) {
                pidForward(90);
                Serial1.println("Encoder Mode Selected");
            }
            break;
        }

        int ledState = ((long)(t_current - t_start) / 100000) % (1 + handState);
        

        if (ledState != 0 || ledState == maxHandStates) {
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
        }
    }
    
    // Center in starting cell
    digitalWrite(LED_BUILTIN, HIGH);
    smart_delay(25); // let robot settle before loop starts

    explorer.explore();
    
    // print the graph so you can verify it over Serial
    for (auto& [id, node] : graph.nodes) {
        Serial1.print("Node "); Serial.println(id);
        for (auto& edge : node.edges) {
            Serial1.print("  -> Node "); Serial.print(edge.to_node_id);
            Serial1.print(" cost: "); Serial.println(edge.cost);
        }
    }
}


long bauds[] = {9600, 9600, 9600, 9600};
int idx = 0;

void loop() {
    delay(1000);
    distancePrint();
}
