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
    Serial.print(left());
    Serial.print(" | ");
    Serial.print(front());
    Serial.print(" | ");
    Serial.println(right());
}

// Non-blocking delay that keeps printing sensor readings


void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    delay(2000);
    Serial.println("BOOT OK");

    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_BUILTIN, LOW);  delay(200);
        digitalWrite(LED_BUILTIN, HIGH); delay(200);
    }
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("LED TEST DONE");

    Wire.begin();
    Wire.setClock(400000);
    tofSetup();
    motorSetup();
    imuSetup();
    
    // Wait to Start
    digitalWrite(LED_BUILTIN, HIGH);
    smart_delay(25);
    int handState = 0;
    double t_start = micros();
    double t_buffer = micros();


    while(true) {
        double t_current = micros();
        int front_dist = front();
        if(front_dist < maxHandDistance && front_dist > 0 && t_current > t_buffer + 300000UL) {
            t_buffer = micros();
            handState = front_dist / (maxHandDistance / maxHandStates) + 1;
            Serial.print("Hand Found: "); Serial.println(handState);
        }

        if ((front_dist > maxHandDistance * 1.5) && handState > 0) {
            digitalWrite(LED_BUILTIN, HIGH);

            delay (250);
            if (handState == maxHandStates) {
                return;
            } else if (handState == 2) {
                explorer.isEncoder = true;
                pidForward(50, true);
                delay(2000);
                Serial.println("First Done");
            } else if (handState == 1) {
                pidForward(50);
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
        Serial.print("Node "); Serial.println(id);
        for (auto& edge : node.edges) {
            Serial.print("  -> Node "); Serial.print(edge.to_node_id);
            Serial.print(" cost: "); Serial.println(edge.cost);
        }
    }
}

int numbers[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int check = 0; 

void loop() {
    int sum = 0;

    for (int i = 0; i < 10; ++i) {
        sum += numbers[i];
    }



    Serial.print("Left: "); Serial.print(left()); Serial.print(" | Front: "); Serial.print(sum / 10); Serial.print(" | Right: "); Serial.println(right()); 
    numbers[check] = front();
    check++;
    if (check > 9) {
        check = 0;
    }
}
