#pragma once

#ifndef BEACON_SENSOR_H
#define BEACON_SENSOR_H


#include <Arduino.h>


class BeaconSensor {
public:
    BeaconSensor(int receiverPin, int transmitterPin, int targetFreqHz = 38000);


    void begin();
    bool detectBeacon();       // Returns true if IR signal detected
    void transmitPulse();      // Manually fire the TX LED
    void setAutoTransmit(bool enabled); // Auto-respond when beacon detected


private:
    int _rxPin;
    int _txPin;
    int _freqHz;
    bool _autoTransmit;


    void _generateCarrier(int durationMs);
};

extern BeaconSensor beacon;
extern bool beaconHandshakeComplete;

#endif
