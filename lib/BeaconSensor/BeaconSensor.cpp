#include "BeaconSensor.h"


BeaconSensor::BeaconSensor(int receiverPin, int transmitterPin, int targetFreqHz)
    : _rxPin(receiverPin), _txPin(transmitterPin),
      _freqHz(targetFreqHz), _autoTransmit(false) {}


#define IR_RX_PIN   6   // TL1838V signal output → digital pin with interrupt support
#define IR_TX_PIN   11   // IR LED anode (through ~100Ω resistor to 5V)


BeaconSensor beacon(IR_RX_PIN, IR_TX_PIN, 38000);


bool beaconHandshakeComplete = false;


void BeaconSensor::begin() {
    pinMode(_rxPin, INPUT);    // TL1838V output is active-LOW
    pinMode(_txPin, OUTPUT);
    digitalWrite(_txPin, LOW);
}


// TL1838V output goes LOW when it detects a 38kHz modulated IR signal
bool BeaconSensor::detectBeacon() {
    bool detected = (digitalRead(_rxPin) == LOW);


    if (detected && _autoTransmit) {
        _generateCarrier(10); // Respond with a 10ms burst
    }


    return detected;
}


void BeaconSensor::transmitPulse() {
    _generateCarrier(10);
}


void BeaconSensor::setAutoTransmit(bool enabled) {
    _autoTransmit = enabled;
}


// Bit-bang a 38kHz carrier on the TX pin for durationMs milliseconds
// Period = 1,000,000 / 38000 ≈ 26.3µs → ~13µs HIGH, ~13µs LOW
void BeaconSensor::_generateCarrier(int durationMs) {
    unsigned long halfPeriodUs = 1000000UL / (_freqHz * 2);
    unsigned long endTime = millis() + durationMs;


    while (millis() < endTime) {
        digitalWrite(_txPin, HIGH);
        delayMicroseconds(halfPeriodUs);
        digitalWrite(_txPin, LOW);
        delayMicroseconds(halfPeriodUs);
    }
}
