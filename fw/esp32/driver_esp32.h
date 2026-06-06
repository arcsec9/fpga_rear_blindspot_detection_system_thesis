#pragma once

#include "driver_esp32.cpp"

// Function prototypes
class esp32 
{
public:
    int measureDistance(int trigPin, int echoPin, float calibrationFactor);
    void sendDataToFPGA(int value);
    void readLuxSensors();
    void readUltrasonicSensors();
    void readHB100Sensor();
    void handleFPGARequest();
    void printSensorData();
};