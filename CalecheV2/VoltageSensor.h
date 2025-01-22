#ifndef VOLTAGESENSOR_H
#define VOLTAGESENSOR_H

#include <Adafruit_ADS1X15.h>

class VoltageSensor {
public:
    // Public member variables
    bool initialized = false;
    float voltage = 0.0;
    float current = 0.0;
    int batteryPercentage = 100;

    // Public methods
    void start();
    void shutdown();
    void update();

private:
    // Private member variables
    Adafruit_ADS1115 ads1115;
    unsigned long lastUpdateTime = 0; // Stores the last update time

    // Private methods
    float readBatteryVoltage();
    int readCurrentSensor( int adcPin );
};

#endif // VOLTAGESENSOR_H
