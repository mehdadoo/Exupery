#ifndef VOLTAGESENSOR_H
#define VOLTAGESENSOR_H

class VoltageSensor {
public:
    // Public member variables
    bool initialized = false;
    float voltage = 0.0;
    int batteryPercentage = 0;

    // Public methods
    void start();
    void shutdown();
    void update();

private:
    // Private member variables
    unsigned long lastUpdateTime = 0; // Stores the last update time
    // Private methods
    float readBatteryVoltage();
};

#endif // VOLTAGESENSOR_H
