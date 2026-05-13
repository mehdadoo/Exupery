#ifndef WEB_REMOTE_H
#define WEB_REMOTE_H

#include "Dashboard.h"
#include "SpeedSensor.h"
#include "PedalSensor.h"

class WebRemote {
public:
    WebRemote(Dashboard& dashboard, SpeedSensor& speedSensor, PedalSensor& pedalSensor);
    void update();

private:
    Dashboard& dashboard;
    SpeedSensor& speedSensor;
    PedalSensor& pedalSensor;
    bool pendingToggle[4];
    int steeringOverride;
    unsigned long steeringLastMs;

    void applyToggle(uint8_t i);
};

#endif
