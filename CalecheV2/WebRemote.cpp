#include "WebRemote.h"
#include "WiFiPrinter.h"
#include "PortExpander.h"
#include "PinDefinitions.h"
#include "Buzzer.h"
#include "Horn.h"
#include <ArduinoJson.h>

WebRemote::WebRemote(Dashboard& d, SpeedSensor& s, PedalSensor& p)
    : dashboard(d), speedSensor(s), pedalSensor(p)
{
    memset(pendingToggle, 0, sizeof(pendingToggle));

    WiFiPrinter::onMessage([this](const String& msg) {
        StaticJsonDocument<64> doc;
        if (deserializeJson(doc, msg) != DeserializationError::Ok)
            return;
        if (!doc.containsKey("toggleButton"))
            return;
        uint8_t i = doc["toggleButton"].as<uint8_t>();
        if (i < 4)
            pendingToggle[i] = true;
    });
}

void WebRemote::update()
{
    for (uint8_t i = 0; i < 4; i++) {
        if (pendingToggle[i]) {
            pendingToggle[i] = false;
            applyToggle(i);
        }
    }
}

void WebRemote::applyToggle(uint8_t i)
{
    dashboard.toggleState[i] = !dashboard.toggleState[i];
    Buzzer::getInstance().beep();

    if (i == 1 || i == 2) {
        Horn::getInstance().beep();
    } else if (i == 3) {
        // Handbrake: only allowed when fully stopped
        if (!speedSensor.isStopped() || !pedalSensor.isStopped())
            dashboard.toggleState[3] = LOW;
    }

    if (i == 0) {
        PortExpander& pe = PortExpander::getInstance();
        if (pe.initialized)
            pe.digitalWriteMCP23S17(MOSFET_NIGH_LIGHT_PIN, dashboard.toggleState[0]);
    }
}
