#include <functional>

class IgnitionSwitch {
public:
    IgnitionSwitch();

    // Method to set up the switch
    void setup();

    // Check the current state of the ignition
    void update();

    // Set external event listeners
    void setOnTurnedOnListener(std::function<void()> callback);
    void setOnTurnedOffListener(std::function<void()> callback);

    bool isKeyOn = false; // State of the key switch

private:
    

    // Event listeners
    std::function<void()> onTurnedOn = nullptr;
    std::function<void()> onTurnedOff = nullptr;
};
