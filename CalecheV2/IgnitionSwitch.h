#include <functional>

class IgnitionSwitch {
public:
    void setup();
    void update();
    void start();
    void shutdown();

    // Set external event listeners
    void setOnTurnedOnListener(std::function<void()> callback);
    void setOnTurnedOffListener(std::function<void()> callback);

    bool isKeyOn = false;
private:
    
    unsigned long keyOffTime = 0; // Time when key switched off
    bool waitingForOff = false;   // Flag to track off delay
    // Event listeners
    std::function<void()> onTurnedOn = nullptr;
    std::function<void()> onTurnedOff = nullptr;
};
