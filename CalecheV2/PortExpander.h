#ifndef PORT_EXPANDER_H
#define PORT_EXPANDER_H

#include "PinDefinitions.h"

class PortExpander 
{
public:
    // Singleton access method
    static PortExpander& getInstance() 
    {
        static PortExpander instance;
        return instance;
    }

    // Public methods
    void start();
    void shutdown();
    void update();

    uint8_t digitalReadMCP23S17(const PortExpanderPin& pin);
    void digitalWriteMCP23S17(const PortExpanderPin& pin,  uint8_t value);
    // Public member variables
    bool initialized = false;

private:
    // Private constructor for Singleton
    PortExpander() {}
    ~PortExpander() {}

    // Delete copy constructor and assignment operator to prevent copying
    PortExpander(const PortExpander&) = delete;
    PortExpander& operator=(const PortExpander&) = delete;

    // Private methods
    void writeMCP23S17(uint8_t registerAddress, uint8_t data);
    uint8_t readMCP23S17(uint8_t registerAddress);
    void pinModeMCP23S17(const PortExpanderPin& pin, uint8_t mode);
    void pullUpMCP23S17(const PortExpanderPin& pin, bool enable);
    
};

#endif // PORT_EXPANDER_H
