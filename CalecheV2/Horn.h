#ifndef HORN_H
#define HORN_H

class Horn 
{
public:
    // Singleton access method
    static Horn& getInstance() 
    {
        static Horn instance;
        return instance;
    }

    // Public methods
    void update();
    void beep();

    // Public member variables
    bool initialized = false;
private:
    // Private constructor for Singleton
    Horn() {}
    ~Horn() {}

    // Delete copy constructor and assignment operator to prevent copying
    Horn(const Horn&) = delete;
    Horn& operator=(const Horn&) = delete;

    unsigned long beepStart; // Track start time for timeout
    bool is_beeping;
};

#endif // HORN_H
