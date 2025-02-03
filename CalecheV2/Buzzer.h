#ifndef BUZZER_H
#define BUZZER_H

class Buzzer 
{
public:
    // Singleton access method
    static Buzzer& getInstance() 
    {
        static Buzzer instance;
        return instance;
    }

    // Public methods
    void update();
    void beep();
    void beep2();
    void beep3();
    void toggle();
    void off();

    // Public member variables
    bool initialized = false;
private:
    // Private constructor for Singleton
    Buzzer() {}
    ~Buzzer() {}

    // Delete copy constructor and assignment operator to prevent copying
    Buzzer(const Buzzer&) = delete;
    Buzzer& operator=(const Buzzer&) = delete;

    unsigned long beepStart; // Track start time for timeout
    bool is_beeping;
    bool is_beeping_2;
    bool is_beeping_3;
    bool beep_state = LOW;
};

#endif // BUZZER_H
