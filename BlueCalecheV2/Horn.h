#ifndef HORN_H
#define HORN_H

class Horn
{
  public:
    static Horn& getInstance()
    {
      static Horn instance;
      return instance;
    }

    void setup();
    void update();
    void beep();
    bool isOn() const { return isBeeping; }

  private:
    Horn();
    ~Horn() {}

    Horn(const Horn&) = delete;
    Horn& operator=(const Horn&) = delete;

    unsigned long beepStart;
    unsigned long debounceStartTime;
    bool isBeeping;
    bool lastButtonReading;
    bool stableButtonState;
};

#endif
