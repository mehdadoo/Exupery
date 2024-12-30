#ifndef CONSTANT_DEFINITIONS_H
#define CONSTANT_DEFINITIONS_H

// GearBox
#define MIN_GEARBOX_ANGLE               170
#define MAX_GEARBOX_ANGLE               0
#define SPEED_SENSOR_LED_BLINK_DURATION 100

//BrakeSystem
#define MIN_BRAKE_ANGLE           55
#define DEFAULT_BRAKE_ANGLE       85
#define MAX_BRAKE_ANGLE           95
#define MAX_BRAKE_LEVER_STEPS     17
#define MIN_BRAKE_LEVER_THRESHOLD 2
#define BRAKE_BLINK_RATE          45
#define BRAKE_BLINK_MULTIPLIER    4
#define DELAY_BEFORE_FULL_BRAKE_BRIGHTNESS 2000

//SpeedSensor
#define WHEEL_DIAMETER        24
#define INCHES_TO_METERS      0.0254
#define SECONDS_PER_MINUTE    60
#define STOP_TIMEOUT          2300 // at 3k/h this time is needed for each reading of the speed sensor. so basically we consider speeds lower than this as stopped!
#define DEBOUNCE_INTERVAL     50         // Debounce interval in milliseconds
#define SPEED_UPDATE_FREQUENCY 1000

//WiFiPrinter
#define WIFI_PASSWORD                 "cScrT9vJRTHCf3vzh2"
#define WIFI_SSID                     "Bbox-B3A9B36F"
#define MAX_WIFI_CONNECTION_RETRIES   5
#define RETRY_INTERVAL                1000
#define RPM_CONSTANT                  1
#define SPEED_CONSTANT                2
#define BRAKE_LEVER_CONSTANT          3
#define BRAKE_SERVO_CONSTANT          4
#define CUSTOM_MESSAGE                5



#endif

