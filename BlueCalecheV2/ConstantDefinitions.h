#ifndef CONSTANT_DEFINITIONS_H
#define CONSTANT_DEFINITIONS_H

// Physical inputs
#define ENABLE_SPEED_SENSOR          true
#define ENABLE_HAND_BRAKE_BUTTON     true
#define ENABLE_NIGHT_LIGHT_BUTTON    true
#define ENABLE_HORN_BUTTON           true


//BrakeSystem
#define BRAKE_SERVO_1_MIN_VALUE        85  // Front brakes
#define BRAKE_SERVO_1_HAND_BRAKE_VALUE 35
#define BRAKE_SERVO_1_MAX_VALUE        27

#define BRAKE_SERVO_2_MIN_VALUE        60  // Back brakes
#define BRAKE_SERVO_2_HAND_BRAKE_VALUE 100
#define BRAKE_SERVO_2_MAX_VALUE        125

#define BRAKE_LEVER_RESOLUTION    1000
#define MIN_BRAKE_LEVER_THRESHOLD 0
#define BRAKE_BLINK_RATE          45 //ms
#define BRAKE_BLINK_MULTIPLIER    4
#define BRAKE_EASE_OUT_MULTIPLIER 3.0
#define HAND_BRAKE_DEBOUNCE_MS    50
#define NIGHT_LIGHT_DEBOUNCE_MS   50

//Horn
#define HORN_DURATION             150   //ms
#define HORN_BUTTON_DEBOUNCE_MS   50    //ms

//Built-in LED heartbeat
#define HEARTBEAT_INTERVAL_MS     2000
#define HEARTBEAT_DURATION_MS     30
#define HEARTBEAT_BRIGHTNESS      16
#define BRAKE_POT_MIN_READING     2300  // Joystick center/rest position
#define BRAKE_POT_MAX_READING     4095
#define BRAKE_POT_DEAD_ZONE       0
#define BRAKE_POT_FILTER_SAMPLES  8
#define BRAKE_POT_SAMPLE_INTERVAL 5     //ms
#define BRAKE_POT_REVERSED        false


//SpeedSensor
#define WHEEL_DIAMETER        24
#define INCHES_TO_METERS      0.0254
#define SECONDS_PER_MINUTE    60
#define SENSOR_INTERVAL_3KMH  2300  // at 3k/h this time is needed for each reading of the speed sensor. so basically we consider speeds lower than this as stopped!
#define SENSOR_INTERVAL_25KMH 276 // Time in milliseconds for one magnet read at 30 km/h (24-inch wheel)
#define SENSOR_INTERVAL_30KMH 230 // Time in milliseconds for one magnet read at 30 km/h (24-inch wheel)
#define SENSOR_INTERVAL_50KMH 138 // Time in milliseconds for one magnet read at 50 km/h (24-inch wheel)

//WiFiPrinter (Access Point mode - ESP32 hosts its own network)
#define AP_SSID                       "Caleche"
#define AP_PASSWORD                   "caleche48v"
#define OTA_HOSTNAME                  "blue-caleche"
#define UPDATE_OVER_WS_FREQUENCY      50    //ms


#endif

