#ifndef CONSTANT_DEFINITIONS_H
#define CONSTANT_DEFINITIONS_H

#define MODULE_CONNECTION_TIMEOUT     2000
#define PWM_RESOLUTION                255 // PWM resolution (0-255 for analogWrite)

//IgnitionSwitch
#define IGNITION_SWITCH_OFF_TIME_DELAY 500
#define IGNITION_MOSFET_STARTUP_DELAY 100

//PortExapnder


//VoltageSensor
#define VOLTAGE_SENSOR_DEVIDER_FACTOR   0.0642 // the ratio of the R1 and R2 read by ohm meter
#define VOLTAGE_SENSOR_UPDATE_INTERVAL  1000
#define MIN_BATTERY_VOLTAGE             42.0   // Cutoff voltage (0% charge)
#define MAX_BATTERY_VOLTAGE             54.6  // Fully charged voltage (100% charge)

//SpeedSensor
#define WHEEL_DIAMETER          24
#define INCHES_TO_METERS        0.0254
#define SENSOR_INTERVAL_25KMH   276 // Time in milliseconds for one magnet read at 30 km/h (24-inch wheel)
#define SENSOR_INTERVAL_50KMH   138 // Time in milliseconds for one magnet read at 50 km/h (24-inch wheel)
#define LIMIT_AUTHORISED_SPEED  20
#define MAX_AUTHORISED_SPEED    30
#define SPEED_SAMPLES           5  // Number of samples for smoothing
#define DOWN_SHIFT_SPEED        10
#define UP_SHIFT_SPEED          13

//PedalSensor
#define PEDAL_SENSOR_STOP_DELAY 200

//InclinationSensor
#define MPU_UPDATE_INTERVAL     50

//Dashboard
#define DASHBOARD_BUTTON_DEBOUNCE_DELAY 10
#define VOLTMETER_UPDATE_INTERVAL       10

//BrakeSystem
#define JOYSTICK_THROTTLE_SERVO_BRAKE_MAX  12
#define JOYSTICK_THROTTLE_SERVO_BRAKE_MIN  70
#define JOYSTICK_THROTTLE_REST_MIN         90
#define JOYSTICK_THROTTLE_REST_MAX         120
#define JOYSTICK_THROTTLE_MAX_VALUE        182
#define BRAKE_SERVO_1_MIN_VALUE            20
#define BRAKE_SERVO_1_MAX_VALUE            88
#define BRAKE_SERVO_2_MIN_VALUE            38
#define BRAKE_SERVO_2_MAX_VALUE            93
#define BRAKE_BLINK_RATE                   45 //ms
#define BRAKE_BLINK_MULTIPLIER             4

//SteeringSystem
#define JOYSTICK_STEERING_MAX_VALUE     206
#define JOYSTICK_STEERING_MIN_VALUE     0
#define JOYSTICK_STEERING_REST_GAP      14
#define STERING_SERVO_MAX_VALUE         178
#define STERING_SERVO_MIN_VALUE         14

//ThrottleSystem
#define POTENTIOMETER_MIN_VALUE     30    //.84v is the motor throttle min voltage
#define POTENTIOMETER_1_MIN_VALUE   30    //.84v is the motor throttle min voltage
#define POTENTIOMETER_1_MAX_VALUE   63    //75% of the max throttle of the motors. it will translate roughly to 2.85v (the throttle full range is 3.6v)
#define POTENTIOMETER_2_MIN_VALUE   30    //.84v is the motor throttle min voltage
#define POTENTIOMETER_2_MAX_VALUE   63    //75% of the max throttle of the motors. it will translate roughly to 2.85v (the throttle full range is 3.6v)
#define THROTTLE_UPDATE_EASE_SPEED  40 // the more the slower the ease
#define ENGINE_1 true
#define ENGINE_2 false

//WiFiPrinter
#define WIFI_PASSWORD                 "cScrT9vJRTHCf3vzh2"
#define WIFI_SSID                     "Bbox-B3A9B36F"
#define MAX_WIFI_CONNECTION_RETRIES   4
#define RETRY_INTERVAL                1500  //ms
#define UPDATE_OVER_HTTP_FREQUENCY    300   //ms

//LCDDisplay
#define DISPLAY_FPS 30

//Buzzer
#define BUZZER_BEEP_DURATION 30

//Horn
#define HORN_DURATION 10

#endif