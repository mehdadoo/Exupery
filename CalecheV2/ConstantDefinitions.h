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
#define LIMIT_AUTHORISED_SPEED  17
#define MAX_AUTHORISED_SPEED    23
#define SPEED_SAMPLES           5  // Number of samples for smoothing


//PedalSensor
#define PEDAL_SENSOR_STOP_DELAY 200

//InclinationSensor
#define MPU_UPDATE_INTERVAL     50

//Dashboard
#define DASHBOARD_BUTTON_DEBOUNCE_DELAY 10
#define VOLTMETER_UPDATE_INTERVAL       10

//BrakeSystem
#define JOYSTICK_THROTTLE_SERVO_BRAKE_MAX  12
#define JOYSTICK_THROTTLE_SERVO_BRAKE_MIN  80
#define JOYSTICK_THROTTLE_REST_MIN         90
#define JOYSTICK_THROTTLE_REST_MAX         110
#define JOYSTICK_THROTTLE_MAX_VALUE        182

#define BRAKE_SERVO_1_MIN_VALUE            85  // <- front brakes
#define BRAKE_SERVO_1_HAND_BRAKE_VALUE     35
#define BRAKE_SERVO_1_MAX_VALUE            27 

#define BRAKE_SERVO_2_MIN_VALUE            60 // <- back brakes
#define BRAKE_SERVO_2_HAND_BRAKE_VALUE     100
#define BRAKE_SERVO_2_MAX_VALUE            125

#define BRAKE_BLINK_RATE                   45 //ms
#define BRAKE_BLINK_MULTIPLIER             4
#define BRAKE_EASE_OUT_MULTIPLIER          3.0


//SteeringSystem
#define JOYSTICK_STEERING_MAX_VALUE     206
#define JOYSTICK_STEERING_MIN_VALUE     0
#define JOYSTICK_STEERING_REST_GAP      14
#define STEERING_SERVO_MAX_VALUE        178
#define STEERING_SERVO_MIN_VALUE        0    //14
#define STEERING_SPEED_SCALE_FACTOR     0.9 // steering can be affected upto 70% by speed

//ThrottleSystem
#define POTENTIOMETER_MIN_VALUE     30    //.84v is the motor throttle min voltage
#define POTENTIOMETER_1_MIN_VALUE   10    //
#define POTENTIOMETER_1_MAX_VALUE   35    //75% of the max throttle of the motors. it will translate roughly to 2.85v (the throttle full range is 3.6v)
#define POTENTIOMETER_2_MIN_VALUE   33    //.84v is the motor throttle min voltage
#define POTENTIOMETER_2_MAX_VALUE   55    //50 is less than 50% of the max throttle of the motors. it will translate roughly to 2.2v (the throttle full range is 3.6v)
#define THROTTLE_UPDATE_EASE_SPEED  40    // the more the slower the ease
#define MINIMUM_SPEED_FOR_ENGINE_2_TO_START 2
#define DOWN_SHIFT_SPEED            2
#define UP_SHIFT_SPEED              3
#define ENGINE_1 true
#define ENGINE_2 false
#define KNOB_MAX_VALUE  207
#define KNOB_MIN_VALUE  0

//WiFiPrinter (Access Point mode — ESP32 hosts its own network)
#define AP_SSID                       "Caleche"
#define AP_PASSWORD                   "caleche48v"
#define UPDATE_OVER_WS_FREQUENCY      50    //ms

//LCDDisplay
#define DISPLAY_FPS 30

//Buzzer
#define BUZZER_BEEP_DURATION 30

//Horn
#define HORN_DURATION 10

#endif