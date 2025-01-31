#ifndef CONSTANT_DEFINITIONS_H
#define CONSTANT_DEFINITIONS_H

#define MODULE_CONNECTION_TIMEOUT     2000
#define PWM_RESOLUTION                255 // PWM resolution (0-255 for analogWrite)

//IgnitionSwitch
#define IGNITION_SWITCH_OFF_TIME_DELAY 500
#define IGNITION_MOSFET_STARTUP_DELAY 100

//PortExapnder
#define PORT_EXPANDER_PORT_A 'A'
#define PORT_EXPANDER_PORT_B 'B'

//VoltageSensor
#define VOLTAGE_SENSOR_DEVIDER_FACTOR   0.0642 // the ratio of the R1 and R2 read by ohm meter
#define VOLTAGE_SENSOR_UPDATE_INTERVAL  1000
#define MIN_BATTERY_VOLTAGE             42.0   // Cutoff voltage (0% charge)
#define MAX_BATTERY_VOLTAGE             54.6  // Fully charged voltage (100% charge)

//SpeedSensor
#define WHEEL_DIAMETER          24
#define INCHES_TO_METERS        0.0254
#define SECONDS_PER_MINUTE      60
#define SENSOR_INTERVAL_3KMH    2300  // at 3k/h this time is needed for each reading of the speed sensor. so basically we consider speeds lower than this as stopped!
#define SENSOR_INTERVAL_25KMH   276 // Time in milliseconds for one magnet read at 30 km/h (24-inch wheel)
#define SENSOR_INTERVAL_30KMH   230 // Time in milliseconds for one magnet read at 30 km/h (24-inch wheel)
#define SENSOR_INTERVAL_50KMH   138 // Time in milliseconds for one magnet read at 50 km/h (24-inch wheel)

//PedalSensor
#define PEDAL_SENSOR_STOP_DELAY 200

//InclinationSensor
#define MPU_UPDATE_INTERVAL     100

//Dashboard
#define DASHBOARD_BUTTON_DEBOUNCE_DELAY 50
#define VOLTMETER_UPDATE_INTERVAL       10

//BrakeSystem
#define JOYSTICK_THROTTLE_SERVO_BRAKE_MAX  12
#define JOYSTICK_THROTTLE_SERVO_BRAKE_MIN  70
#define JOYSTICK_THROTTLE_REST_MIN         90
#define JOYSTICK_THROTTLE_REST_MAX         120
#define JOYSTICK_THROTTLE_MAX_VALUE        182
#define BRAKE_SERVO_MIN_VALUE              20
#define BRAKE_SERVO_MAX_VALUE              88
#define BRAKE_BLINK_RATE                   45 //ms
#define BRAKE_BLINK_MULTIPLIER             4

//SteeringSystem
#define JOYSTICK_STEERING_MAX_VALUE     206
#define JOYSTICK_STEERING_MIN_VALUE     0
#define JOYSTICK_STEERING_REST_GAP      14
#define STERING_SERVO_MAX_VALUE         178
#define STERING_SERVO_MIN_VALUE         14

//ThrottleSystem
#define POTENTIOMETER_MIN_VALUE 30 //.84v is the motor throttle min voltage
#define POTENTIOMETER_MAX_VALUE 63 //75% of the max throttle of the motors. it will translate roughly to 2.85v (the throttle full range is 3.6v)

//WiFiPrinter
#define WIFI_PASSWORD                 "cScrT9vJRTHCf3vzh2"
#define WIFI_SSID                     "Bbox-B3A9B36F"
#define MAX_WIFI_CONNECTION_RETRIES   5
#define RETRY_INTERVAL                1000  //ms
#define UPDATE_OVER_HTTP_FREQUENCY    200   //ms

//LCDDisplay
#define DISPLAY_FPS 50

#endif