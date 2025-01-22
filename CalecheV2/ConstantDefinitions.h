#ifndef CONSTANT_DEFINITIONS_H
#define CONSTANT_DEFINITIONS_H


#define PWM_RESOLUTION 255 // PWM resolution (0-255 for analogWrite)

#define MPU_UPDATE_INTERVAL           200
#define MODULE_CONNECTION_TIMEOUT     2000
#define CURRENTSENSOR_UPDATE_INTERVAL 2000
#define VOLTMETER_UPDATE_INTERVAL     200


#define JOYSTICK_THROTTLE_MAX_VALUE     200
#define JOYSTICK_THROTTLE_MIDDLE_VALUE  100
#define JOYSTICK_THROTTLE_MIN_VALUE     17

#define JOYSTICK_STEERING_MAX_VALUE     164
#define JOYSTICK_STEERING_MIN_VALUE     22

#define STERING_SERVO_MAX_VALUE     136
#define STERING_SERVO_MIN_VALUE     0


#define BRAKE_SERVO_MIN_VALUE 0
#define BRAKE_SERVO_MAX_VALUE 180

#define POTENTIOMETER_MIN_VALUE 30 //.84v is the motor throttle min voltage
#define POTENTIOMETER_MAX_VALUE 63 //75% of the max throttle of the motors. it will translate roughly to 2.85v (the throttle full range is 3.6v)


//SpeedSensor
#define WHEEL_DIAMETER        24
#define INCHES_TO_METERS      0.0254
#define SECONDS_PER_MINUTE    60
#define SENSOR_INTERVAL_3KMH  2300  // at 3k/h this time is needed for each reading of the speed sensor. so basically we consider speeds lower than this as stopped!
#define SENSOR_INTERVAL_25KMH 276 // Time in milliseconds for one magnet read at 30 km/h (24-inch wheel)
#define SENSOR_INTERVAL_30KMH 230 // Time in milliseconds for one magnet read at 30 km/h (24-inch wheel)
#define SENSOR_INTERVAL_50KMH 138 // Time in milliseconds for one magnet read at 50 km/h (24-inch wheel)

//WiFiPrinter
#define WIFI_PASSWORD                 "cScrT9vJRTHCf3vzh2"
#define WIFI_SSID                     "Bbox-B3A9B36F"
#define MAX_WIFI_CONNECTION_RETRIES   5
#define RETRY_INTERVAL                1000  //ms
#define UPDATE_OVER_HTTP_FREQUENCY    200   //ms

#define MIN_BATTERY_VOLTAGE  42.0   // Cutoff voltage (0% charge)
#define MAX_BATTERY_VOLTAGE  54.6  // Fully charged voltage (100% charge)

#endif