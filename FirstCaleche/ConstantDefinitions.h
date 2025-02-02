#ifndef CONSTANT_DEFINITIONS_H
#define CONSTANT_DEFINITIONS_H

// GearBox
#define MIN_GEARBOX_ANGLE       170
#define MAX_GEARBOX_ANGLE       10
#define FIRST_GEAR_SPEED        3.0 //kmh
#define LAST_GEAR_SPEED         12.0 //kmh
#define TOTAL_GEARS             8
#define SHIFT_COOLDOWN_TIME     1500 //ms
#define DOWNSHIFT               true
#define UPSHIFT                 false

//BrakeSystem
#define MIN_BRAKE_ANGLE           55
#define DEFAULT_BRAKE_ANGLE       85
#define MAX_BRAKE_ANGLE           95
#define MAX_BRAKE_LEVER_STEPS     13 //steps
#define MIN_BRAKE_LEVER_THRESHOLD 1  //steps
#define BRAKE_BLINK_RATE          45 //ms
#define BRAKE_BLINK_MULTIPLIER    4
#define BRAKE_EASE_OUT_MULTIPLIER 2.0

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
#define UPDATE_OVER_HTTP_FREQUENCY    500   //ms


#endif

