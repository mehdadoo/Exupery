# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32 firmware for a 48V electric vehicle ("Calèche"). Written in Arduino C++ using the Arduino IDE framework. All subsystems are organized as C++ classes with a consistent `start()` / `update()` / `shutdown()` lifecycle.

## Build & Flash

This is an Arduino IDE project with no build scripts. Use the Arduino IDE or Arduino CLI:

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32 .

# Upload via USB (replace COM port as appropriate)
arduino-cli upload --fqbn esp32:esp32:esp32 --port COM3 .

# OTA upload (when WiFi is connected on the vehicle)
arduino-cli upload --fqbn esp32:esp32:esp32 --port <vehicle-ip> .
```

Required libraries (install via Arduino Library Manager):
- `ESP32Servo`
- `DigiPotX9Cxxx`
- `Adafruit ADS1X15`
- `Arduino_GFX`
- `ArduinoJson`
- `ArduinoOTA` / `WebServer` (bundled with ESP32 core)

## Architecture

### Entry Point & Lifecycle

`CalecheV2.ino` is the entry point. `IgnitionSwitch` monitors the physical key switch and fires `start()` / `shutdown()` callbacks, which call `start()` / `shutdown()` on every subsystem in dependency order. The Arduino `loop()` calls `update()` on all subsystems every iteration.

Every subsystem guards its `update()` with an `initialized` flag — never call `update()` before `start()`, and never call `start()` without SPI/I2C being active (SPI is started in `start()` and ended in `shutdown()` in the main `.ino`).

### Dependency Graph

```
Dashboard ──────────────────────────────────┐
  └── ADS1115 (I2C) for joystick ADC       │
  └── VoltageSensor, SpeedSensor,           │
      PedalSensor (read-only refs)          │
                                            ▼
BrakeSystem(Dashboard, SpeedSensor)   ThrottleSystem(Dashboard, PedalSensor, SpeedSensor)
SteeringSystem(Dashboard, SpeedSensor)
```

### Communication Buses

| Bus  | Pins (SCK/MOSI/MISO) | Devices |
|------|----------------------|---------|
| SPI  | 12 / 13 / 37         | MCP23S17 port expander (CS=38), TFT display (CS=2) |
| I2C  | SDA=35, SCL=36       | ADS1115 (joystick), MPU6050 (inclination) |

### Key Subsystems

- **PortExpander** (`MCP23S17` via SPI) — Singleton. All buttons, brake light MOSFET, horn, night light, reverse MOSFET, buzzer, wheel-speed sensor, and pedal sensor are read/written through this chip. Use `PortExpander::getInstance()`. Pin mapping is defined as `PortExpanderPin` structs in `PinDefinitions.h`.

- **ThrottleSystem** — Two X9C digital potentiometers control two separate motor controllers. Currently the `experimentalThrottle()` path is active (joystick = pot1, knob = pot2 multiplier). The multi-engine shifting logic (`chooseEngine()`, `calculateTargetPotValues()`) is present but commented out in `update()`. Speed is hard-limited via `limitMaxSpeed()` between `LIMIT_AUTHORISED_SPEED` (17 km/h) and `MAX_AUTHORISED_SPEED` (23 km/h).

- **BrakeSystem** — Two servos: `SERVO_BRAKE_1` (front, pin 7) and `SERVO_BRAKE_2` (rear, pin 17). Joystick pulled backward past `JOYSTICK_THROTTLE_SERVO_BRAKE_MIN` triggers braking; `toggleState[3]` activates handbrake. Brake light blinks faster as brake pressure increases, goes solid when stopped.

- **SteeringSystem** — Single servo (pin 21) mapped from joystick steering axis. Steering range is reduced at higher speeds via `STEERING_SPEED_SCALE_FACTOR`.

- **Dashboard** — Reads a 4-axis joystick (ADS1115 ADC channels), 4 toggle buttons (via PortExpander Port B), and drives 3 analog voltmeters via PWM channels 5/6/7. Button 4 (`toggleState[3]`) = handbrake; long-press button sequence triggers WiFi setup via callback.

- **WiFiPrinter** — Optional; activated by `dashboard.onRequestWiFi`. Serves `/getData` as JSON at 300 ms intervals for monitoring. Also handles OTA firmware updates. The companion HTML dashboard is `WiFiPrinter.html`. SSID/password are hardcoded in `ConstantDefinitions.h`.

- **LCDDisplay** — TFT via SPI (Arduino_GFX). Initialized before `SPI.begin()` because it calls `start()` independently; it targets 30 FPS (`DISPLAY_FPS`).

- **Buzzer / Horn** — Both are Singletons wrapping PortExpander output pins. `beep()`, `beep2()`, `beep3()` are non-blocking (managed in `update()`).

### Constants

All tunable values (servo min/max, speed limits, throttle mapping, timing intervals) are in `ConstantDefinitions.h`. All hardware pin assignments are in `PinDefinitions.h`. Change hardware mappings only in these two files.

## Conventions

- Singletons (`PortExpander`, `Buzzer`, `Horn`) are accessed via `::getInstance()` — never constructed directly.
- All subsystems check `initialized` before acting in `update()`.
- WiFiPrinter::print() is the debug log — messages accumulate in the JSON `"message"` field and are cleared after each HTTP GET to `/getData`.
