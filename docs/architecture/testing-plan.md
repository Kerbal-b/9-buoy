# Buoy Project Testing Plan

This file tracks how each subsystem should be tested.

> This is a working test checklist. Use [System Architecture](system-architecture.md) and the current [hardware wiring index](../hardware/wiring/README.md) for present-day hardware and connections. Old Arduino Nano, Bluetooth, STM32, and hydrophone-comparison examples have been removed or updated.

## Testing Stages

### 1. Bench Testing

Test single parts before putting them in the buoy.

Examples:
- ESP32 powers on and runs the current test firmware
- Motor driver responds to control signals
- Motors spin in the expected direction
- ESP32 Wi-Fi communicates with the laptop control station
- GPS reports coordinates
- Sensors return readable values

### 2. Subsystem Testing

Test one subsystem at a time.

Examples:
- Movement: forward, reverse, left, right, stop
- Communication: send commands and receive data
- Manual control: controller input reaches the buoy correctly
- Environmental data: readings display on the computer and save onboard
- Navigation: verify GPS position reporting; waypoint navigation and return-to-home remain future integration tests

### 3. Dry Integrated Testing

Test multiple systems together before water testing.

Examples:
- Manual commands move the motors correctly
- Sensor data sends to the computer while motors are active
- Power system remains stable under motor load
- Data storage works while communication is active

### 4. Pool Testing

Test the real buoy in controlled water.

Examples:
- Buoy floats and stays balanced
- Buoy moves forward in water
- Buoy turns in water
- Buoy responds to stop command
- Buoy sends data while floating
- If autonomous waypoint navigation is implemented, test movement toward a nearby GPS target

### 5. Full Demo Testing

Test the expected science fair demonstration sequence.

Examples:
- Start the buoy
- Send it to a target point
- Collect temperature and depth data
- Send data to the laptop
- Save a backup onboard
- Command the buoy to return

## First Priority Tests

- Confirm the ESP32 can command both directions on each dual-PWM MOSFET H-bridge while never activating both inputs together
- Confirm each motor spins correctly
- Confirm the three-motor layout can produce forward movement and turning
- Measure propulsion vibration with the onboard MPU6050 accelerometer
- Confirm the buoy body stays balanced with motors and battery installed
- Confirm manual movement commands can be sent from the computer
- Verify microSD logging, TDS readings on ADS1115 A2, turbidity readings on A3, and stereo audio capture while Wi-Fi telemetry is active
- Verify battery voltage and current monitoring under controlled propulsion loads
- When solar hardware is selected, measure charging performance and confirm the charging circuit cannot overcharge or back-feed the battery

## Propulsion Vibration Test

Use the onboard MPU6050 accelerometer to quantify vibration from the 3 mm propeller shafts and bevel-gear drive system.

1. Secure the buoy in the same fixture and orientation for every dry test, with guards around rotating parts.
2. Record a motors-off accelerometer baseline.
3. Run one motor at a time at repeatable low, medium, and high commands.
4. Repeat with useful combinations of two motors and all three motors.
5. Log the motor command together with the MPU6050 X, Y, and Z acceleration samples for a fixed duration at each test point.
6. Compare acceleration RMS and peak-to-peak values after removing the stationary gravity offset.
7. If the sampling rate is high and steady enough, inspect the frequency spectrum to identify shaft-speed, propeller, gear-mesh, or structural resonances.
8. Stop the test if vibration rises sharply, a shaft visibly runs out, a gear binds, or a propeller or coupling loosens.

Use the motors-off baseline and identical mounting conditions when comparing design revisions. Record the MPU6050 sample rate because low-rate control-station telemetry may show overall motion but miss high-frequency vibration.

## Test Logging Rule

- Record each important test in `notes/buoy-project-notes.md`
- Include the date, what was tested, what worked, what failed, and what should change next
