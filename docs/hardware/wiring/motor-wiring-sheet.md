# Motor Wiring Sheet

The active propulsion system uses three brushed DC motors and three independent GODIYMODULES/IBT-4-style MOSFET H-bridge modules. Each bridge has two directional PWM inputs. The former L298N boards and SN74HC14N inverter are not used.

## Control Logic

| IN1 | IN2 | Result |
| --- | --- | --- |
| `LOW` | `LOW` | Stop |
| PWM | `LOW` | Direction 1 with PWM speed control |
| `LOW` | PWM | Direction 2 with PWM speed control |
| `HIGH` | `HIGH` | Do not command; both inputs are never driven together by firmware |

The firmware operates at `20 kHz`, 8-bit PWM. It drives the inactive input low before applying PWM to the active input and pauses at zero during a commanded reversal. It also stops all motors immediately when Wi-Fi/TCP control is lost or when no control command/heartbeat arrives for `1000 ms`.

## Complete Signal Map

| Motor | Controller IN1 | Controller IN2 | Firmware polarity |
| --- | ---: | ---: | --- |
| Front left | ESP32 `GPIO22` | ESP32 `GPIO21` | Inverted |
| Front right | ESP32 `GPIO19` | ESP32 `GPIO18` | Normal |
| Rear | ESP32 `GPIO33` | ESP32 `GPIO32` | Inverted |

Each controller signal `GND` connects to ESP32 `GND`. Do not connect ESP32 `3.3 V` to the controller unless the exact received board has a separately labeled logic-power input and its documentation requires it.

## Power Distribution

```text
3S battery positive
  |
  +-- main fuse / disconnect switch
        |
        +-- branch fuse 1 --> front-left controller POWER+
        +-- branch fuse 2 --> front-right controller POWER+
        `-- branch fuse 3 --> rear controller POWER+

3S battery negative
  |
  +-- front-left controller POWER-
  +-- front-right controller POWER-
  +-- rear controller POWER-
  `-- ESP32 power-system ground / signal reference
```

Size branch fuses and power wiring from the measured motor stall current. Motor current must not pass through a breadboard, ESP32 board, or signal-ground wire. Join controller power negative and ESP32 ground at a deliberate common point.

## Front-Left Controller

```text
ESP32 GPIO22 ----------------------------> IN1
ESP32 GPIO21 ----------------------------> IN2
ESP32 GND -------------------------------> signal GND

Battery + through branch fuse 1 ---------> POWER+
Battery negative ------------------------> POWER-
Controller MOTOR terminal 1 -------------> front-left motor lead 1
Controller MOTOR terminal 2 -------------> front-left motor lead 2
```

## Front-Right Controller

```text
ESP32 GPIO19 ----------------------------> IN1
ESP32 GPIO18 ----------------------------> IN2
ESP32 GND -------------------------------> signal GND

Battery + through branch fuse 2 ---------> POWER+
Battery negative ------------------------> POWER-
Controller MOTOR terminal 1 -------------> front-right motor lead 1
Controller MOTOR terminal 2 -------------> front-right motor lead 2
```

## Rear Controller

```text
ESP32 GPIO33 ----------------------------> IN1
ESP32 GPIO32 ----------------------------> IN2
ESP32 GND -------------------------------> signal GND

Battery + through branch fuse 3 ---------> POWER+
Battery negative ------------------------> POWER-
Controller MOTOR terminal 1 -------------> rear motor lead 1
Controller MOTOR terminal 2 -------------> rear motor lead 2
```

## Polarity And Commissioning

The front-left and rear channels use tested software inversion; the front-right channel is non-inverted. These defaults reproduce the working configuration recorded during the September 23 motor tests. Motor terminal numbering does not establish the resulting water-flow direction, so confirm each motor individually at low power after rewiring.

1. Verify the received board labels for `POWER+`, `POWER-`, `MOTOR`, `IN1`, `IN2`, and signal `GND` before applying power.
2. Keep propellers clear and test one motor at a time at the lowest useful command.
3. Confirm that the control-panel motor name matches the physical motor.
4. Confirm the water-flow direction. If it is wrong, change either the motor leads or the firmware inversion, but not both.
5. Check controller temperature and supply current before increasing power.

During a control-station motor test, the application refreshes the selected motor command every `250 ms`. A stopped or frozen application therefore allows the firmware watchdog to remove motor drive automatically.

The previous confirmed physical assignment is retained: `GPIO22/21` is front left, `GPIO19/18` is front right, and `GPIO33/32` is rear.
