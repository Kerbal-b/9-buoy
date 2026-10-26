# TX12 control plan

`tx12-channel-map.json` is the control station's editable display map. The production buoy firmware now decodes the first five ELRS channels over MAVLink; editing this file does not change EdgeTX mixes or buoy firmware.

The starter profile assumes a Mode 2 TX12 model with AETR output order. Confirm the output channels and directions in the active EdgeTX channel monitor before using them to command propulsion. The station visualizes fresh receiver-relayed values; it does not send these commands.

| Physical control | Channel | Intended buoy function |
| --- | ---: | --- |
| Right stick horizontal | 1 | Lateral movement, left/right component of the movement vector |
| Right stick vertical | 2 | Longitudinal movement, forward/reverse component of the movement vector |
| Left stick vertical | 3 | Motor power limit, bottom 0% to top 100%; scales every movement command |
| Left stick horizontal | 4 | Buoy yaw |
| SE, three positions | 5 | Low/middle disarm; high requests ELRS arm after low was seen |
| SF, three positions | 6 | Low/middle: buoy Wi-Fi telemetry; high: MAVLink telemetry through TX Backpack |
| SB, three positions | 7 | Low/middle: manual mode; high: start loaded mission (firmware support pending) |
| SC, three positions | 8 | Position meanings pending |
| SA, momentary | 9 | Low/high meanings pending |
| SD, momentary | 10 | Low/high meanings pending |
| S1 and S2 | Unassigned | Pending |

The two right-stick axes combine into a movement vector, so diagonal stick positions request movement at an angle. The left-stick vertical power limit applies to movement commands, including yaw. The production firmware uses a 40 microsecond center dead zone. CH3 now allows the full configured 255 PWM range; the three-motor mix and ramp determine each motor's actual output. To arm, first put SE low, then raise it with neutral movement sticks and the power stick fully down. A stale RC frame (500 ms) or fresh zero-link-quality report stops ELRS-driven motors, stops and saves active microSD recording, and requires another SE low-to-high arm cycle. Missing radio-status telemetry does not affect control when receiver control frames are fresh. Moving SE away from high disarms propulsion. Wi-Fi `CTRL STOP` also disarms the radio until SE cycles low again. Automatic stop is the only recovery action for now; no switch is reserved for recovery. Verify physical stick directions and CH5 values in the EdgeTX channel monitor with propellers removed before allowing propulsion.

SF only selects the telemetry route. Low and middle use buoy Wi-Fi for detailed sensors and power history. High uses low-rate MAVLink through the TX Backpack for battery/current, GPS, motor state, link status and channels. Buoy Wi-Fi can disconnect in either position without disarming TX12; the receiver's RC failsafe remains active. The TX12 model must actually output SF on CH6 for the switch to work. Verify CH6 in the EdgeTX channel monitor.

**SE on CH5 needs a radio-mode check.** [ExpressLRS MAVLink mode](https://www.expresslrs.org/software/mavlink/) allows Hybrid or 16ch/2 switch mode. In [Hybrid mode](https://www.expresslrs.org/software/switch-config/), CH5 is two-position, so the middle position of a three-position SE switch cannot be distinguished. The buoy treats values below 1300 microseconds as disarmed/ready and values above 1700 microseconds as an arm request. Intermediate values remain disarmed. Verify the configured ELRS arming method and actual CH5 output before using SE for propulsion.

The station's stick bars use 988-2012 microseconds as an approximate visual range and show the raw received value. Switch indicators divide that range into low, middle, and high thirds; momentary buttons use low and high halves. These describe channel values, not guaranteed physical up/down positions. Set each switch position's `meaning` in the JSON file after deciding its command; leave it `null` until then. Verify physical direction against the TX12 output monitor.

SB/CH7 selects manual or mission request: low and middle request manual authority; high requests mission mode. The firmware currently reports that request as unavailable and inhibits motor outputs, because the buoy has no tested waypoint guidance controller. Mission files can be read from the SD card, but selecting high does not start the mission. After returning SB to low or middle, cycle SE low then high to re-arm. Verify CH7 positions in the EdgeTX channel monitor before operation.
