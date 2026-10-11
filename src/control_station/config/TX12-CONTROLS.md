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
| SF, three positions | 6 | Position meanings pending |
| SB, three positions | 7 | Position meanings pending |
| SC, three positions | 8 | Position meanings pending |
| SA, momentary | 9 | Low/high meanings pending |
| SD, momentary | 10 | Low/high meanings pending |
| S1 and S2 | Unassigned | Pending |

The two right-stick axes combine into a movement vector, so diagonal stick positions request movement at an angle. The left-stick vertical power limit applies to movement commands, including yaw. The production firmware uses a 40 microsecond center dead zone and caps ELRS motor commands at 50% while hardware directions are checked. To arm, first put SE low, then raise it with neutral movement sticks and the power stick fully down. A stale RC frame (500 ms), stale link status, zero link quality, or moving SE away from high stops ELRS-driven motors. Wi-Fi `CTRL STOP` also disarms the radio until SE cycles low again. Verify physical stick directions and CH5 values in the EdgeTX channel monitor with propellers removed before allowing propulsion.

**SE on CH5 needs a radio-mode check.** [ExpressLRS MAVLink mode](https://www.expresslrs.org/software/mavlink/) allows Hybrid or 16ch/2 switch mode. In [Hybrid mode](https://www.expresslrs.org/software/switch-config/), CH5 is two-position, so the middle position of a three-position SE switch cannot be distinguished. The buoy treats values below 1300 microseconds as disarmed/ready and values above 1700 microseconds as an arm request. Intermediate values remain disarmed. Verify the configured ELRS arming method and actual CH5 output before using SE for propulsion.

The station's stick bars use 988-2012 microseconds as an approximate visual range and show the raw received value. Switch indicators divide that range into low, middle, and high thirds; momentary buttons use low and high halves. These describe channel values, not guaranteed physical up/down positions. Set each switch position's `meaning` in the JSON file after deciding its command; leave it `null` until then. Verify physical direction against the TX12 output monitor.
