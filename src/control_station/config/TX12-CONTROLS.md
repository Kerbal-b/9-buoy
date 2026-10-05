# TX12 control plan

`tx12-channel-map.json` is the control station's editable display map. It records the intended buoy functions below so the future firmware channel decoder can use the same plan. Editing this file does not change EdgeTX mixes or buoy firmware.

The starter profile assumes a Mode 2 TX12 model with AETR output order. Confirm the output channels and directions in the active EdgeTX channel monitor before using them to command propulsion. The station visualizes fresh receiver-relayed values; it does not send these commands.

| Physical control | Channel | Intended buoy function |
| --- | ---: | --- |
| Right stick horizontal | 1 | Lateral movement, left/right component of the movement vector |
| Right stick vertical | 2 | Longitudinal movement, forward/reverse component of the movement vector |
| Left stick vertical | 3 | Motor power limit, bottom 0% to top 100%; scales every movement command |
| Left stick horizontal | 4 | Buoy yaw |
| SE, three positions | 5 | Position meanings pending |
| SF, three positions | 6 | Position meanings pending |
| SB, three positions | 7 | Position meanings pending |
| SC, three positions | 8 | Position meanings pending |
| SA, momentary | 9 | Low/high meanings pending |
| SD, momentary | 10 | Low/high meanings pending |
| S1 and S2 | Unassigned | Pending |

The two right-stick axes combine into a movement vector, so diagonal stick positions request movement at an angle. The left-stick vertical power limit applies to movement commands, including yaw. Exact mixing, dead zones, direction signs, and safe behavior on lost RC data still need to be specified and implemented in buoy firmware.

**SE on CH5 needs a radio-mode check.** [ExpressLRS MAVLink mode](https://www.expresslrs.org/software/mavlink/) allows Hybrid or 16ch/2 switch mode. In [Hybrid mode](https://www.expresslrs.org/software/switch-config/), CH5 is two-position and commonly serves as the arming channel, so the middle position of a three-position SE switch cannot be distinguished. Use a compatible Full Resolution 16ch/2 setup for three distinct CH5 states, and verify the configured ELRS arming method before assigning SE actions. The control station will still display three bins, but it cannot invent a missing middle value.

The station's stick bars use 988-2012 microseconds as an approximate visual range and show the raw received value. Switch indicators divide that range into low, middle, and high thirds; momentary buttons use low and high halves. These describe channel values, not guaranteed physical up/down positions. Set each switch position's `meaning` in the JSON file after deciding its command; leave it `null` until then. Verify physical direction against the TX12 output monitor.
