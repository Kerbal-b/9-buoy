# Propulsion calibration design

The Propulsion tab currently applies each motor's PWM start boost, minimum, maximum, curve, ramp, and polarity through `CTRL MOTOR CAL`. The firmware applies those values in RAM. It does not yet save a propulsion profile to the buoy SD card or derive thrust from a water trial.

## Desired buoy configuration

Store a versioned propulsion profile in `/config/propulsion.cfg`, separate from `/config/navigation.cfg`. Include each motor's wiring direction, forward and reverse PWM limits, start boost, ramp, and measured response table. Upload the complete file, validate its version and checksum, write it to a temporary SD file, then atomically replace the active profile with a backup. Return the saved version and checksum so the control station can read it back. Load the active profile during buoy startup; reject invalid values and use the last valid profile or safe defaults. Apply a new profile only while the motors are stopped.

## Water calibration

1. With the buoy floating in open, bounded water, calibrate the waterline and magnetic heading first. Record GPS satellite count, HDOP, coordinates, heading, roll, pitch, gyro yaw rate, motor PWM, battery voltage, and current together with timestamps.
2. Run brief, supervised forward and reverse pulses on each motor at several safe PWM levels. Let the buoy settle between pulses and stop immediately when the operator releases the control.
3. Estimate translation from GPS displacement and yaw response from the magnetometer and gyro. Compare paired runs in opposite directions to reduce the effect of steady current and wind. Reject trials with poor GPS quality, large waves, or a stale heading signal.
4. Fit a bounded motor response profile and show the observed data before upload. The operator should be able to accept or edit the profile in the Propulsion tab.

GPS measures motion over the ground, so these trials estimate *installed propulsion response*, not absolute thrust in newtons. Water current, wind, hull drag, battery voltage, and waves affect the result. The mission controller can use the saved profile for an initial motor command, then adjust from fresh GPS position and magnetic heading feedback while staying inside the mission boundary.
