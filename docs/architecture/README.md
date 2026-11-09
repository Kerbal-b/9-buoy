# Buoy Architecture Documents

Use [System Architecture](system-architecture.md) for the current high-level design. The other files serve narrower purposes:

- [Physical Design](physical-design.md) records hull, propulsion geometry, and Fusion/build decisions.
- [Sketched Layout](sketched-layout.md) is a conceptual drawing and must not be used as the dimensioned CAD reference.
- [Testing Plan](testing-plan.md) lists subsystem and integration checks.

## Current System Groups

The project inventory groups components under Controller, Propulsion, Navigation, Science, Power, Storage, and Operator Equipment. The Power group includes battery supply, solar charging as a planned feature, and electrical monitoring. The listed battery is a 12 V 2600 mAh Li-ion pack with a 12.6 V 1 A charger. A separate 1 A, 12 V MPPT Li-ion/LiFePO4 charger module and a 5.5 V, 190 mA, 70 × 70 mm panel are identified; compatibility across the charger, panel, and pack is not verified. Regulators, fuses, and distribution design are not yet finalized.

## Sources Of Truth

- Installed parts and identification: [component inventory](../hardware/components/README.md).
- Current GPIO, buses, and sensor assignments: [firmware interface map](../hardware/components/firmware-interface-map.md), checked against the active [ESP32 firmware](../../src/firmware/esp32/esp32-buoy-firmware/esp32-buoy-firmware.ino).
- Current physical wiring: [hardware wiring index](../hardware/wiring/README.md).
- Hull dimensions and physical component placement: the latest Fusion model and measurements verified against the built buoy. Narrative notes here are not a substitute for that model.

## Known Stale Or Conflicting Notes

- The old controller trade-study text, Arduino Nano references, STM32 comparison, and hydrophone-processing rationale are superseded by the installed ESP32 and two INMP441 microphones.
- Bluetooth is not the active computer link. The control station uses Wi-Fi; the game controller connects to the laptop.
- The active ESP32 firmware supports direct TX12 control through the SkyGuy Nano receiver using ELRS MAVLink on UART2. Backpack Wi-Fi is for telemetry viewing and is not required for propulsion control. See the [production ELRS setup](../../src/firmware/esp32/esp32-buoy-firmware/ELRS-MAVLINK.md).
- The deployed GL.iNet Opal (`GL-SFT1200`) hosts a Python 3.6 standard-library MAVLink bridge on UDP 14550 and HTTP 8080. The phone page/API is read-only. Its successful HTTP/API response and iPhone page view are documented, but live GPS display and simultaneous QGC operation are not yet validated. See the [software behavior guide](../software/system-behavior.md) and [software inventory](../software/software-inventory.md).
- Mission start and mission-run behavior are implemented but not field-validated. The November 8, 2026 test's start attempts were blocked by unavailable calibrated heading; see [software behavior](../software/system-behavior.md) for the complete interlock sequence and the field evidence.
- pH and dissolved oxygen are not installed. Current water sensors are temperature, TDS, and turbidity. The JSN-SR04T measures an air gap to the surface, not underwater sonar depth.
- The top-view sketch's 26.9 degree duct angle is historical and must not be treated as current geometry. The physical-design log records later CAD measurements and also describes an active tangential motor mix; reconcile these against the current CAD and physical build before publishing one definitive geometry.
- The physical-design notes assume motors above the waterline with a gear-and-shaft transfer in places, while the current component inventory identifies three ICHIFIT underwater thrusters. Confirm the installed drive arrangement and update the CAD notes before using the older packaging assumptions.
- Battery and solar hardware are not fully specified. Do not charge the pack with the separate MPPT module until battery chemistry/cell configuration and charger limits are verified as compatible; verify that the 5.5 V panel meets the controller's input requirements. Use only the charger supplied for this pack unless its instructions specify otherwise. Battery monitoring does not mean the power-distribution or charging design is complete.
- Compass heading is implemented using the QMC5883 with stored calibration and mission-heading validity checks; it is not an unimplemented interface. However, reliable heading under motor load has not been established, and mission start was blocked by heading availability during the November 8, 2026 water test.
