# JSN-SR04T V3.0 Wiring Map

The installed depth/range sensor is a JSN-SR04T V3.0 waterproof ultrasonic module. The active firmware uses its default trigger/echo pulse-width mode.

Firmware source: `src/firmware/esp32/esp32-buoy-firmware/esp32-buoy-firmware.ino` (`JSN_SR04T_TRIG_PIN` and `JSN_SR04T_ECHO_PIN`).

## Installed Connections

| JSN-SR04T V3.0 pin | Connect to | Notes |
| --- | --- | --- |
| `VCC` | ESP32 `3.3 V` | The V3.0 specification permits `3.0–5.5 V`; 3.3 V operation keeps the echo signal in the ESP32-safe logic domain. |
| `GND` | ESP32/common `GND` | Required common reference. |
| `TRIG` / `RX` | ESP32 `GPIO13` | Firmware sends a `10 µs` high trigger pulse. |
| `ECHO` / `TX` | ESP32 `GPIO39` | Input-only pin used for pulse-width capture. |

```text
ESP32 3.3 V  ---> VCC
ESP32 GND    ---> GND
ESP32 GPIO13 ---> TRIG / RX
ESP32 GPIO39 <--- ECHO / TX
```

## Firmware Compatibility

The V3.0 specification calls for a trigger pulse of at least `10 µs`; the firmware sends exactly `10 µs`. It measures the returned echo width and converts round-trip travel time to millimetres. The module's documented blind zone is approximately `21 cm`, so readings closer than that are not valid.

The firmware currently uses a `30 ms` echo timeout. At the speed of sound used by the firmware, that limits a successful round-trip measurement to about `5.1 m`, slightly below the module's advertised `6 m` maximum.

## Scientific Limitation

The sealed probe is weather-resistant, but this is an ultrasonic measurement through air. It is not an underwater sonar transducer. For depth-style measurements, mount it above the water and measure the air gap to the surface, then calculate water level from a surveyed reference height. Do not immerse the probe unless documentation for the exact installed assembly explicitly permits that use.

## References

- [JSN-SR04T V3.0 product specification](https://www.laskakit.cz/user/related_files/jsn-sr04t-3_0-2.pdf)
- [Installed component product image](../components/images/jsn-sr04t-v3-product.jpg)
