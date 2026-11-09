# microSD Module Wiring Map

The active firmware uses a dedicated SPI bus and requests a `20 MHz` SD clock.

| microSD signal | ESP32 connection | Notes |
| --- | --- | --- |
| `CS` | `GPIO1` | Conflicts with UART0 TX and ordinary USB-serial logging. |
| `SCK` / `CLK` | `GPIO17` | GPS transmit is disabled so this pin can be SD clock. |
| `MOSI` / `DI` | `GPIO26` | ESP32 to card data. |
| `MISO` / `DO` | `GPIO35` | Card to ESP32 data; input-only pin is appropriate. |
| `GND` | ESP32/common `GND` | Required. |
| `VCC` | Supply required by the exact breakout | Verify whether the installed module expects `3.3 V` or has a regulator/level shifting for `5 V`. |

## Bench Constraint

Because `GPIO1` is also the normal UART0 transmit line, attaching this SD controller prevents normal serial-log use and may interfere with flashing. If upload fails, power down and disconnect the module's `CS` connection from `GPIO1`, upload, then reconnect it before the SD test.

The firmware continues operating when no card is available and skips file writes. Card availability and write statistics are included in its status/logging path.
