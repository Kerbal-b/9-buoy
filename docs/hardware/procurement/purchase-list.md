# Buoy Project Purchase List

This file tracks component sources, links, and later can be expanded with prices, quantities, and order status.

For the current installed-system links synchronized from the live component sheet, use the [current component product-link register](../components/product-links.md). This older purchasing log retains candidates and replaced parts for history.

## Purchase Tracking Fields

- Component name
- Subsystem
- Store
- Link
- Price
- Quantity
- Status
- Notes

## Confirmed Purchase Links

### Movement

- Component name: GODIYMODULES dual-PWM MOSFET H-bridge motor driver, 5V-15V
- Subsystem: Movement
- Store: Amazon
- Link: https://www.amazon.com/dp/B0DT3NCP7Q
- Price: Not recorded yet
- Quantity: 3 modules required; listing contains 2 modules per pack
- Status: Selected; active rewiring in progress
- Notes: One controller per motor. Uses IN1/IN2 directional PWM from the ESP32. Advertised current is a peak rating; verify motor stall current and cooling.

- Component name: ICHIFIT RC jet-boat underwater motor/thruster, 7.4V, advertised 16800 RPM
- Subsystem: Movement
- Store: Amazon
- Link: https://www.amazon.com/dp/B07WY4MDYZ
- Price: Not recorded yet
- Quantity: 3 planned
- Status: Installed model confirmed; physical-unit photographs still requested
- Notes: Product image is stored in the component library. Record the order link and measure running/stall current before finalizing fuses and motor-driver cooling.

### Storage

- Component name: Mini TF / microSD SPI module
- Subsystem: Storage
- Store: Amazon
- Link: https://www.amazon.com/dp/B0F82XWT4F
- Price: Not recorded yet
- Quantity: 1 installed
- Status: Identified in the live component inventory
- Notes: Six-pad Mini TF breakout used by the current ESP32 SD wiring.

- Component name: 280 DC Mini Electric Motor, DC 3-12V, 1500-11000 RPM
- Subsystem: Movement
- Store: Amazon
- Link: https://www.amazon.com/dp/B0BP2T25H6?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1
- Price: Not recorded yet
- Quantity: Not recorded yet
- Status: Earlier candidate / relationship to current ICHIFIT thrusters not confirmed
- Notes: Retained for purchasing history. Do not treat this bare-motor listing as the active propulsion selection unless the installed ICHIFIT assemblies are confirmed to contain this exact motor.

- Component name: BOJACK L298N Motor DC Dual H-Bridge Motor Driver Controller Board
- Subsystem: Movement
- Store: Amazon
- Link: https://www.amazon.com/BOJACK-H-Bridge-Controller-Intelligent-Mega2560/dp/B0C5JCF5RS/ref=rvi_d_sccl_2/139-5259390-9872852?pd_rd_w=dsI7b&content-id=amzn1.sym.f5690a4d-f2bb-45d9-9d1b-736fee412437&pf_rd_p=f5690a4d-f2bb-45d9-9d1b-736fee412437&pf_rd_r=ADAKW8SYV5C7A1MX2ZM9&pd_rd_wg=0h9aq&pd_rd_r=9e1ada61-2d74-42d1-b497-a0ea0260d8de&pd_rd_i=B0C5JCF5RS&psc=1
- Price: Not recorded yet
- Quantity: Not recorded yet
- Status: Legacy / replaced
- Notes: Removed from the active ESP32 motor-control design when the three dual-PWM MOSFET bridges were selected.

### Communication

- Component name: DSD TECH HC-05 Bluetooth Serial Pass-through Module Wireless Serial Communication with Button for Arduino
- Subsystem: Communication
- Store: Amazon
- Link: https://www.amazon.com/dp/B01G9KSAF6?ref=ppx_yo2ov_dt_b_fed_asin_title
- Price: Not recorded yet
- Quantity: Not recorded yet
- Status: Name matched from link research
- Notes: Good fit for Bluetooth communication between the Arduino and the computer.

### Power

- Component name: HiLetgo 5pcs Voltage Detection Module DC 0~25V Voltage Sensor for Arduino
- Subsystem: Power
- Store: Amazon
- Link: https://www.amazon.com/dp/B01HTC4XKY?ref=ppx_yo2ov_dt_b_fed_asin_title
- Price: Not recorded yet
- Quantity: Not recorded yet
- Status: Name matched from link research
- Notes: Useful for battery-voltage monitoring.

### Hull And Mechanical

- Component name: MAKERELE outdoor electrical junction box with transparent hinged cover, IP67 waterproof ABS enclosure
- Subsystem: Hull And Mechanical
- Store: Amazon
- Link: https://www.amazon.com/dp/B09CMJZGS6?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1
- Price: Not recorded yet
- Quantity: Not recorded yet
- Status: Name matched from link research
- Notes: Useful as the waterproof electronics enclosure for the buoy.

## Pending Link Assignments

### 2026-04-18

- Component name: Pending confirmation
- Subsystem: Pending confirmation
- Store: Amazon
- Link: https://www.amazon.com/dp/B0DLWY4LJ6?ref=ppx_yo2ov_dt_b_fed_asin_title
- Price: Not recorded yet
- Quantity: Not recorded yet
- Status: Link saved
- Notes: Amazon page could not be cleanly identified from available automated access.

- Component name: Pending confirmation
- Subsystem: Pending confirmation
- Store: Amazon
- Link: https://amazon.com/dp/B0G56F1SKH?ref=ppx_yo2ov_dt_b_fed_asin_title
- Price: Not recorded yet
- Quantity: Not recorded yet
- Status: Link saved
- Notes: Amazon page could not be cleanly identified from available automated access.

- Component name: Pending confirmation
- Subsystem: Pending confirmation
- Store: Amazon
- Link: https://amazon.com/dp/B0FYCPQM3D?ref=ppx_yo2ov_dt_b_fed_asin_title
- Price: Not recorded yet
- Quantity: Not recorded yet
- Status: Link saved
- Notes: Amazon page could not be cleanly identified from available automated access.
