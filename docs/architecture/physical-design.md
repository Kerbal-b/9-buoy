# Buoy Project Physical Design

This file tracks the working physical design for the buoy in Autodesk Fusion.

> **Design log, not sole source of truth:** It contains dated alternatives and repeated layout notes. The latest Fusion model and measurements from the built buoy must resolve dimensions and motor axes. See [System Architecture](system-architecture.md) for current system status and known conflicts.

> **Check propulsion packaging:** Some older sections assume motors above the waterline with bevel gears and shafts; the current component inventory identifies three ICHIFIT underwater thrusters. Verify the as-built arrangement before relying on those placement and sealing assumptions.

## Purpose

Use this file to connect CAD work to the project plan.

After each Fusion design session, record:

- what was modeled or changed
- what dimensions were chosen
- what components were placed
- what assumptions were made
- what still needs testing in real life

## Fusion File Workflow

- Use Autodesk Fusion as the source for the 3D design
- Save screenshots in `assets/fusion/screenshots/`
- Save exported files in `assets/fusion/exports/`
- Use this file to explain the design decisions behind the model

## Current Physical Design Focus

- Buoy body shape
- Layered hull structure
- Motor placement
- Motor mount layout
- Waterproof enclosure placement
- Battery placement
- Balance and float stability
- Sensor mounting positions

## Current Physical Design Idea

The current concept is a three-layer buoy design with separation between underwater propulsion hardware, protected electronics, and top-mounted antennas and solar hardware.

### Bottom Layer

- Propellers
- Underwater instruments or sensors
- Gears, shafts, or other drive-transfer parts that connect the propellers to motors above the waterline
- Any submerged support structure needed to hold the propulsion system in alignment

### Middle Layer

- Waterproof electronics enclosure
- ESP32 development board
- Motor driver
- Battery
- Pool noodle flotation structure
- Main body support structure
- K'NEX prototype frame structure
- Wiring and service access

### Top Layer

- GPS antenna
- Space for communication antennas; the buoy uses Wi-Fi and a SkyGuy ExpressLRS receiver for direct TX12 control through production ELRS MAVLink firmware
- Solar panel
- Any top-mounted access cover or protective mounting plate

### Motor Placement

- Motors kept above the waterline
- Motors remain outside the sealed electronics box
- Wiring and service access

### Propulsion Layout

- Three ducted propellers arranged around the hull: two mirrored front ducts and one centered rear duct
- The outer circular frame is 190 mm in diameter
- The main hull is being reprinted with the revised front-motor geometry. This revised geometry is now the active project baseline; the earlier motor angles apply only to the superseded prototype hull.
- The front-left motor axis is measured from CAD at 16.9 degrees to the horizontal centerline
- With positive x drawn to the right, the same undirected front-left thrust axis can be written as -16.9 degrees or 163.1 degrees; forward and reverse thrust point in opposite directions along this line
- The front-left motor center is 97.02 mm from the vertical centerline and 42.157 mm from the horizontal centerline; both centerlines pass through the hull center
- The rear motor center is on the vertical centerline and 100 mm from the hull center
- These offsets are recorded as magnitudes until the project formally defines positive x and y directions
- The exact front-right motor angle and center offsets still need to be recorded from CAD; do not assume mirroring in final calculations until those values are confirmed
- This layout replaces the earlier assumption that all three propellers would be equally spaced at 120 degrees
- The front-right motor center, duct dimensions, propeller diameters, and all positive-thrust directions still need to be measured from the CAD model
- The goal is strong maneuverability, including turning and repositioning
- The layout should support omnidirectional-style control logic as much as the hull shape allows
- Compared with the previous 53.8-degree acute-separation layout, the revised axes appear to pass farther from the buoy center and should provide more yaw moment from the two front ducts, but the final improvement must be verified using exact CAD coordinates, confirmed positive-thrust directions, and a water test

### Active External Tangential Motor Layout

- This configuration is active in software following completion of the physical motor rebuild.
- Coordinate convention: positive x is right and positive y is forward.
- Rear water jet points left, giving a buoy-force axis of `(1, 0)` to the right.
- Front-left buoy-force axis: `(0.5, 0.8660254)`, or 60 degrees from positive x.
- Front-right buoy-force axis: `(-0.5, 0.8660254)`, or 120 degrees from positive x.
- The corresponding front water jets point back-left `(-0.5, -0.8660254)` and back-right `(0.5, -0.8660254)`.
- Pure positive yaw uses relative motor signs `rear +`, `front-left -`, `front-right +`; reversing all three signs reverses yaw.
- Because the thrust axes are tangential to the hull, their yaw moment arms are near their maximum values for the available motor-center radii.
- `USE_EXTERNAL_TANGENTIAL_MOTOR_LAYOUT` and `ACTIVE_MOTOR_MIX` now select this layout.
- Confirm the resulting movement directions with restrained, low-power tests before unrestricted operation.

### Propeller Gearboxes

- The previous worm-gear concept has been replaced by module 0.8, 15-tooth copper transmission bevel gears
- The propeller drive shafts are 3 mm in diameter; the earlier 2 mm shafts were too flexible and caused excessive vibration
- The exact bevel-gear pair ratio, shaft angle, backlash, and underwater lubrication or sealing method still need to be confirmed
- Gearbox direction and propeller retention must be checked under both forward and reverse motor operation

### Design Reasoning

- Keeping the motors above the waterline reduces water exposure risk
- Keeping only the propellers, underwater instruments, and drive-transfer parts below the waterline simplifies waterproofing of the sensitive electronics
- Keeping the electronics box in the middle layer protects the controller hardware while still allowing access to the drive system
- Using pool noodles as flotation is a simple first-prototype approach because they are cheap, buoyant, and easy to shape around the middle layer
- Using K'NEX as the structural frame should speed up early physical modeling and pool testing because the layout can be rebuilt quickly
- Keeping the GPS antenna and any planned solar panel on the top layer gives them better exposure and keeps them away from splash-heavy lower areas
- The current three-duct arrangement is intended to provide forward motion, repositioning, and yaw control through coordinated motor thrust

### Main Packaging Questions

- How the shafts or gear transfers will pass between the lower and upper layers
- How the hull will seal around those moving parts
- How much drag the lower layer will create
- Whether the center of mass stays low enough for stable floating
- How underwater sensors can be placed without interfering with propeller flow
- How the pool noodle flotation will be attached securely to the middle layer
- How rigid the K'NEX frame will remain under motor load
- Whether the top layer becomes too top-heavy once the solar panel and antennas are mounted
- How the solar panel will be protected from splash and impact

## Design Checklist

- Buoy can float with the full component load
- Motor layout allows forward movement and turning
- Electronics enclosure fits inside the body
- Battery placement supports stable balance
- Wiring paths are possible and protected
- Sensors can be mounted without interfering with propulsion
- The design is practical to build with available materials

## Subsystem Links

### Movement

- Motor positions
- Current front-left, front-right, and rear propeller-duct positions
- Propulsion direction
- Bevel-gear transfer path from upper motors to lower propellers
- Mounting brackets
- Turning layout

### Hull And Mechanical

- Body shape
- Layer separation between bottom, middle, and top hardware zones
- Float volume
- Pool noodle flotation layout
- K'NEX frame layout for rapid prototyping
- Waterproof enclosure mounting
- Seal design around any rotating shafts or drive-transfer parts
- Access for assembly and maintenance

### Power

- Battery location
- Solar panel location
- Cable routing
- Space for switch and voltage monitoring

### Environmental Data

- Sensor placement
- Water contact points
- Protection from splash and prop wash
- The DS18B20 water-temperature sensor now uses the `GPIO14` 1-Wire line.
- The INMP441 microphone data line uses `VP / GPIO36`, which keeps the audio sensor separate from the DS18B20 bus.

### Communication And Navigation Support

- GPS antenna position with clear sky exposure
- ExpressLRS antenna placement if/when the receiver is integrated
- Cable path from top-layer antennas down to the electronics box

## First Fusion Modeling Priorities

- Block out the layered body shape
- Place the front-left duct using the CAD measurement of 16.9 degrees from horizontal and center-offset magnitudes of 97.02 mm horizontally and 42.157 mm vertically
- Place the rear motor 100 mm from the hull center on the vertical centerline
- Confirm the exact front-right motor values before mirroring them
- Confirm the outer circular frame diameter is 190 mm
- Reserve motor space above the waterline and outside the enclosure
- Place the waterproof enclosure in the middle layer
- Add pool noodle flotation volume around the middle layer
- Use K'NEX geometry as the first support-frame concept
- Reserve top-layer space for the GPS antenna and possible solar panel; do not assume solar charging hardware is selected
- Check whether the battery can be placed low enough for stability
- Model the module 0.8, 15-tooth copper bevel-gear transfer paths between layers

## Current Assumptions

- The motors will remain above the waterline
- The motors will remain outside the sealed electronics box
- The propellers will remain below the waterline
- The propulsion system will use the current two-front/one-rear duct layout rather than equally spaced 120-degree drive positions
- The middle layer will carry the enclosure and use pool noodles for flotation
- The first physical support frame may be built from K'NEX to speed up prototyping
- The top layer is intended to carry the GPS antenna and may carry a solar panel; solar hardware remains unselected
- The layered design is being used to make waterproofing easier for the electrical system
- The first prototype should still be practical to build, not just theoretically maneuverable

## Session Log Template

Copy this section for each Fusion work session.

### YYYY-MM-DD

#### What I Modeled

-
#### Design Decisions

-
#### Dimensions Or Placement Chosen

-
#### Open Questions

-
#### Next Fusion Tasks

-
## Physical Build Log

### 2026-05-03

#### What Was Built

- Built the three exterior propellers for the buoy.
- Turned the planned triangular three-propeller layout into real physical parts.

#### Design Impact

- The movement system now has a physical propulsion set that can be checked against the exterior frame and layer layout.
- This makes it possible to validate propeller spacing, clearance, and mounting orientation on the real build instead of only in planning.

#### Next Physical Checks

- Measure the two front-duct angles and center coordinates from the final CAD model and the reprinted hull; use those measured values in movement-vector and thrust-allocation calculations.
- Confirm the rear duct center and thrust axis relative to the hull centerline.
- Check for interference with the frame, flotation parts, and any planned underwater sensor positions.
- Verify that the propeller mounting method is rigid enough for water testing.
- Check the 3 mm shafts for runout and alignment, then use the onboard MPU6050 accelerometer to measure vibration through the full motor-speed range.

### 2026-08-16

#### What Changed In The Design

- Changed the front propeller-duct layout from the earlier equal 120-degree-spacing concept.
- Set both mirrored front duct axes to 26.9 degrees from the horizontal reference line.
- Confirmed the outer circular frame diameter as 190 mm.
- Replaced the worm-gear gearbox concept with module 0.8, 15-tooth copper transmission bevel gears.
- Replaced the 2 mm propeller shafts with 3 mm shafts because the thinner shafts caused excessive vibration.

#### Dimensions Or Placement Chosen

- Outer circular frame: 190 mm diameter.
- Front-left duct axis: 26.9 degrees from horizontal.
- Front-right duct axis: 26.9 degrees from horizontal, mirrored from the left duct.
- Rear duct: centered on the vertical fore-aft centerline in the current top view.
- Propeller drive-shaft diameter: 3 mm.

#### Open Measurements

- Exact center coordinates of all three ducts.
- Duct and propeller diameters.
- Positive-thrust direction for each propeller.
- Bevel-gear ratio, shaft angle, backlash, and gearbox sealing details.
- Measured shaft runout and MPU6050 accelerometer vibration data across the operating speed range.

### 2026-09-06

#### What Changed In The Design

- Rotated the two mirrored front propeller ducts closer to horizontal in the latest top-view configuration.
- Replaced the previous estimated 53.8-degree acute front-axis separation with an estimated 30-degree separation.

#### Estimated Angles From The Screenshot

- Front-left duct axis: about 15 degrees from horizontal, or 75 degrees left of the forward centerline.
- Front-right duct axis: about 15 degrees from horizontal, or 75 degrees right of the forward centerline.
- Rear duct axis: aligned with the vertical fore-aft centerline.
- Front-axis acute separation: about 30 degrees.
- Supplementary separation through the vertical centerline: about 150 degrees.
- Screenshot-estimation uncertainty: approximately plus or minus 2 degrees.

#### Preliminary Yaw Assessment

- The revised front thrust lines appear to have a larger perpendicular offset from the buoy center than the previous layout, so differential front-motor thrust should create a stronger yaw moment.
- The improvement cannot be finalized from angles alone; exact duct-center coordinates and the positive-thrust direction of every propeller are still required for the motor mixer.
- Confirm the result with a restrained thrust test and an in-water yaw-rate test.

### 2026-09-07

#### Confirmed Motor Center Geometry

- Front-left motor axis: 16.9 degrees to the horizontal centerline.
- Front-left motor-center horizontal offset: 97.02 mm from the vertical centerline through the hull center.
- Front-left motor-center vertical offset: 42.157 mm from the horizontal centerline through the hull center.
- Rear motor center: 100 mm from the hull center on the vertical centerline.
- The offset signs remain undefined until the project establishes its positive x and y coordinate convention.
- The earlier approximately 15-degree screenshot estimate is superseded for the front-left motor by the CAD measurement.
- A provisional symmetric triangle places the front-right motor at mirrored offsets of 97.02 mm horizontally and 42.157 mm vertically. Confirm the right motor independently before using this assumption in the final thrust-allocation matrix.
