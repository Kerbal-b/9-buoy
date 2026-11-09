# Buoy Project Sketched Layout

This is a first-pass physical layout sketch for the layered buoy design.

> **Historical concept sketch:** This is not the current dimensioned design. The 26.9-degree front-duct value and the Arduino/Bluetooth-era hardware references below are superseded. Check the latest Fusion model, [Physical Design](physical-design.md), and [System Architecture](system-architecture.md) before using any placement or angle.

## Top View

```text
                    [ GPS Antenna ]
                         /\
                        /  \

                 ( Top equipment zone )
                  [ Solar Panel Area ]

          M1
           \
            \
             \         [ Electronics Box ]
              \        [ Battery Inside / Near Box ]
               \            [ Main Support Frame ]
                \________________________
                /                        \
               /      Pool Noodle Ring    \
              /____________________________\
             /              |               \
            /               |                \
           /                |                 \
         P1                 |                  P2
                            |
                            |
                           P3

Legend:
- `P1 P2 P3` = propeller positions
- `M1 M2 M3` = motor positions above waterline and outside the sealed box
- `P1` and `P2` are the mirrored front ducts; each duct axis is 26.9 degrees from horizontal
- `P3` is the rear duct, centered on the vertical fore-aft centerline in the current design
- The outer circular frame is 190 mm in diameter
- This current layout supersedes the earlier 120-degree equal-spacing sketch
- The center zone holds the enclosure and structural support
- Pool noodles wrap around the middle layer for flotation
```

## Simplified Top View With Labels

```text
                 M1
                  \
                   P1

       [ Pool Noodle Float Ring ]

             [ Electronics Box ]
             [ Battery / Driver ]

        P2                   P3
         \                   /
          M2               M3

       Top layer above center:
       [ GPS ]
       [ Bluetooth antenna? ]
       [ Solar panel ]
```

## Side View

```text
                 TOP LAYER
        __________________________________
       | GPS | BT? | Solar Panel Mounting |
       |__________________________________|

                 MIDDLE LAYER
        __________________________________
       | Pool Noodle Float Structure      |
       | Waterproof Electronics Box       |
       | Arduino / Driver / Battery       |
       | Motors above waterline           |
       |__________________________________|
                    ~ Waterline ~

                 BOTTOM LAYER
        __________________________________
       | Shaft / Gear Transfer Parts      |
       | Propellers                       |
       | Underwater Sensors               |
       | Lower Support Structure          |
       |__________________________________|
```

## Proposed Zone Layout

- Center: waterproof electronics box and main frame
- Around center: pool noodle flotation ring or segmented float pieces
- Top center: solar panel
- Top high point: GPS antenna
- Top side or edge: optional Bluetooth antenna
- Outer lower perimeter: two mirrored front ducts and one centered rear duct
- Just above waterline: three motors aligned to the drive-transfer paths

## First Fusion Modeling Order

1. Sketch the center electronics box footprint
2. Sketch the pool noodle flotation ring around it
3. Mark the mirrored front duct axes at 26.9 degrees from horizontal and the rear duct on the vertical centerline
4. Add motor positions above the waterline
5. Add rough top-layer mounting area for the solar panel and antennas
6. Test for clearance between propeller paths, sensor positions, and flotation
