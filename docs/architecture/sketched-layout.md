# Buoy Layout Reference Status

## No verified placement sketch is available

The previous top and side views were removed because they showed unsupported propeller positions and an outdated drive arrangement. In particular, they depicted mirrored front ducts, a centered rear duct, and motors above the waterline driving submerged propellers through shafts. Those details do not provide a reliable drawing of the current buoy.

The project confirms three ICHIFIT underwater thrusters, logically identified as front-left, front-right, and rear in the motor wiring documentation. The current firmware selects a tangential motor mix. Neither fact establishes the exact as-built position, angle, spacing, or thrust direction of each physical propeller.

| Item | Current documentation status |
| --- | --- |
| Thruster count and product family | Three ICHIFIT underwater thrusters are identified in the component inventory. |
| Logical motor names and electrical connections | Front-left, front-right, and rear are mapped in the [motor wiring sheet](../hardware/wiring/motor-wiring-sheet.md). |
| Control mix | The active firmware defines a tangential motor mix; see [physical design notes](physical-design.md) and the firmware. |
| Physical propeller center coordinates and axes | Not verified in this document. |
| Duct/guard shape, clearances, and water-flow direction | Not verified; confirm from the current CAD and the built buoy. |
| Motor/propeller mounting arrangement | Do not infer from older shaft-and-gear concept notes. Confirm the installed assembly. |

The Fusion export [`Boui-Assembly-v2.f3d`](../../assets/fusion/exports/Boui-Assembly-v2.f3d) is available as a design artifact, but this page does not assert that it matches the latest as-built configuration. Treat the CAD model as a source to inspect and reconcile with the physical buoy, not as field-validated evidence by itself.

## What a replacement sketch must record

Before publishing a new spatial drawing, inspect the current Fusion model and compare it with clear top, side, front, and rear photographs of the assembled buoy. Record a coordinate convention and, for each thruster/propeller:

- its center relative to the hull origin;
- its thrust-axis angle and positive water-flow direction;
- the propeller/duct diameter and clearance;
- the corresponding firmware motor name and driver channel; and
- whether each measurement comes from CAD or the as-built buoy.

Until those measurements are reconciled, use the [component inventory](../hardware/components/README.md), [motor wiring sheet](../hardware/wiring/motor-wiring-sheet.md), and [system architecture](system-architecture.md) for the documented facts. None of those pages is a dimensioned physical placement drawing.
