# Software Resources

This folder is the software counterpart to the hardware component library. It records the tools, libraries, services, and important configuration used to build and operate the buoy.

- [Software inventory](software-inventory.md) — programs, libraries, online services, versions, purposes, and project evidence.
- [Configuration reference](configuration-reference.md) — reproducible build, communication, radio, and control-station settings.
- [Software behavior guide](system-behavior.md) — end-to-end data paths, control modes, mission start gates, and session/mission recording behavior.
- [MAVLink reference](mavlink-reference.md) — packet/message inventory, decoded field layouts, GPS source/precedence, and links to the raw API contract.
- [Router bridge API contract](../../src/router_bridge/API.md) — exact JSON schema and meanings for `/api/raw`, `/api/telemetry`, and `/api/state`.

## Recording Rule

When software, a control behavior, a telemetry field, an SD file format, a protocol, or a radio setting changes, update the matching inventory/configuration/behavior/protocol page in the same change. Firmware and application source remain the implementation source of truth; these pages explain the implemented contract and identify behavior that still needs field validation. Do not place passwords, private keys, binding phrases, student-identifying information, or account credentials in these documents.

Last verified against the working project files and current router deployment record: November 8, 2026. Development-machine software versions are snapshots; the router bridge versions are the field deployment record.
