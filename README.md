# Buoy Project

Project folders:

- `docs/` final project documents and polished reference material
- `architecture/` working system design, subsystem planning, and testing design
- `src/` working files, scripts, or source code
- `assets/` images, media, and design resources
- `notes/` scratch notes, meeting notes, and planning
- `archive/` old or inactive material

Markdown navigation:

- `markdown-guide.md` entry point for all project Markdown files by task type

Core project documents:

- `docs/Buoy Project.gdoc`
- `docs/components-list.md`
- `docs/purchase-list.md`

Working design documents:

- `architecture/system-architecture.md`
- `architecture/testing-plan.md`
- `architecture/physical-design.md`

Project subsystem structure:

- `Movement` motors, motor drivers, propellers, thrust layout, steering, and motor mounting
- `Communication` wireless link between buoy and computer, data sending, and command receiving
- `Navigation` GPS-based movement to a target point, target holding, and return behavior
- `Manual Control` commands sent from a computer, with possible game-controller input through the computer
- `Environmental Data` water temperature, depth, pH, and other possible water-quality sensors
- `Power` battery, voltage regulation, power distribution, runtime, and power safety
- `Hull and Mechanical` buoy body, waterproofing, floatation, stability, mounting, and cable routing
- `Testing` bench testing, pool testing, subsystem tests, and full-system tests
- `Presentation` science fair demo, poster, slides, and demonstration plan

Source code structure:

- `src/firmware/arduino/` Arduino Nano production firmware
- `src/firmware/esp32/` ESP32 production firmware
- `src/firmware/esp32-test/` ESP32 bring-up and protocol test firmware
- `src/control_station/` laptop program that will connect to Arduino by Bluetooth and handle controller-based manual control

Project working rule:

- At the end of each meeting or work session, save a dated summary in a separate file under `notes/sessions/` using the format `YYYY-MM-DD.md`.
- If multiple work sessions happen on the same day, keep them in the same dated file as separate sections.
- Each dated session file should include what was worked on, completed tasks, and short meeting notes.
- Keep `notes/buoy-project-notes.md` as the index and workflow reference for the session-note system.
- When a new project rule or workflow decision is created during work on this project, update `README.md` so the rule is preserved for future sessions.
- The project can be worked on in parallel across two tracks: electronic design and outside design.
- The first movement design choice is to use three motors so the buoy can turn and reposition itself while navigating to a GPS location.
- The motor system will use Arduino control, motor drivers, and three brushed DC motors.
- Early work should include both physical motor layout design and Arduino connectivity testing.
- The recommended build order is: movement, manual control, communication, environmental data, navigation, then full-system testing.
- Use a project-local Python virtual environment for `src/control_station/` so laptop-side dependencies do not pollute the global Python installation.
- Use `docs/` for final documents and stable reference files.
- Use `architecture/` for in-progress system design, architecture planning, and testing structure.
- Keep a running purchase list in `docs/purchase-list.md`.
- When a product link is provided, save the store link in `docs/purchase-list.md` so the source can be referenced later.
- Use `architecture/physical-design.md` to sync Autodesk Fusion design work back into the project.
- Save Fusion screenshots in `assets/fusion/screenshots/` and exported files in `assets/fusion/exports/`.
- Codex comands: codex --oss --sandbox workspace-write
- codex --oss -m qwen2.5-coder:3b --sandbox workspace-write
- codex --oss -m qwen2.5-coder:7b --sandbox workspace-write
- codex --oss -m phi3 --sandbox workspace-write
