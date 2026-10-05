# Mission transcript and checkpoint recording

A mission is one SD session. The transcript declares a route and, for each leg or checkpoint, which sensors are active and whether audio is needed. The station should send the transcript before launch; the buoy should execute it without needing live Wi-Fi.

## Intended transcript

```yaml
mission_id: lake-survey-01
route:
  - id: transit-a
    destination: [47.000000, -122.000000]
    sensors: [gps, water_temperature]
  - id: checkpoint-a
    destination: [47.000100, -122.000100]
    arrival_radius_m: 3
    dwell_seconds: 45
    sensors: [gps, water_temperature, imu]
    audio:
      record: true
      sample_rate_hz: 16000
  - id: checkpoint-b
    destination: [47.000200, -122.000200]
    arrival_radius_m: 3
    dwell_seconds: 30
    sensors: [gps, water_temperature]
    audio:
      record: false
```

The mission log should record each instruction, transition, checkpoint arrival/departure, GPS fix, UTC time, monotonic timestamp, sensor selection, audio start/stop, and any error. GPS and temperature can continue during transit when declared. Each audio checkpoint should produce its own named WAV file, plus an index linking it to checkpoint ID, position, and timestamps. Missions with no audio checkpoints should have no audio payload. A pause should stop writes to the current audio chunk and a resume should begin a new chunk, retaining both within the same session. The final manifest should list every chunk and its checkpoint.

## Current implementation boundary

The current firmware creates one session and logs science/telemetry during manual mission capture. Audio starts disabled and can be toggled manually. Multiple audio intervals currently append to `audio.wav`; `audio.idx` keeps their timestamps, but the WAV alone does not encode the gaps. Checkpoint commands, transcript upload/execution, per-checkpoint WAV chunks, and automated sensor switching are not yet implemented. `CTRL SCIENCE STOP` ends capture, and `CTRL SCIENCE SAVE` closes the session; they are not pause/resume controls.

The planned alternate upload route is control station → router or TX Backpack access point → ExpressLRS TX Backpack MAVLink UDP → ELRS radio link → buoy receiver UART → buoy mission storage. The control station can discover the TX Backpack and listen for MAVLink battery, position, and channel telemetry on UDP `14550`. Neither production buoy firmware nor the station implements MAVLink mission transfer, and production firmware does not yet send MAVLink telemetry. The isolated CRSF bench sketch verifies radio control and telemetry only. Mission transfer will need a MAVLink receiver on the buoy, a defined mapping from the transcript to mission messages, and an acknowledged save before launch.

SD downloads enter service mode only after a mission is saved and closed. Service mode stops motors, pauses sensor polling and audio capture, disables heavy Wi-Fi streams, and exits after transfer or cancellation. A mission must never start or continue while service mode is active.

## Open decision: audio priority versus navigation

The active firmware slows non-GPS sensor sampling and science/telemetry logging to 30 seconds while SD audio recording is active, and enlarges the audio frame queue. GPS serial parsing and motor safety/control loops remain responsive, but the 30-second power/current readings and reduced IMU updates may be inadequate for autonomous navigation. The current firmware also uses 1x digital microphone gain after session019 showed severe clipping at 4x.

Before relying on this policy in autonomous missions, decide sensor rates by mission phase and checkpoint. GPS position, drift/current estimation, motor control, and any safety-critical current measurements must retain navigation-appropriate rates while underway. Audio can receive priority during stationary or low-navigation-demand checkpoint recording, or through a different scheduling policy that protects navigation. Review this during mission transcript design; do not treat the 30-second profile as the final autonomous mode behavior.
