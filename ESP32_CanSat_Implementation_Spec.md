# ESP32 CanSat Demo Firmware — Implementation Plan

## Implementation status — 2026-09-04

The firmware baseline is implemented and compiles for `esp32` with ESP-IDF 6.1.
Host tests pass for the command parser, NMEA parser, telemetry
formatter, and flight-state machine. The mechanism and battery monitor remain
disabled by default, and no firmware has been flashed to hardware.

| Phase | Current status |
| --- | --- |
| Phase 0 — Interfaces/configuration | Software interfaces are defined; exact hardware values and the authoritative competition protocol remain open. |
| Phase 1 — Skeleton/logic | Implemented and host-tested. |
| Phase 2 — Drivers | Implemented for the provisional BMP280/BME280, MPU6050, NMEA UART, UART radio, ADC, and GPIO mechanism interfaces; physical bench validation remains open. |
| Phase 3 — Runtime plumbing | FreeRTOS tasks, queues, notifications, and mutex-protected snapshots are implemented; on-device timing validation remains open. |
| Phase 4 — Telemetry/radio | The 22-field bounded CSV formatter and 1 Hz task are implemented; live radio capture testing remains open. |
| Phase 5 — Commands/simulation | Implemented and parser-tested; an on-device scripted pressure-profile test remains open. |
| Phase 6 — State/mechanism | State logic and the one-shot request path are implemented and host-tested; inert-load testing and threshold tuning remain open. |
| Phase 7 — Reliability/rehearsal | Software fault handling, host tests, watchdog coverage, CI, and a rehearsal plan are implemented; physical endurance and flight-hardware tests remain open. |

The generated binary is a development baseline, not flight-ready firmware. The
hardware checklist and all relevant acceptance tests must be completed before
the mechanism is enabled.

## 1. Purpose and scope

This plan covers the firmware for the **ESP-WROOM-32 payload controller** used in the CanSat demo. It is an implementation plan, not source code. The firmware will:

- acquire barometer, IMU, GPS, and battery data;
- determine the flight state from validated altitude samples;
- transmit one ASCII CSV telemetry packet per second when telemetry is enabled;
- receive and execute the required ground commands;
- support real and simulated pressure inputs;
- release the payload mechanism once during descent; and
- fail safely when data or peripherals are unavailable.

A basic Python 3 ground-station logger/plotter accompanies the firmware. Mechanical
design, antenna/link-budget work, and FPGA/VHDL work remain outside this plan.

## 2. Decisions and hardware inputs required before coding

The architecture below is fixed. Hardware-specific values must be supplied in one project configuration header or through ESP-IDF Kconfig; they must not be scattered through task code.

| Configuration item | Required value |
| --- | --- |
| ESP-IDF version and target | ESP-WROOM-32 / `esp32`; pin the ESP-IDF version used by the team |
| Barometer | Exact part number, I2C address, bus number, SDA/SCL pins, and sample rate |
| IMU | Exact MPU6050 address, shared/dedicated I2C bus, interrupt pin if used, and sample rate |
| GPS | Module, UART number, RX/TX pins, baud rate, and enabled NMEA sentences |
| Radio | LoRa or XBee model, SPI/UART pins, reset/CS/DIO pins where applicable, and link settings |
| Battery monitor | ADC pin, divider resistor values, ADC attenuation, and current-sensor details if fitted |
| Mechanism | Servo or burn-wire, output pin, active level/PWM values, arm delay, and fire duration |
| Mission identity | Team ID and any competition-defined mode/state strings |

Until these values are confirmed, drivers will expose stable interfaces and use configuration placeholders. No pin number, voltage scale, actuator pulse, or RF setting should be guessed.

Default demo behavior must be safe:

- the mechanism output is inactive immediately at boot;
- automatic and command-triggered deployment are disabled until all required safety conditions are true;
- a build-time option controls whether the `MEC` bench command is permitted; and
- the physical actuator/load remains disconnected during early software tests.

## 3. Proposed project layout

Use ESP-IDF components so each subsystem can be initialized and tested independently.

```text
cansat_firmware/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   └── app_main.c
└── components/
    ├── cansat_config/
    ├── cansat_types/
    ├── sensors/
    ├── gps/
    ├── radio/
    ├── flight_state/
    ├── mechanism/
    └── telemetry/
```

Each component will have a public header, private implementation file(s), `CMakeLists.txt`, and a small test or test harness where practical. `app_main.c` owns initialization order, FreeRTOS object creation, and task creation; it must not contain sensor, parser, state-machine, or packet-formatting logic.

## 4. Shared types and ownership

Define fixed-width fields before task implementation. Use bounded character arrays and avoid dynamic allocation in steady-state flight code.

### `sensor_reading_t`

- relative altitude in metres (`float`);
- temperature in degrees Celsius (`float`);
- pressure in kPa (`float`);
- gyro R/P/Y in degrees per second (`float[3]`);
- acceleration R/P/Y in g (`float[3]`);
- sample timestamp in milliseconds since boot (`uint32_t`); and
- validity flags for barometer and IMU data.

The R/P/Y names are required by the telemetry format. The sensor wrapper must document which physical X/Y/Z axis maps to each field.

### `gps_data_t`

- UTC time in a fixed-size representation;
- latitude and longitude in decimal degrees;
- GPS altitude in metres;
- satellite count (`uint8_t`);
- fix-valid flag; and
- timestamp of the last valid update.

Only checksum-valid `$GPGGA`/`$GNGGA` and `$GPRMC`/`$GNRMC` data may update this snapshot.

### Other shared types

- `flight_state_t`: `STARTUP`, `LAUNCH_PAD`, `ASCENT`, `DESCENT`, `MECH_RELEASED`, `LANDED`, and `ERROR`.
- `command_t`: parsed command kind plus a typed payload for commands that carry a value.
- `telemetry_packet_t`: team ID, mission time, packet count, mode, state, latest sensor/GPS values, voltage/current, and command echo.
- `system_status_t`: initialization results, telemetry enabled state, simulation enabled/active state, last errors, and mechanism armed/fired latches.

Ownership rules:

- `SensorSampleTask` is the only writer of sensor snapshots.
- `GPSTask` is the only writer of the GPS snapshot.
- `FlightStateTask` is the only writer of the flight state.
- `CommandTask` is the only writer of command/simulation mode and command echo.
- `MechanismTask` is the only code allowed to drive the mechanism output.
- `TelemetryTask` is the only owner of `packet_count`.

## 5. Runtime architecture and data flow

Create the following FreeRTOS objects in `app_main` before starting tasks:

| Object | Type and size | Producer → consumer | Behavior |
| --- | --- | --- | --- |
| `sensorLatestQueue` | Queue, length 1, `sensor_reading_t` | SensorSampleTask → TelemetryTask | Use overwrite semantics so telemetry always obtains the newest sample. |
| `altitudeQueue` | Queue, sized for at least two state-analysis windows | SensorSampleTask → FlightStateTask | Carries every validated altitude sample. Log an overrun; never silently substitute invalid data. |
| `gpsMutex` | Mutex | GPSTask ↔ TelemetryTask | Protects a shared latest-GPS snapshot copied during a short critical section. |
| `commandQueue` | Bounded queue of raw command frames | radio RX path → CommandTask | Frames have a fixed maximum length and are always null-terminated before parsing. |
| `radioMutex` | Mutex | radio TX/RX tasks | Protects a shared SPI/UART transaction where the selected module requires it. |
| `stateMutex` | Mutex | FlightStateTask ↔ readers | Protects a shared flight-state/status snapshot. |
| `mechanismNotify` | Direct task notification | FlightStateTask/CommandTask → MechanismTask | Event bits distinguish automatic release from an allowed bench command. |
| `mechanismResultQueue` | Queue, length 1 | MechanismTask → FlightStateTask | Reports the one-shot release result so FlightStateTask remains the only state writer. |

Do not use one queue as if it broadcasts a sample to multiple consumers: receiving removes the item. The separate latest-sensor and altitude queues make both paths deterministic.

### Task set

| Task | Priority tier | Responsibility |
| --- | --- | --- |
| `SensorSampleTask` | High | Periodic barometer/IMU acquisition, calibration, validation, and publication to both sensor queues. Use `vTaskDelayUntil` or a data-ready notification. |
| `FlightStateTask` | High | Ground calibration, altitude history, state transitions, and the one-shot automatic-release request. |
| `CommandTask` | High | Strict command parsing, validation, side effects, and command echo. |
| `MechanismTask` | High | Validate safety gates, arm/fire once, report the result, and return output to its safe state. It blocks when idle. |
| `RadioRxTask` | Medium/High | Obtain complete frames from the selected radio and enqueue commands. For LoRa, a DIO ISR only notifies this task; the task performs the SPI read. |
| `GPSTask` | Medium | Consume UART events, frame and checksum NMEA sentences, and update the GPS snapshot. |
| `TelemetryTask` | Medium | At 1 Hz, copy snapshots, format one bounded CSV record, transmit it when enabled, and update packet count. |

ISRs must only acknowledge hardware and notify a task or enqueue a small event with the `FromISR` APIs. They must not parse NMEA, format telemetry, read a LoRa packet over SPI, log extensively, or drive the mechanism.

## 6. Driver work

### Barometer

- Verify device identity and load its factory calibration data during initialization.
- Configure the selected oversampling/filter/rate.
- Convert raw readings with the exact datasheet compensation formula.
- Calculate altitude relative to the calibrated ground reference.
- Expose both real pressure and the selected pressure source used by the flight logic.
- Reject bus errors, non-finite values, out-of-range pressure, and implausible step changes.

### MPU6050

- Verify identity, wake the device, and configure accelerometer/gyro ranges and sample rate.
- Convert raw data to g and degrees per second.
- Determine and document the payload-axis mapping.
- A complementary filter may be added only if the demo requires estimated attitude; raw converted axes remain the source for required telemetry fields unless the competition definition says otherwise.

### GPS

- Use the ESP-IDF UART driver event queue rather than a busy polling loop.
- Bound the NMEA sentence buffer and discard overlong or checksum-invalid input.
- Parse UTC, fix status, latitude, longitude, altitude, and satellite count.
- Preserve the last valid position while marking it stale after a configurable timeout.

### Radio

- Hide LoRa/XBee differences behind `radio_init`, `radio_send`, and `radio_receive` interfaces.
- Apply a maximum frame length and timeout to all operations.
- Serialize bus access only when the chosen module/driver needs it.
- Report send failures to system status without blocking the 1 Hz task indefinitely.

### Battery monitoring

- Use ESP-IDF ADC calibration when supported by the board/eFuse data.
- Average a small fixed number of samples, then apply the configured divider scale.
- Return an explicit unavailable/invalid status if no current sensor is installed; do not invent a current value.

### Mechanism

- Provide separate internal operations for safe initialization, arm, fire, and force-safe.
- Enforce a maximum energization/PWM duration independent of the caller.
- Reject every request after the fired latch is set.
- Accept an automatic request only when the shared state is `DESCENT`. Accept `MEC` outside descent only in the explicitly enabled bench configuration; all other fire requests are rejected.
- Record the request source, timestamp, and outcome.

## 7. Flight-state behavior

All transitions are implemented in one state-machine module and tested with recorded/synthetic altitude sequences. Thresholds are named configuration values, not magic numbers.

1. `STARTUP`: initialize hardware and collect health results. Proceed to ground calibration only when the barometer is usable; otherwise enter `ERROR`.
2. `LAUNCH_PAD`: average the first configurable number of valid, stable pressure/altitude samples to establish ground altitude. Transition to `ASCENT` only after relative altitude remains above the launch threshold (initial demo value: 10 m) for the configured confirmation count.
3. `ASCENT`: maintain a fixed-size rolling altitude window. Transition to `DESCENT` only when its trend is negative beyond the configured noise margin for multiple windows; a single lower sample cannot trigger deployment.
4. `DESCENT`: on entry, submit exactly one automatic-release request. After `MechanismTask` reports success, transition to `MECH_RELEASED`. Detect landing only when altitude stays inside the configured band (initial demo value: ±2 m) and vertical trend is near zero for a sustained interval.
5. `MECH_RELEASED`: continue descent/landing evaluation without allowing another release.
6. `LANDED`: remain terminal until reboot. Continue or reduce telemetry only if the final competition rules permit it.
7. `ERROR`: keep the mechanism safe, continue diagnostic telemetry if the radio is usable, and never infer flight transitions from invalid altitude.

The state machine consumes pressure-derived altitude from the active source. Simulated pressure must pass through the same conversion, validation, queue, and state-machine path as real pressure so simulation tests the real flight logic.

The mechanism fired latch is separate from the flight state and remains set for the rest of the power cycle. A state error or re-entry therefore cannot fire it again.

## 8. Command behavior

Accept only a complete, bounded command frame with the expected team identifier and syntax defined by the ground-station protocol. Matching is exact after removing permitted line termination; malformed, truncated, unknown, or out-of-range commands have no side effects.

Required command effects:

| Command | Effect |
| --- | --- |
| `CXON` | Enable the 1 Hz telemetry transmission cadence and establish the elapsed-time fallback epoch. |
| `CXOFF` | Disable radio transmission. Sampling, command reception, state tracking, and safety tasks continue. |
| `SIM_ENABLE` | Permit simulated pressure input but do not select it yet. |
| `SIM_ACTIVATE` | Select simulated pressure only after simulation has been enabled. |
| `SIM_DISABLE` | Return to real pressure before launch and clear the stored simulated sample. |
| `SIM_PRESSURE` / `SP<value>` | Validate and store a pressure value in the protocol's required units. Convert to kPa at the parser boundary if necessary. |
| `MEC` | Request a named mechanism only when the bench-command build option and runtime safety gates permit it. It still passes through `MechanismTask` and the one-shot latch. |

After processing, update `CMD_ECHO` with a sanitized representation of the most recently accepted command. Decide the exact external spelling from the competition command document before implementing the parser; aliases must not be added casually.

## 9. Telemetry packet contract

At each enabled 1 Hz transmission tick, format exactly one comma-delimited ASCII record, terminated by carriage return (`\r`), in this order:

```text
TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,ALTITUDE,TEMPERATURE,PRESSURE,
VOLTAGE,CURRENT,GYRO_R,GYRO_P,GYRO_Y,ACCEL_R,ACCEL_P,ACCEL_Y,
GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,CMD_ECHO\r
```

The actual transmitted record has no embedded newline; the line break above is only for readability.

Formatting rules:

- `MISSION_TIME` uses GPS UTC after a valid fix/time is available. Before that, it uses elapsed time from the most recent `CXON` epoch, with one-second resolution.
- `PACKET_COUNT` is a `uint32_t`, starts from the required initial value, increments once after each successfully handed-off telemetry packet, never decreases while powered, and resets only on reboot.
- `MODE` reports real flight or simulation mode with the exact required string.
- `STATE` is a human-readable state string, not an enum number.
- Define precision per field in one formatter specification (initial proposal: altitude 0.1 m, temperature 0.1 °C, pressure 0.01 kPa, voltage/current 0.01, IMU 0.01, latitude/longitude at least 5 decimal places).
- Invalid or unavailable values use one documented protocol-safe representation; they must not be confused with a genuine zero.
- `CMD_ECHO` may not contain commas, carriage returns, or line feeds.
- Use a fixed-size output buffer and check the formatter result for truncation before calling `radio_send`.
- Capture all shared snapshots near the start of the tick so one packet is internally consistent.

Use `vTaskDelayUntil` for the 1 Hz cadence. A delayed or failed transmission must not cause a burst of catch-up packets.

## 10. Startup, fault handling, and observability

Startup order:

1. Set the mechanism pin to its safe state before initializing other peripherals.
2. Initialize non-volatile/configuration services and create all synchronization objects.
3. Initialize I2C, UART, SPI/ADC, then each device driver.
4. Record a self-test result for every expected device.
5. Start receive, acquisition, state, command, mechanism, and telemetry tasks.
6. Calibrate ground altitude and enter `LAUNCH_PAD` only after stable valid samples.

Fault policy:

- A missing/failed barometer is critical: enter `ERROR` and inhibit automatic mechanism release.
- IMU, GPS, battery-current, or radio failures are reported independently and do not fabricate data.
- Sensor failures retain the last good telemetry value with a stale/health indication in logs/status; invalid samples never enter the state machine.
- Repeated queue overruns, formatter truncation, and radio timeouts are counted and logged at a rate limit.
- Add critical tasks to the ESP-IDF task watchdog only after their normal blocking behavior and watchdog feed points are defined. A task waiting legitimately on a queue/notification must not cause resets.
- Avoid unbounded waits, recursion, and steady-state heap allocation.

Log state changes, accepted/rejected commands, sensor health changes, queue overruns, radio failures, and mechanism requests/outcomes. Do not log every high-rate sample in the final demo build.

## 11. Phased implementation plan

Each phase has a deliverable and an exit check. Later phases should not hide failures from earlier ones.

### Phase 0 — Freeze interfaces and configuration

- Confirm the hardware/configuration table in Section 2.
- Obtain the authoritative telemetry/command protocol and resolve command spellings, units, team ID, packet-count initial value, and invalid-field representation.
- Define shared structs, enums, component APIs, task rates, queue sizes, thresholds, and safety gates.
- Produce a pin/bus allocation table and check ESP32 boot-strapping/input-only pin restrictions.

**Exit check:** no unresolved value is embedded as an assumed pin, voltage, actuator timing, RF parameter, or external protocol rule.

### Phase 1 — Project skeleton and host-testable logic

- Create the ESP-IDF component layout and configuration module.
- Implement pure modules for NMEA checksum/parsing, command parsing, pressure-to-altitude conversion, flight-state transitions, and CSV formatting.
- Add tests for valid, invalid, boundary-length, stale, and non-finite inputs.

**Exit check:** parsers, formatter, and state machine pass their test vectors without physical hardware.

### Phase 2 — Hardware drivers, one at a time

- Bring up I2C and verify barometer identity, calibration, and plausible readings.
- Bring up the MPU6050 and verify axis/range conversion.
- Bring up GPS UART and verify recorded/live NMEA parsing.
- Bring up the radio with a loopback or second node.
- Calibrate voltage/current readings against a meter.
- Verify the unpowered/disconnected mechanism control waveform and timeout.

**Exit check:** each driver has a recorded bench result and returns explicit errors instead of stale/uninitialized values.

### Phase 3 — Acquisition and shared-data plumbing

- Create tasks and FreeRTOS objects from Section 5.
- Publish validated sensor samples to the overwrite queue and altitude stream.
- Update/copy GPS and state snapshots under their mutexes.
- Instrument queue high-water/overrun and task timing.

**Exit check:** consumers receive fresh, coherent data at the planned rates with no queue misuse or unbounded blocking.

### Phase 4 — Telemetry and radio operation

- Assemble the exact CSV record at 1 Hz.
- Verify field order, precision, time fallback/GPS switchover, sanitization, termination, maximum length, and packet-count behavior.
- Exercise `CXON`/`CXOFF` without stopping other tasks.
- Measure transmit duration and handle radio timeouts without cadence drift.

**Exit check:** a captured radio stream passes an automated golden-packet parser for a multi-minute run.

### Phase 5 — Commands and simulation

- Implement strict command framing/parsing and command echo.
- Implement the simulation enable → activate → pressure sequence.
- Route simulated pressure through the normal altitude/state pipeline.
- Test malformed, duplicated, burst, out-of-range, and wrong-team commands.

**Exit check:** rejected commands have no side effects, and a scripted pressure profile drives the expected telemetry and states.

### Phase 6 — Flight state and mechanism integration

- Tune launch, descent, and landed confirmation thresholds with recorded or scripted profiles.
- Verify state transitions under noise, plateaus, dropouts, and implausible jumps.
- First test release requests with the actuator disconnected; then use a safe inert load.
- Verify automatic and permitted bench requests converge on the same one-shot mechanism path.

**Exit check:** every test profile produces the expected state sequence, and repeated/re-entered/concurrent release requests can energize the output no more than once per boot.

### Phase 7 — Reliability and demo rehearsal

- Run startup with each noncritical peripheral absent and with the barometer absent.
- Exercise command bursts, full queues, GPS loss/recovery, radio timeouts, I2C errors, sensor stalls, and near-counter/buffer boundaries.
- Measure stack high-water marks and heap before/after a long run; adjust static sizes with margin.
- Enable and validate the task watchdog configuration.
- Conduct an end-to-end rehearsal using the exact demo power source, radio settings, sensor rates, and inert mechanism setup.

**Exit check:** the complete acceptance checklist below passes twice from a cold boot without manual firmware intervention.

## 12. Demo acceptance checklist

- Safe mechanism output is observable from reset through startup.
- Ground altitude calibrates only from stable, valid readings.
- Live sensor and GPS fields update correctly; loss of a source is visible and does not fabricate zeroes.
- Telemetry starts/stops with `CXON`/`CXOFF` and remains at 1 Hz without accumulated drift.
- Every packet has 22 fields in the specified order and ends in `\r`.
- Mission time switches cleanly from elapsed fallback to GPS UTC.
- Packet count is monotonic for the power cycle.
- Simulation cannot activate before enable and uses the same flight-state path as real pressure.
- Noisy/single bad altitude samples do not create a state transition.
- The scripted profile reaches `LAUNCH_PAD → ASCENT → DESCENT → MECH_RELEASED → LANDED`.
- The mechanism cannot fire more than once per boot, including repeated `MEC` commands and state re-entry.
- A critical barometer failure inhibits deployment and produces `ERROR` behavior.
- No task watchdog reset, stack warning, growing heap use, queue overflow, or buffer truncation occurs during the planned demo duration plus margin.

## 13. Documentation deliverables

At completion, retain:

- confirmed pin/bus/configuration table;
- shared API and field/unit definitions;
- telemetry and command protocol examples;
- state-transition diagram and configured thresholds;
- mechanism safety checklist;
- driver bench-test records and calibration values;
- automated test vectors and results; and
- demo startup, operation, and recovery procedure.
