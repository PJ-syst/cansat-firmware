# ESP32 CanSat firmware

Flight firmware and a Python ground station for an ESP-WROOM-32 CanSat payload. The
firmware is built with ESP-IDF 6.1 and combines environmental and inertial sensing,
GPS, radio telemetry, command handling, flight-state detection, and guarded one-shot
mechanism control.

> [!CAUTION]
> This repository is a development baseline, not flight-ready hardware. The deployment
> output and battery monitor are disabled by default. Keep the deployment load
> disconnected until every item in the
> [hardware configuration checklist](docs/HARDWARE_CONFIGURATION.md) has been verified.

## Features

- BMP280/BME280 pressure and temperature acquisition
- MPU6050 accelerometer and gyroscope acquisition
- NMEA GPS parsing with independent freshness checks for each GPS field
- UART radio transport with strict command validation
- 22-field CSV telemetry at 1 Hz
- Pressure simulation for flight-profile testing
- Launch, ascent, descent, deployment, and landing state detection
- One-shot, state-gated mechanism control with a safe default output
- Optional battery voltage monitoring
- Bounded sensor and UART recovery with periodic health diagnostics
- Python serial ground station with raw logging and live plots
- Host-side tests and GitHub Actions CI

## System overview

The ESP32 samples its sensors and feeds altitude measurements into a flight-state
machine. GPS and radio processing run independently. Once telemetry is enabled with a
`CXON` command, the latest readings are formatted as CSV and transmitted once per
second. A deployment request is accepted only in the permitted state and is consumed
after one attempt.

The main firmware components are:

| Component | Responsibility |
| --- | --- |
| `sensors` | BMP280/BME280 and MPU6050 acquisition |
| `gps` | GPS UART input and NMEA parsing |
| `radio` | Bidirectional UART radio transport |
| `command` | Command envelope validation and parsing |
| `flight_state` | Altitude-driven mission state transitions |
| `mechanism` | Safe, one-shot output control |
| `battery` | Optional ADC voltage measurement |
| `telemetry` | Fixed-width CSV packet formatting |

## Requirements

- ESP32 development board using an ESP-WROOM-32 module
- [ESP-IDF 6.1](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
- Python 3.11 or newer for the ground station
- A C11 compiler such as GCC for host-side firmware tests
- PowerShell 7 for the provided host-test script

## Configure, build, and flash

Open an ESP-IDF 6.1 terminal in the repository root, then run:

```console
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p COM5 flash monitor
```

Replace `COM5` with the ESP32 serial port. Hardware-specific values are under
**CanSat firmware** in `menuconfig`; at minimum, confirm the team ID, sensor addresses,
UART pins, baud rates, and all mechanism settings before flashing.

A successful build produces:

```text
build/cansat_firmware.bin
build/bootloader/bootloader.bin
build/partition_table/partition-table.bin
```

See [Hardware configuration](docs/HARDWARE_CONFIGURATION.md) before connecting the
final payload.

## Run the ground station

Create a virtual environment and install the two runtime dependencies:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -r ground_station\requirements.txt
python -m ground_station.cansat_ground_station --port COM5 --team-id TEAM1
```

Use the configured radio serial port and replace `TEAM1` with the firmware's team ID.
Add `--no-plot` for a logging-only console. Incoming packets are appended to
`telemetry-log.csv` before parsing, and commands typed into the console are sent as
`TEAM_ID,COMMAND\r`.

Supported commands include `CXON`, `CXOFF`, `SIM_ENABLE`, `SIM_ACTIVATE`,
`SIM_DISABLE`, `SP=<pressure_kPa>`, and the safety-gated `MEC` command. For exact
packet fields and command rules, see the [serial protocol](docs/PROTOCOL.md).

## Test

Run the portable C tests from PowerShell:

```powershell
.\host_tests\run_tests.ps1
```

Run the ground-station unit tests without connecting hardware:

```console
python -m unittest ground_station.test_ground_station
```

CI runs both test suites and builds the ESP32 firmware on every push and pull request.
Before hardware integration or a flight rehearsal, complete the
[reliability test plan](docs/RELIABILITY_TEST_PLAN.md).

## Repository layout

```text
components/       ESP-IDF firmware components
docs/             Protocol, hardware, and reliability notes
ground_station/   Python telemetry logger, plotter, and command console
host_tests/       Native C test runner and test cases
main/             Firmware entry point and project configuration
```

## Current status

Core firmware, host tests, the ground station, and CI are implemented. Hardware pin
assignments, calibration, radio parameters, mechanism electronics, and the competition
protocol still require validation on the final CanSat assembly.
