# CanSat serial protocol

Both telemetry and commands are printable ASCII and end with a carriage return (`\r`).
The configured radio baud rate defaults to 9600 baud.

## Commands

Every command must use this envelope:

```text
TEAM_ID,COMMAND\r
```

`TEAM_ID` must exactly equal `CONFIG_CANSAT_TEAM_ID`. It may contain letters, digits,
hyphens, and underscores. Supported command payloads are:

- `CXON`
- `CXOFF`
- `SIM_ENABLE`
- `SIM_ACTIVATE`
- `SIM_DISABLE` (prelaunch only)
- `SP=<pressure_kPa>` or `SIM_PRESSURE,<pressure_kPa>`
- `MEC` or `MEC,<mechanism_name>` when the bench mechanism option permits it

Unknown, malformed, oversized, wrong-team, and out-of-range commands have no side
effects. Accepted commands appear in `CMD_ECHO`, with delimiters sanitized for CSV.

## Telemetry

Exactly 22 comma-separated fields are sent at 1 Hz while enabled:

```text
TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,ALTITUDE,TEMPERATURE,PRESSURE,
VOLTAGE,CURRENT,GYRO_R,GYRO_P,GYRO_Y,ACCEL_R,ACCEL_P,ACCEL_Y,
GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,CMD_ECHO\r
```

The physical record contains no embedded newline. Invalid and stale fields are `NA`.
GPS time, position, altitude, and satellite count have independent validity and
freshness. `PACKET_COUNT` increments only after successful radio handoff.
