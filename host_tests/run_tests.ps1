$ErrorActionPreference = 'Stop'

$workspace = Split-Path -Parent $PSScriptRoot
$output = Join-Path $PSScriptRoot 'cansat_tests.exe'
$arguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror',
    "-I$workspace/components/cansat_types/include",
    "-I$workspace/components/command/include",
    "-I$workspace/components/flight_state/include",
    "-I$workspace/components/gps/include",
    "-I$workspace/components/telemetry/include",
    "$workspace/components/cansat_types/cansat_types.c",
    "$workspace/components/command/command_parser.c",
    "$workspace/components/flight_state/flight_state.c",
    "$workspace/components/gps/nmea_parser.c",
    "$workspace/components/telemetry/telemetry_formatter.c",
    "$PSScriptRoot/test_main.c",
    '-lm', '-o', $output
)

& gcc @arguments
if ($LASTEXITCODE -ne 0) { throw "gcc failed with exit code $LASTEXITCODE" }
& $output
if ($LASTEXITCODE -ne 0) { throw "tests failed with exit code $LASTEXITCODE" }
