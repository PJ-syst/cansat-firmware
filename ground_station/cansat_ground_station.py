#!/usr/bin/env python3
"""Validated serial telemetry logger, plotter, and command console."""

from __future__ import annotations

import argparse
import csv
import math
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from queue import Empty, Queue
from threading import Thread
from typing import Any

FIELD_NAMES = (
    "team_id", "mission_time", "packet_count", "mode", "state", "altitude_m",
    "temperature_c", "pressure_kpa", "voltage_v", "current_a", "gyro_r_dps",
    "gyro_p_dps", "gyro_y_dps", "accel_r_g", "accel_p_g", "accel_y_g",
    "gps_time", "gps_altitude_m", "gps_latitude_deg", "gps_longitude_deg",
    "gps_satellites", "command_echo",
)

FLOAT_FIELDS = set(FIELD_NAMES[5:16]) | set(FIELD_NAMES[17:20])
INTEGER_FIELDS = {"packet_count", "gps_satellites"}


def parse_telemetry(line: str, expected_team_id: str | None = None) -> dict[str, Any]:
    """Parse one 22-field packet, mapping the firmware's NA marker to None."""
    values = line.rstrip("\r\n").split(",")
    if len(values) != len(FIELD_NAMES):
        raise ValueError(f"expected 22 fields, received {len(values)}")
    packet: dict[str, Any] = dict(zip(FIELD_NAMES, values))
    if expected_team_id is not None and packet["team_id"] != expected_team_id:
        raise ValueError(f"unexpected team ID {packet['team_id']!r}")
    for name in FLOAT_FIELDS:
        packet[name] = None if packet[name] == "NA" else float(packet[name])
    for name in INTEGER_FIELDS:
        packet[name] = None if packet[name] == "NA" else int(packet[name])
    return packet


def format_command(team_id: str, command: str) -> bytes:
    """Create the canonical TEAM_ID,COMMAND carriage-return-terminated frame."""
    team_id = team_id.strip()
    command = command.strip()
    if not team_id or any(ch not in "-_" and not ch.isalnum() for ch in team_id):
        raise ValueError("team ID must contain only letters, digits, '-' or '_'")
    if not command or "\r" in command or "\n" in command:
        raise ValueError("command must be one non-empty line")
    return f"{team_id},{command}\r".encode("ascii")


class RawRecorder:
    def __init__(self, path: Path) -> None:
        self._file = path.open("a", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        if self._file.tell() == 0:
            self._writer.writerow(("received_utc", "raw_packet"))

    def write(self, raw_packet: str) -> None:
        self._writer.writerow((datetime.now(timezone.utc).isoformat(), raw_packet))
        self._file.flush()

    def close(self) -> None:
        self._file.close()


class LivePlot:
    def __init__(self, history: int = 300) -> None:
        import matplotlib.pyplot as plt

        self._plt = plt
        self._samples: deque[int] = deque(maxlen=history)
        self._altitude: deque[float] = deque(maxlen=history)
        self._temperature: deque[float] = deque(maxlen=history)
        self._pressure: deque[float] = deque(maxlen=history)
        plt.ion()
        self._figure, self._axes = plt.subplots(3, 1, sharex=True)
        self._figure.canvas.manager.set_window_title("CanSat ground station")

    def update(self, packet: dict[str, Any]) -> None:
        self._samples.append(int(packet["packet_count"] or 0))
        self._altitude.append(
            math.nan if packet["altitude_m"] is None else float(packet["altitude_m"])
        )
        self._temperature.append(
            math.nan if packet["temperature_c"] is None else float(packet["temperature_c"])
        )
        self._pressure.append(
            math.nan if packet["pressure_kpa"] is None else float(packet["pressure_kpa"])
        )
        for axis, values, label in zip(
            self._axes,
            (self._altitude, self._temperature, self._pressure),
            ("Altitude (m)", "Temperature (C)", "Pressure (kPa)"),
        ):
            axis.clear()
            axis.plot(tuple(self._samples), tuple(values))
            axis.set_ylabel(label)
            axis.grid(True)
        self._axes[-1].set_xlabel("Packet count")
        self._figure.canvas.draw_idle()
        self._plt.pause(0.001)


def _command_reader(commands: Queue[str]) -> None:
    while True:
        try:
            commands.put(input())
        except EOFError:
            return


def run(args: argparse.Namespace) -> int:
    import serial

    recorder = RawRecorder(args.log)
    plot = None if args.no_plot else LivePlot(args.history)
    commands: Queue[str] = Queue()
    Thread(target=_command_reader, args=(commands,), daemon=True).start()
    invalid_packets = 0
    print("Enter CXON, CXOFF, SIM_ENABLE, SIM_ACTIVATE, SIM_DISABLE, SP=<kPa>, or MEC.")
    try:
        with serial.Serial(args.port, args.baud, timeout=1) as connection:
            while True:
                try:
                    command = commands.get_nowait()
                    connection.write(format_command(args.team_id, command))
                except Empty:
                    pass
                raw = connection.read_until(b"\r", 320)
                if not raw:
                    continue
                try:
                    text = raw.decode("ascii", errors="strict").rstrip("\r\n")
                    recorder.write(text)
                    packet = parse_telemetry(text, args.team_id)
                    invalid_packets = 0
                    print(
                        f"#{packet['packet_count']} {packet['state']} "
                        f"alt={packet['altitude_m']}m gps=({packet['gps_latitude_deg']},"
                        f"{packet['gps_longitude_deg']})"
                    )
                    if plot is not None:
                        plot.update(packet)
                except (UnicodeError, ValueError) as error:
                    invalid_packets += 1
                    if invalid_packets == 1 or invalid_packets % 10 == 0:
                        print(f"Rejected telemetry packet ({invalid_packets}): {error}")
    except KeyboardInterrupt:
        return 0
    finally:
        recorder.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--team-id", required=True)
    parser.add_argument("--log", type=Path, default=Path("telemetry-log.csv"))
    parser.add_argument("--history", type=int, default=300)
    parser.add_argument("--no-plot", action="store_true")
    return parser


if __name__ == "__main__":
    raise SystemExit(run(build_parser().parse_args()))
