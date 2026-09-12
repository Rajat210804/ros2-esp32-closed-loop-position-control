"""Line-based serial protocol between the ROS 2 host and the ESP32 axis firmware.

Deliberately plain ASCII so you can open a serial monitor and drive the axis by
hand while debugging. Every frame is one newline-terminated line.

Host -> MCU
    H                 home (seek limit switch, zero the encoder)
    G <counts>        go to absolute position, in encoder counts
    K <kp> <ki> <kd>  set PID gains
    E <0|1>           disable / enable the motor driver
    S                 stop where you are (target := current position)

MCU -> host
    T <us> <pos> <target> <err> <pwm> <homed>   telemetry, streamed at fixed rate
    A <text>                                     acknowledgement / log line
    X <text>                                     fault
"""

from __future__ import annotations

from dataclasses import dataclass


# --------------------------------------------------------------------------
# Host -> MCU
# --------------------------------------------------------------------------

def cmd_home() -> bytes:
    return b"H\n"


def cmd_goto(counts: int) -> bytes:
    return f"G {int(counts)}\n".encode()


def cmd_gains(kp: float, ki: float, kd: float) -> bytes:
    return f"K {kp:.6g} {ki:.6g} {kd:.6g}\n".encode()


def cmd_enable(on: bool) -> bytes:
    return f"E {1 if on else 0}\n".encode()


def cmd_stop() -> bytes:
    return b"S\n"


# --------------------------------------------------------------------------
# MCU -> host
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Telemetry:
    """One telemetry frame from the firmware."""
    stamp_us: int      # MCU microseconds, monotonic since boot
    position: int      # encoder counts
    target: int        # encoder counts
    error: int         # target - position, counts
    pwm: int           # -255..255, signed drive effort
    homed: bool        # True once the homing routine has completed


@dataclass(frozen=True)
class Ack:
    text: str


@dataclass(frozen=True)
class Fault:
    text: str


class ProtocolError(ValueError):
    """Raised when a line cannot be parsed."""


def parse_line(line: str):
    """Parse one line from the MCU.

    Returns Telemetry, Ack or Fault. Returns None for blank lines so the reader
    loop can ignore keepalives and partial flushes without branching.
    Raises ProtocolError on a malformed frame.
    """
    line = line.strip()
    if not line:
        return None

    kind, _, rest = line.partition(" ")

    if kind == "T":
        parts = rest.split()
        if len(parts) != 6:
            raise ProtocolError(f"telemetry needs 6 fields, got {len(parts)}: {line!r}")
        try:
            return Telemetry(
                stamp_us=int(parts[0]),
                position=int(parts[1]),
                target=int(parts[2]),
                error=int(parts[3]),
                pwm=int(parts[4]),
                homed=parts[5] not in ("0", "false", "False"),
            )
        except ValueError as exc:
            raise ProtocolError(f"bad telemetry field in {line!r}") from exc

    if kind == "A":
        return Ack(rest)

    if kind == "X":
        return Fault(rest)

    raise ProtocolError(f"unknown frame type {kind!r} in {line!r}")


# --------------------------------------------------------------------------
# Unit conversion
# --------------------------------------------------------------------------

import math


def counts_to_radians(counts: float, counts_per_rev: int) -> float:
    return counts * 2.0 * math.pi / counts_per_rev


def radians_to_counts(radians: float, counts_per_rev: int) -> int:
    return int(round(radians * counts_per_rev / (2.0 * math.pi)))
