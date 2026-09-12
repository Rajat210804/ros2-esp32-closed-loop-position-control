"""Unit tests for the serial protocol and the PID/plant simulation.

These run without ROS installed:  python3 -m pytest test/ -q
"""
import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from single_axis import protocol
from single_axis.mock_axis import MockAxis, PID


# ---------------------------------------------------------------- protocol

def test_command_encoding():
    assert protocol.cmd_home() == b"H\n"
    assert protocol.cmd_goto(1234) == b"G 1234\n"
    assert protocol.cmd_goto(-7) == b"G -7\n"
    assert protocol.cmd_enable(True) == b"E 1\n"
    assert protocol.cmd_enable(False) == b"E 0\n"
    assert protocol.cmd_stop() == b"S\n"
    assert protocol.cmd_gains(6.0, 0.8, 0.09) == b"K 6 0.8 0.09\n"


def test_parse_telemetry():
    t = protocol.parse_line("T 105233 4821 5000 179 143 1")
    assert isinstance(t, protocol.Telemetry)
    assert t.stamp_us == 105233
    assert t.position == 4821
    assert t.target == 5000
    assert t.error == 179
    assert t.pwm == 143
    assert t.homed is True


def test_parse_negative_and_unhomed():
    t = protocol.parse_line("T 1 -320 0 320 -88 0")
    assert t.position == -320
    assert t.pwm == -88
    assert t.homed is False


def test_parse_ack_and_fault():
    assert protocol.parse_line("A homing: complete").text == "homing: complete"
    assert isinstance(protocol.parse_line("X limit switch open"), protocol.Fault)


def test_blank_lines_ignored():
    assert protocol.parse_line("") is None
    assert protocol.parse_line("   \r\n") is None


@pytest.mark.parametrize("bad", [
    "T 1 2 3",                  # too few fields
    "T 1 2 3 4 5 6 7",          # too many
    "T a b c d e f",            # non-numeric
    "Q whatever",               # unknown frame type
])
def test_malformed_frames_raise(bad):
    with pytest.raises(protocol.ProtocolError):
        protocol.parse_line(bad)


# -------------------------------------------------------------- unit conv

def test_counts_radians_roundtrip():
    cpr = 1200
    for counts in (0, 1, 300, 1200, -450):
        rad = protocol.counts_to_radians(counts, cpr)
        assert protocol.radians_to_counts(rad, cpr) == counts


def test_one_rev_is_two_pi():
    assert protocol.counts_to_radians(1200, 1200) == pytest.approx(2 * math.pi)


# -------------------------------------------------------------------- PID

def test_derivative_on_measurement_has_no_setpoint_kick():
    """A step in target must not produce a derivative spike.

    Output limits are opened up here so the assertion tests the control law
    itself rather than the saturation clamp.
    """
    pid = PID(kp=1.0, ki=0.0, kd=100.0, out_min=-1e9, out_max=1e9)
    pid.update(0.0, 0.0, 0.01)           # settle
    out = pid.update(1000.0, 0.0, 0.01)  # large step in TARGET, measurement unchanged
    assert out == pytest.approx(1000.0)  # kp term only, no kd contribution


def test_derivative_does_respond_to_measurement_change():
    """The mirror of the test above: motion alone must produce a kd term."""
    pid = PID(kp=0.0, ki=0.0, kd=10.0, out_min=-1e9, out_max=1e9)
    pid.update(0.0, 0.0, 0.01)
    out = pid.update(0.0, 5.0, 0.01)     # measurement moved +5 over 10 ms
    assert out == pytest.approx(-10.0 * 500.0)


def test_integral_is_clamped():
    pid = PID(kp=0.0, ki=1.0, kd=0.0, i_limit=50.0)
    for _ in range(1000):
        pid.update(100.0, 0.0, 0.01)
    assert abs(pid.integral) <= 50.0


# ------------------------------------------------------------------ plant

def _run(axis, seconds, dt=0.002):
    last = None
    for _ in range(int(seconds / dt)):
        last = axis.step(dt)
    return last


def test_axis_converges_to_setpoint():
    axis = MockAxis()
    axis.home()
    axis.goto(5000)
    final = _run(axis, 4.0)
    assert abs(final["error"]) < 25, f"steady-state error too large: {final['error']}"


def test_integral_action_beats_stiction():
    """Integral action must remove the residual error that stiction leaves.

    Deliberately uses a low kp, so that near the target the proportional term
    alone falls below the stiction threshold and the axis stalls short. That is
    the regime where integral action earns its place; at high kp the
    proportional term clears stiction on its own and the test proves nothing.
    """
    no_i = MockAxis(kp=0.5, ki=0.0, kd=0.02)
    no_i.home(); no_i.goto(5000)
    err_without = abs(_run(no_i, 12.0)["error"])

    with_i = MockAxis(kp=0.5, ki=1.5, kd=0.02)
    with_i.home(); with_i.goto(5000)
    err_with = abs(_run(with_i, 12.0)["error"])

    assert err_without > 5, "test invalid: stiction left no error to correct"
    assert err_with < err_without


def test_disabled_axis_does_not_move():
    axis = MockAxis()
    axis.home()
    axis.enable(False)
    axis.goto(5000)
    final = _run(axis, 2.0)
    assert final["position"] == 0
    assert final["pwm"] == 0


def test_travel_limits_respected():
    axis = MockAxis(travel_counts=3000)
    axis.home()
    axis.goto(999999)
    final = _run(axis, 6.0)
    assert final["position"] <= 3000


def test_bidirectional_approach_settles_both_ways():
    axis = MockAxis()
    axis.home()
    axis.goto(6000); _run(axis, 4.0)
    axis.goto(3000); from_above = _run(axis, 4.0)
    axis.goto(1000); _run(axis, 4.0)
    axis.goto(3000); from_below = _run(axis, 4.0)
    assert abs(from_above["error"]) < 25
    assert abs(from_below["error"]) < 25



def test_no_integral_windup_on_long_move():
    """A long saturated move must not produce large overshoot.

    This is the regression test for conditional integration. Without it the
    integral accumulates through the whole saturated travel and the axis
    sails well past the target before recovering.
    """
    axis = MockAxis()
    axis.home()
    axis.goto(9000)

    peak = 0
    for _ in range(int(6.0 / 0.002)):
        sample = axis.step(0.002)
        peak = max(peak, sample["position"])

    overshoot = peak - 9000
    assert overshoot < 60, f"overshoot of {overshoot} counts indicates integral windup"


def test_saturated_output_never_exceeds_pwm_limit():
    pid = PID(kp=1000.0, ki=1000.0, kd=0.0)
    for _ in range(200):
        out = pid.update(50000.0, 0.0, 0.002)
        assert -255.0 <= out <= 255.0
