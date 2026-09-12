# single_axis — closed-loop position axis with a ROS 2 interface

A single precision positioning axis: geared DC motor, quadrature encoder,
H-bridge and limit switch, with the PID loop on an ESP32 and a ROS 2 driver on
top. Includes a bidirectional repeatability test that produces a measured
number, not an estimate.

```
  ROS 2 (rclpy)                      ESP32 (500 Hz)              hardware
  ─────────────                      ─────────────               ────────
  /joint_states        ◄── serial ── telemetry  ◄── encoder ◄──  motor + encoder
  ~/position_command   ──► serial ─► setpoint   ──► PWM     ──►  H-bridge
  ~/home (Trigger)                   homing routine ◄────────    limit switch
```

**The control loop runs on the MCU, not in ROS.** This is the important
architectural decision in the project and the thing to say first if anyone asks
about it. ROS 2 is not a real-time system: your node is scheduled by Linux and
your data crosses USB, so loop jitter is tens of milliseconds and not bounded.
A position loop built on that cannot hold tolerance. ROS owns setpoints,
telemetry, coordination and logging. The MCU owns the loop. Every serious
motion system is split along this line.

---

## Build

```bash
mkdir -p ~/single_axis_ws/src
# copy src/single_axis into ~/single_axis_ws/src/
cd ~/single_axis_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select single_axis
source install/setup.bash
```

Tested against ROS 2 Humble and Jazzy. Needs `pyserial` for real hardware
(`sudo apt install python3-serial`); mock mode does not.

## Run

No hardware yet:

```bash
ros2 launch single_axis axis.launch.py mock:=true
```

With hardware:

```bash
ros2 launch single_axis axis.launch.py port:=/dev/ttyUSB0
ros2 service call /axis_driver/home std_srvs/srv/Trigger
ros2 topic pub --once /axis_driver/position_command std_msgs/msg/Float64 "{data: 3.14}"
```

Watch the loop close, which is the plot worth putting in the README:

```bash
ros2 run rqt_plot rqt_plot /joint_states/position[0] /axis_driver/tracking_error/data
```

## Measure repeatability

```bash
ros2 run single_axis repeatability_test --ros-args \
  -p target_rad:=3.14 -p approach_rad:=1.0 -p cycles:=50
```

It approaches the same target 50 times, alternating direction, waits for
settle, and reports unidirectional and bidirectional repeatability, reversal
error and total spread, plus a CSV of raw samples.

**Alternating direction is the whole point.** A unidirectional test hides
gearbox and coupling backlash and will flatter your number several times over.
Separating the two also lets you say *which* part of your error is backlash,
which is a much stronger claim than a single number.

---

## Interface

| | type | notes |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | position rad, velocity rad/s, effort = PWM |
| `~/tracking_error` | `std_msgs/Float64` | radians |
| `~/homed` | `std_msgs/Bool` | |
| `~/position_command` | `std_msgs/Float64` | radians, absolute |
| `~/home` | `std_srvs/Trigger` | blocking on the MCU |
| `~/enable` | `std_srvs/SetBool` | |

Publishing `JointState` on `/joint_states` rather than a custom message is
deliberate: `robot_state_publisher`, RViz and `ros2_control` all consume it
without adaptation, so this axis drops into a larger robot later.

## Wiring

| signal | GPIO | note |
|---|---|---|
| encoder A | 34 | input-only pin, needs an external pull-up |
| encoder B | 35 | input-only pin, needs an external pull-up |
| motor IN1 | 25 | |
| motor IN2 | 26 | |
| motor EN (PWM) | 27 | 20 kHz, above audible |
| limit switch | 32 | **wire normally closed to GND** |

Wire the limit switch normally closed. A broken wire then reads identically to
a triggered switch and the axis refuses to move, rather than driving
cheerfully into its own end stop. This is a standard machine-safety practice
and it is worth being able to explain why.

`counts_per_rev = encoder_PPR × 4 × gear_ratio`. The ×4 is x4 quadrature
decoding — both edges of both channels. Verify it empirically before trusting
any number the test reports: home, command exactly one revolution, and check
the output shaft actually turned once.

## Tuning

1. `ki = 0`, `kd = 0`. Raise `kp` until the axis oscillates around the target.
2. Halve `kp`.
3. Raise `kd` until the overshoot is damped. Too much and it goes rough and noisy — the derivative is amplifying encoder quantisation.
4. Add the smallest `ki` that removes the steady-state error stiction leaves behind. Too much and the axis hunts.

Defaults in `config/axis_params.yaml` (`kp=4.0, ki=0.5, kd=0.12`) were found by
sweeping gains against the simulated plant in `mock_axis.py`. **They are a
starting point for your motor, not an answer.** Retune on hardware and keep the
before/after plot.

## Tests

```bash
cd src/single_axis && python3 -m pytest test/ -q     # 21 tests, no ROS needed
```

Covers protocol encode/parse, malformed-frame rejection, unit conversion
round-trips, derivative-on-measurement, integral clamping, anti-windup
regression, output saturation, and plant convergence from both directions.

---

## Two details worth understanding before you discuss this

Both were real bugs caught by the test suite, and both are standard interview
questions.

**Derivative on measurement.** The derivative term acts on the measurement, not
the error. A step change in setpoint therefore produces no derivative spike.
Differentiate the error instead and every new command slams the motor —
"derivative kick".

**Conditional integration.** On a long move the output saturates at the PWM
limit for most of the travel. If the integral keeps accumulating through that
stretch it reaches a huge value and the axis sails past the target while it
unwinds. Clamping the integral only caps how bad this gets; the fix is to
freeze integration whenever the controller is already saturated and the error
would push it further in. `test_no_integral_windup_on_long_move` is the
regression test, and it failed on the first version of this code.

## What is yours to do

The software is only half of it, and the half that does not by itself prove
anything. What makes this project worth putting on a CV:

- build it, wire it, and find out what your motor actually does
- tune the gains on hardware and keep the before/after step-response plot
- run the repeatability test and report the real number, whatever it is
- photograph the rig and record ten seconds of the axis moving

A modest, honestly measured number with a plot and a video is worth far more
than an impressive one you cannot account for. If the repeatability comes out
poor, that is a finding, not a failure — knowing your backlash is 40 counts and
being able to say why is the engineering.
