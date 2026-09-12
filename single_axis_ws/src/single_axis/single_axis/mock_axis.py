"""A simulated DC-motor axis, so the ROS 2 node runs with no hardware attached.

This exists for two reasons. First, you can develop and demo the whole ROS side
before the motor and encoder arrive. Second, when the real axis misbehaves you
can run the identical node against the mock and find out whether the bug is in
your ROS code or in your wiring.

The plant is a first-order velocity model with viscous friction and Coulomb
stiction, integrated to position, driven by the same PID structure that runs on
the MCU. It is not a faithful model of your motor. It is good enough to produce
realistic-looking step responses, overshoot and steady-state error.
"""

from __future__ import annotations


class PID:
    """Position PID with derivative-on-measurement and conditional integration.

    Two details here matter more than the gains, and both are things an
    interviewer will ask about:

    Derivative on measurement. The derivative term acts on the measurement, not
    on the error. A step change in the setpoint therefore produces no
    derivative spike. Take the derivative of the error instead and every new
    command slams the motor, which is the classic "derivative kick".

    Conditional integration. On a long move the output saturates at the PWM
    limit for most of the travel. If the integral keeps accumulating during
    that time it reaches an enormous value, and the axis then sails past the
    target while the integral unwinds. Clamping the integral to a fixed limit
    is not enough -- it just caps how bad the overshoot is. The fix is to stop
    integrating whenever the controller is already saturated and the error
    would push it further in. That is what the check below does, and the
    firmware does the same thing.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        i_limit: float = 4000.0,
        out_min: float = -255.0,
        out_max: float = 255.0,
    ):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self._last_measurement = None

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd

    def reset(self) -> None:
        self.integral = 0.0
        self._last_measurement = None

    def update(self, target: float, measurement: float, dt: float) -> float:
        error = target - measurement

        if self._last_measurement is None:
            derivative = 0.0
        else:
            derivative = (measurement - self._last_measurement) / dt
        self._last_measurement = measurement

        # What the output would be if we accepted this integration step.
        candidate = self.integral + error * dt
        provisional = self.kp * error + self.ki * candidate - self.kd * derivative

        # Integrate only when doing so does not drive us deeper into saturation.
        if self.out_min < provisional < self.out_max:
            self.integral = candidate
        elif (provisional >= self.out_max and error < 0) or (
            provisional <= self.out_min and error > 0
        ):
            # Error has reversed sign; integrating now pulls us back into range.
            self.integral = candidate

        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))

        output = self.kp * error + self.ki * self.integral - self.kd * derivative
        return max(self.out_min, min(self.out_max, output))


class MockAxis:
    """Simulated axis with the same interface the serial backend exposes."""

    def __init__(
        self,
        counts_per_rev: int = 1200,
        kp: float = 4.0,
        ki: float = 0.5,
        kd: float = 0.12,
        travel_counts: int = 12000,
    ):
        self.counts_per_rev = counts_per_rev
        self.travel_counts = travel_counts

        self.position = 0.0        # counts
        self.velocity = 0.0        # counts/s
        self.target = 0            # counts
        self.pwm = 0               # -255..255
        self.homed = False
        self.enabled = True

        self.pid = PID(kp, ki, kd)

        # Plant constants, hand-tuned to look like a small geared DC motor.
        self._gain = 5.2           # counts/s per unit pwm, steady state
        self._tau = 0.045          # s, velocity time constant
        self._stiction = 11.0      # pwm units that produce no motion at all
        self._stamp_us = 0

    # -- commands ---------------------------------------------------------

    def home(self) -> None:
        self.position = 0.0
        self.velocity = 0.0
        self.target = 0
        self.homed = True
        self.pid.reset()

    def goto(self, counts: int) -> None:
        self.target = int(max(0, min(self.travel_counts, counts)))

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self.pid.set_gains(kp, ki, kd)

    def enable(self, on: bool) -> None:
        self.enabled = on
        if not on:
            self.pwm = 0
            self.pid.reset()

    def stop(self) -> None:
        self.target = int(round(self.position))

    # -- integration ------------------------------------------------------

    def step(self, dt: float):
        """Advance the simulation by dt seconds and return a telemetry tuple."""
        if self.enabled:
            effort = self.pid.update(self.target, self.position, dt)
            self.pwm = int(max(-255, min(255, round(effort))))
        else:
            self.pwm = 0

        # Stiction: small efforts move nothing. This is why a real axis has
        # steady-state error that only integral action removes.
        effective = 0.0
        if abs(self.pwm) > self._stiction:
            effective = self.pwm - (self._stiction if self.pwm > 0 else -self._stiction)

        target_velocity = effective * self._gain
        alpha = dt / (self._tau + dt)
        self.velocity += alpha * (target_velocity - self.velocity)
        self.position += self.velocity * dt

        # Hard stops at both ends of travel.
        if self.position < 0.0:
            self.position, self.velocity = 0.0, 0.0
        elif self.position > self.travel_counts:
            self.position, self.velocity = float(self.travel_counts), 0.0

        self._stamp_us += int(dt * 1e6)

        pos = int(round(self.position))
        return {
            "stamp_us": self._stamp_us,
            "position": pos,
            "target": self.target,
            "error": self.target - pos,
            "pwm": self.pwm,
            "homed": self.homed,
        }
