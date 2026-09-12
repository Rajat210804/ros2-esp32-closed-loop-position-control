#!/usr/bin/env python3
"""Measure bidirectional positioning repeatability of the axis.

This is the script that produces the number you put on your CV, so it is worth
understanding exactly what it measures and what it does not.

Method. The axis approaches one target position N times, alternating the
approach direction: from below, then from above, then from below again. After
each approach it waits for the axis to settle (velocity under a threshold for a
sustained window) and records where it actually stopped. Alternating direction
is the whole point -- a unidirectional test hides backlash in the gearbox and
coupling, and will flatter your number by a factor of several.

Reported statistics, all in encoder counts and in the axis's own units:

    unidirectional repeatability   std dev within each approach direction
    bidirectional repeatability    std dev across all approaches
    reversal error (backlash)      mean(from above) - mean(from below)
    total spread                   max - min across all approaches

The honest CV bullet comes from the bidirectional number and the sample count,
for example: "measured bidirectional positioning repeatability of +/-14 counts
(0.07 deg) over 50 approaches, with 31 counts of reversal error attributable to
gearbox backlash".

Run:
    ros2 run single_axis repeatability_test --ros-args \
        -p target_rad:=3.14 -p approach_rad:=1.0 -p cycles:=50
"""

from __future__ import annotations

import math
import statistics
import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


class RepeatabilityTest(Node):

    def __init__(self):
        super().__init__("repeatability_test")

        self.declare_parameter("joint_name", "axis_1")
        self.declare_parameter("counts_per_rev", 1200)
        self.declare_parameter("target_rad", 3.0)
        self.declare_parameter("approach_rad", 1.0)   # how far to back off between approaches
        self.declare_parameter("cycles", 50)
        self.declare_parameter("settle_velocity_rad_s", 0.02)
        self.declare_parameter("settle_time_s", 0.5)
        self.declare_parameter("timeout_s", 10.0)
        self.declare_parameter("csv_path", "repeatability.csv")

        self.joint_name = self.get_parameter("joint_name").value
        self.counts_per_rev = int(self.get_parameter("counts_per_rev").value)
        self.target = float(self.get_parameter("target_rad").value)
        self.approach = float(self.get_parameter("approach_rad").value)
        self.cycles = int(self.get_parameter("cycles").value)
        self.settle_v = float(self.get_parameter("settle_velocity_rad_s").value)
        self.settle_t = float(self.get_parameter("settle_time_s").value)
        self.timeout = float(self.get_parameter("timeout_s").value)

        self.pub_cmd = self.create_publisher(Float64, "/axis_driver/position_command", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._home_client = self.create_client(Trigger, "/axis_driver/home")

        self.position = None
        self.velocity = 0.0
        self._settled_for = 0.0
        self._last_time = None

        # results[0] = approached from below, results[1] = approached from above
        self.results: list[list[float]] = [[], []]

        self._state = "wait_for_data"
        self._cycle = 0
        self._elapsed = 0.0
        self.create_timer(0.02, self._tick)

    # -- feedback ---------------------------------------------------------

    def _on_joint_state(self, msg: JointState) -> None:
        if self.joint_name not in msg.name:
            return
        i = msg.name.index(self.joint_name)
        self.position = msg.position[i]
        self.velocity = msg.velocity[i] if msg.velocity else 0.0

    def _command(self, radians: float) -> None:
        self.pub_cmd.publish(Float64(data=radians))

    def _is_settled(self, dt: float) -> bool:
        if abs(self.velocity) < self.settle_v:
            self._settled_for += dt
        else:
            self._settled_for = 0.0
        return self._settled_for >= self.settle_t

    # -- state machine ----------------------------------------------------

    def _tick(self) -> None:
        dt = 0.02
        self._elapsed += dt

        if self.position is None:
            return

        if self._state == "wait_for_data":
            self.get_logger().info("homing before measurement")
            if self._home_client.wait_for_service(timeout_sec=2.0):
                self._home_client.call_async(Trigger.Request())
            self._state, self._elapsed, self._settled_for = "back_off", 0.0, 0.0
            return

        if self._state == "back_off":
            # Alternate the approach direction each cycle.
            from_below = (self._cycle % 2 == 0)
            standoff = self.target - self.approach if from_below else self.target + self.approach
            self._command(standoff)
            if self._is_settled(dt) or self._elapsed > self.timeout:
                self._state, self._elapsed, self._settled_for = "approach", 0.0, 0.0
            return

        if self._state == "approach":
            self._command(self.target)
            if self._is_settled(dt):
                direction = self._cycle % 2
                self.results[direction].append(self.position)
                self._cycle += 1
                self.get_logger().info(
                    f"cycle {self._cycle}/{self.cycles}  "
                    f"from {'below' if direction == 0 else 'above'}  "
                    f"stopped at {self._counts(self.position):+d} counts  "
                    f"error {self._counts(self.position - self.target):+d}"
                )
                if self._cycle >= self.cycles:
                    self._report()
                    rclpy.shutdown()
                    return
                self._state, self._elapsed, self._settled_for = "back_off", 0.0, 0.0
            elif self._elapsed > self.timeout:
                self.get_logger().error("axis failed to settle, aborting")
                rclpy.shutdown()
            return

    def _counts(self, radians: float) -> int:
        return int(round(radians * self.counts_per_rev / (2.0 * math.pi)))

    # -- reporting --------------------------------------------------------

    def _report(self) -> None:
        below, above = self.results
        every = below + above

        def stdev_counts(samples):
            if len(samples) < 2:
                return float("nan")
            return abs(self._counts(statistics.stdev(samples)))

        uni_below = stdev_counts(below)
        uni_above = stdev_counts(above)
        bidirectional = stdev_counts(every)
        reversal = 0.0
        if below and above:
            reversal = self._counts(statistics.fmean(above) - statistics.fmean(below))
        spread = self._counts(max(every) - min(every)) if every else 0

        deg_per_count = 360.0 / self.counts_per_rev

        lines = [
            "",
            "=" * 62,
            f"  repeatability over {len(every)} approaches to {self._counts(self.target)} counts",
            "=" * 62,
            f"  unidirectional (from below)   1 sigma = {uni_below:.1f} counts",
            f"  unidirectional (from above)   1 sigma = {uni_above:.1f} counts",
            f"  bidirectional                 1 sigma = {bidirectional:.1f} counts"
            f"  ({bidirectional * deg_per_count:.3f} deg)",
            f"  reversal error (backlash)             = {reversal:+d} counts"
            f"  ({abs(reversal) * deg_per_count:.3f} deg)",
            f"  total spread (max - min)              = {spread} counts",
            "=" * 62,
            "",
            "  CV bullet, filled in from the numbers above:",
            f"  \"measured bidirectional positioning repeatability of "
            f"+/-{bidirectional:.0f} counts ({bidirectional * deg_per_count:.2f} deg)",
            f"   over {len(every)} approaches, isolating {abs(reversal)} counts of reversal error"
            f" as gearbox backlash\"",
            "",
        ]
        report = "\n".join(lines)
        print(report)

        path = self.get_parameter("csv_path").value
        try:
            with open(path, "w") as fh:
                fh.write("cycle,direction,position_rad,position_counts\n")
                for i, value in enumerate(below):
                    fh.write(f"{i * 2},below,{value:.6f},{self._counts(value)}\n")
                for i, value in enumerate(above):
                    fh.write(f"{i * 2 + 1},above,{value:.6f},{self._counts(value)}\n")
            print(f"  raw samples written to {path}\n")
        except OSError as exc:
            print(f"  could not write {path}: {exc}\n", file=sys.stderr)


def main(args=None):
    rclpy.init(args=args)
    node = RepeatabilityTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
