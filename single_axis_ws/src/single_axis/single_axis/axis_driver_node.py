#!/usr/bin/env python3
"""ROS 2 driver for a single closed-loop position axis.

Exposes an ESP32-controlled DC motor axis through a standard ROS interface:

    publishes   /joint_states                sensor_msgs/JointState
                ~/tracking_error             std_msgs/Float64   (radians)
                ~/homed                      std_msgs/Bool
    subscribes  ~/position_command           std_msgs/Float64   (radians)
    services    ~/home                       std_srvs/Trigger
                ~/enable                     std_srvs/SetBool

The PID loop itself runs on the MCU at a fixed rate, not here. That is the
right split: ROS is not real-time, and a control loop that depends on Linux
scheduling and USB latency will not hold tolerance. ROS owns setpoints,
telemetry and coordination; the MCU owns the loop.

Run with mock:=true to exercise the whole interface with no hardware attached.
"""

from __future__ import annotations

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool, Trigger

from single_axis import protocol
from single_axis.mock_axis import MockAxis


class AxisDriverNode(Node):

    def __init__(self):
        super().__init__("axis_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("joint_name", "axis_1")
        self.declare_parameter("counts_per_rev", 1200)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("mock", False)
        self.declare_parameter("kp", 4.0)
        self.declare_parameter("ki", 0.5)
        self.declare_parameter("kd", 0.12)

        self.joint_name = self.get_parameter("joint_name").value
        self.counts_per_rev = int(self.get_parameter("counts_per_rev").value)
        self.mock = bool(self.get_parameter("mock").value)
        rate = float(self.get_parameter("publish_rate_hz").value)

        # Latest telemetry, written by the reader thread, read by the timer.
        self._lock = threading.Lock()
        self._telemetry = None
        self._prev_position = None
        self._prev_stamp_us = None
        self._velocity = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_joint = self.create_publisher(JointState, "/joint_states", sensor_qos)
        self.pub_error = self.create_publisher(Float64, "~/tracking_error", sensor_qos)
        self.pub_homed = self.create_publisher(Bool, "~/homed", 10)

        self.create_subscription(Float64, "~/position_command", self._on_command, 10)
        self.create_service(Trigger, "~/home", self._on_home)
        self.create_service(SetBool, "~/enable", self._on_enable)

        if self.mock:
            self._axis = MockAxis(
                counts_per_rev=self.counts_per_rev,
                kp=float(self.get_parameter("kp").value),
                ki=float(self.get_parameter("ki").value),
                kd=float(self.get_parameter("kd").value),
            )
            self._serial = None
            self._sim_dt = 0.002  # 500 Hz, matching the firmware loop rate
            self.create_timer(self._sim_dt, self._step_mock)
            self.get_logger().info("running against the mock axis, no hardware needed")
        else:
            self._axis = None
            self._open_serial()
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()

        self.create_timer(1.0 / rate, self._publish)
        self._push_gains()

    # -- transport --------------------------------------------------------

    def _open_serial(self):
        import serial  # imported lazily so mock mode needs no pyserial

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self._serial = serial.Serial(port, baud, timeout=0.2)
        self.get_logger().info(f"opened {port} at {baud} baud")

    def _write(self, payload: bytes) -> None:
        if self._serial is not None:
            self._serial.write(payload)

    def _read_loop(self) -> None:
        """Background thread: drain the serial port, keep the newest frame."""
        while rclpy.ok():
            try:
                raw = self._serial.readline().decode("ascii", errors="replace")
            except Exception as exc:  # noqa: BLE001 - a dropped USB link must not kill the node
                self.get_logger().warn(f"serial read failed: {exc}")
                continue

            try:
                frame = protocol.parse_line(raw)
            except protocol.ProtocolError as exc:
                self.get_logger().debug(f"discarding frame: {exc}")
                continue

            if isinstance(frame, protocol.Telemetry):
                with self._lock:
                    self._telemetry = frame
            elif isinstance(frame, protocol.Ack):
                self.get_logger().info(f"mcu: {frame.text}")
            elif isinstance(frame, protocol.Fault):
                self.get_logger().error(f"mcu fault: {frame.text}")

    def _step_mock(self) -> None:
        sample = self._axis.step(self._sim_dt)
        with self._lock:
            self._telemetry = protocol.Telemetry(**sample)

    # -- callbacks --------------------------------------------------------

    def _on_command(self, msg: Float64) -> None:
        counts = protocol.radians_to_counts(msg.data, self.counts_per_rev)
        if self.mock:
            self._axis.goto(counts)
        else:
            self._write(protocol.cmd_goto(counts))

    def _on_home(self, request, response):
        del request
        if self.mock:
            self._axis.home()
        else:
            self._write(protocol.cmd_home())
        response.success = True
        response.message = "homing started"
        return response

    def _on_enable(self, request, response):
        if self.mock:
            self._axis.enable(request.data)
        else:
            self._write(protocol.cmd_enable(request.data))
        response.success = True
        response.message = "enabled" if request.data else "disabled"
        return response

    def _push_gains(self) -> None:
        kp = float(self.get_parameter("kp").value)
        ki = float(self.get_parameter("ki").value)
        kd = float(self.get_parameter("kd").value)
        if self.mock:
            self._axis.set_gains(kp, ki, kd)
        else:
            self._write(protocol.cmd_gains(kp, ki, kd))

    # -- publishing -------------------------------------------------------

    def _publish(self) -> None:
        with self._lock:
            telemetry = self._telemetry
        if telemetry is None:
            return

        # Velocity by finite difference on the MCU clock, lightly filtered.
        # Using the MCU timestamp rather than ROS time avoids the USB jitter
        # that would otherwise show up as velocity noise.
        if self._prev_stamp_us is not None:
            dt = (telemetry.stamp_us - self._prev_stamp_us) * 1e-6
            if dt > 1e-6:
                raw = (telemetry.position - self._prev_position) / dt
                self._velocity += 0.3 * (raw - self._velocity)
        self._prev_stamp_us = telemetry.stamp_us
        self._prev_position = telemetry.position

        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = [self.joint_name]
        js.position = [protocol.counts_to_radians(telemetry.position, self.counts_per_rev)]
        js.velocity = [protocol.counts_to_radians(self._velocity, self.counts_per_rev)]
        js.effort = [float(telemetry.pwm)]
        self.pub_joint.publish(js)

        err = Float64()
        err.data = protocol.counts_to_radians(telemetry.error, self.counts_per_rev)
        self.pub_error.publish(err)

        self.pub_homed.publish(Bool(data=telemetry.homed))


def main(args=None):
    rclpy.init(args=args)
    node = AxisDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
