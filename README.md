# single_axis

A small single-axis closed-loop position control project using an ESP32,
quadrature encoder, geared DC motor, H-bridge and a ROS 2 interface.

I built this mainly to understand the full path from a position command
in ROS 2 to an actual motor movement, including encoder feedback, PID
control, homing and testing.

The main idea is simple:

ROS 2 sends the target position -> ESP32 runs the control loop -> encoder
measures the actual position -> PID corrects the motor position.

---

## Why I made it this way

One thing I wanted to avoid was putting the PID loop inside the ROS 2 node.

ROS 2 is running on a normal Linux computer and communicates with the
ESP32 over serial. The timing of that communication is not deterministic,
so using it as the actual 500 Hz control loop would make the controller
dependent on USB/serial communication and Linux scheduling.

So I split the project into two parts:

```text
                ROS 2
                  |
          position command
                  |
                Serial
                  |
                  v
                ESP32
           +--------------+
           | PID @ 500 Hz |
           +--------------+
                  |
                 PWM
                  |
                  v
              H-Bridge
                  |
                  v
              DC Motor
                  |
                  v
            Encoder feedback
                  |
                  +------> ESP32
