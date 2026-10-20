# Final-Year Robot — Differential-Drive ROS 2 Robot with Sensor Fusion and Vision

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![ros2_control](https://img.shields.io/badge/ros2__control-custom%20HW%20interface-555)](https://control.ros.org/humble/)
[![Arduino](https://img.shields.io/badge/MCU-Arduino-00979D?logo=arduino)](https://www.arduino.cc/)
[![Real hardware](https://img.shields.io/badge/runs%20on-real%20hardware-2EA44F)](#demo)

My B.Tech final-year robot (late 2024) and my first robot running ROS 2 on real hardware. It is a two-wheel differential-drive robot with its own motor firmware, a `ros2_control` hardware interface, odometry corrected by a Kalman filter, joystick teleoperation with an RViz twin, and camera-based line following that runs both on the real robot and in Gazebo. I extended it with a QR-code maze solver in simulation.

## Demo

<table>
<tr>
<td align="center"><a href="https://youtu.be/KG07amkaR44"><img src="https://img.youtube.com/vi/KG07amkaR44/hqdefault.jpg" width="250" alt="Vision line follower, real and Gazebo"></a><br><sub>Vision line follower — real + Gazebo</sub></td>
<td align="center"><a href="https://youtu.be/RFRIoyXHQq8"><img src="https://img.youtube.com/vi/RFRIoyXHQq8/hqdefault.jpg" width="250" alt="Teleoperation with RViz twin"></a><br><sub>Teleoperation with RViz twin</sub></td>
<td align="center"><a href="https://youtu.be/XQghr4gMNv4"><img src="https://img.youtube.com/vi/XQghr4gMNv4/hqdefault.jpg" width="250" alt="Marker-based maze solver"></a><br><sub>Marker-based maze solver (Gazebo)</sub></td>
</tr>
</table>

On LinkedIn (Oct–Dec 2024): [robot complete](https://www.linkedin.com/feed/update/urn:li:activity:7254822495826923520/) · [teleoperation with RViz twin](https://www.linkedin.com/feed/update/urn:li:activity:7256238442001412096/) · [QR maze solver](https://www.linkedin.com/feed/update/urn:li:activity:7256869923757449216/) · [line follower, real and Gazebo](https://www.linkedin.com/feed/update/urn:li:activity:7259421708418232320/) · [RPLidar A1 bring-up](https://www.linkedin.com/feed/update/urn:li:activity:7267394254312751104/) · [twist_mux in Gazebo](https://www.linkedin.com/feed/update/urn:li:activity:7270648174011133952/)

## Key features

- **Firmware** — Arduino PID speed control per wheel with interrupt-driven encoders and an L298N driver.
- **Hardware interface** — C++ `ros2_control` system plugin over serial; DiffDriveController at 100 Hz.
- **Sensor fusion** — a 1-D Kalman filter (Python and C++) fuses wheel-odometry yaw rate with the MPU6050's gyro; a `robot_localization` EKF config is included.
- **Kinematics from scratch** — differential-drive forward/inverse kinematics implemented directly (plus a noisy-odometry variant) alongside the stock controller.
- **Vision** — OpenCV line follower on the robot's Raspberry Pi camera feed and in Gazebo; QR-code maze solver using camera + LiDAR.
- **Teleoperation** — joystick with a live RViz digital twin.

## Hardware

Arduino (motor PID) · L298N · 2 × DC gear motors with encoders · MPU6050 IMU · Raspberry Pi with camera · wheel radius 0.033 m, separation 0.18 m

## Software stack

ROS 2 Humble · ros2_control · robot_localization · Gazebo · RViz · OpenCV · C++ · Python

## Run

```bash
mkdir -p ~/bumperbot_ws/src && cd ~/bumperbot_ws/src
git clone https://github.com/Curious-Utkarsh/bumperbot_ws.git .
cd ~/bumperbot_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash

ros2 launch bumperbot_bringup simulated_robot.launch.py     # simulation
ros2 launch bumperbot_bringup real_robot.launch.py          # real robot
```

## Related

- [Mobi](https://github.com/Curious-Utkarsh/Indoor_Service_AMR) and [MechaBot](https://github.com/MechaMind-Labs/AMR-Mastery) — the larger robots built on this hardware-interface pattern
- [Arduino joystick → ROS 2](https://github.com/MechaMind-Labs/Joystick_Module_Integration_with_ROS2) — serial hardware integration, taught as a workshop

## Credits

Package structure and the base robot follow Antonio Brandi's *Self-Driving and ROS 2 — Learn by Doing* course (the "bumperbot"); the real-robot build, vision line follower and maze solver are my extensions.

---
[Kumar Utkarsh](https://github.com/Curious-Utkarsh) · [LinkedIn](https://www.linkedin.com/in/kumar-utkarsh-124aab227/) · [MechaMind Labs](https://www.mechamindlabs.com/)
