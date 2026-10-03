# Autonomous Warehouse Robot (GPS-Denied Indoor Navigation)

An autonomous mobile robot designed for GPS-denied indoor warehouse environments. The system enables manual mapping, custom map saving, waypoint navigation, safety monitoring, and complete web-GUI interface control.

## System Features

* **Indoor Mapping (SLAM):** Supports manual teleoperation control to build occupancy grid maps without relying on GPS signals.
* **Map Management:** Save generated maps with custom names via the Web GUI and dynamically select existing maps for navigation tasks.
* **Waypoint Navigation:** Autonomously navigate between user-designated target points using Nav2.
* **Human Detection & Safety Alert:** Processes depth camera feeds using YOLO to detect humans. Triggers a physical hardware buzzer alert if a person is closer than 2 meters.
* **Web Control Dashboard:** Simplified GUI displaying real-time depth camera feeds, surrounding map outputs, and navigation controls.

## Repository Structure

* `articubot_one/`: ROS 2 package containing URDF models, EKF/Nav2 configs, launch files, and sensor processing nodes.
* `web_gui/`: Flask backend and HTML frontend interface for web control.
* `firmware/`: Arduino code for wheel encoders and low-level motor controllers.