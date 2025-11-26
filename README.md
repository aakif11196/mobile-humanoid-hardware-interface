# Mobile Humanoid ROS 2 Hardware Interface

A custom ROS 2 Hardware Interface that connects a mobile humanoid robot's actuators and sensors with the ROS 2 control framework. This layer enables clean, reliable, real-time communication between the robot's brain (ROS 2) and body (hardware).

## 📌 Features

- Sends control commands to actuators (motors, servos, etc.)
- Reads encoder / IMU / joint / system sensors and publishes data to ROS 2
- Supports UART, I²C, CAN, and UDP communication protocols
- Handles timing, synchronization, and real-time update loops
- Built-in packet ordering, validation & recovery
- Auto-reconnect logic for unstable networks
- Extensible architecture for new sensors, actuators, or communication modes
- Compatible with `ros2_control` and `controller_manager`

## 🧠 Why Build a Custom Interface?

Although ready-made solutions exist, building from scratch provided:

- **Full flexibility** to debug and optimize low-level behavior
- **Control** over message formats and bandwidth usage
- **Ability to fix** hardware-layer issues directly
- **Real hardware testing** revealed issues simulations never show
- **Smooth, reliable motion control** after iterative tuning

This project strengthened understanding of real-time systems, communication stacks, and control loops at the hardware level.

https://github.com/aakif11196/mobile-humanoid-hardware-interface/HI_working (2).mp4
