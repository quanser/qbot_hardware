# qbot_hardware

Hardware driver nodes for the QBot Platform: motor control, sensors, and manual control input.

## Overview

`qbot_hardware` provides the ROS 2 interface to the QBot Platform's onboard hardware. It publishes sensor data (IMU, battery, joint states, lidar, camera) and accepts motor commands, forming the driver layer that other packages build on for localization, navigation, and policy deployment.

`qbot_localization`'s `qbot_localization_launch.py` directly launches `qbot_platform_driver_interface`, `lidar`, and `fixed_lidar_frame` from this package by node name — this package must be built and available in the same workspace to run localization.

## Nodes

### Sensor-centric

**`qbot_platform_driver_interface`**
Interfaces with the QUARC executable `qbot_platform_driver_physical.rt-linux_qbot_platform`, which sends commands directly to the QBot Platform. Publishes IMU data, battery level, and joint states, and handles motor commands and LED values.

> It's recommended to run this node via `qbot_platform_launch.py` rather than
> standalone, since the launch file starts both the QUARC executable and this
> node together.

**`lidar`**
Publishes LiDAR scan data from the Leishen M10p.

**`csi`**
Publishes image data from the QBot Platform's downward-facing camera.

**`rgbd`**
Publishes RGB and depth data from the Intel RealSense D435 camera.

### User-centric

**`command`**
Allows manual control of the QBot via a Logitech F710 joystick.

### Auxiliary

**`fixed_lidar_frame`**
Static TF publisher that transforms and rotates the LiDAR frame to align with
the center of the QBot Platform.

## Launch Files

| Launch file | Brings up |
|---|---|
| `qbot_platform_launch.py` | All sensor-centric nodes |
| `qbot_platform_manual_drive_launch.py` | Sensor-centric nodes + `command` node, for manual joystick driving |
| `qbot_platform_cartographer_launch.py` | Mapping (no `command` node) |
| `qbot_platform_manual_map_launch.py` | Manual driving + map generation |
| `qbot_platform_slam_and_nav_bringup_launch.py` | Nav2-based navigation to a goal pose set via RViz2 |

## Getting Started

Source ROS 2 Humble before running any node or launch file:

```bash
source /opt/ros/humble/setup.bash
```

### Build

From the workspace root:

```bash
colcon build --packages-select qbot_hardware
source install/setup.bash
```

### Running a node

```bash
ros2 run qbot_hardware <node_name>
```

Example — run the lidar node:
```bash
ros2 run qbot_hardware lidar
```

Nodes should generally run one per terminal session for development. For normal operation, use a launch file instead.

### Running a launch file

```bash
ros2 launch qbot_hardware <launch_file_name>.launch.py
```

Example — manual drive:
```bash
ros2 launch qbot_hardware qbot_platform_manual_drive_launch.py
```

## License

Apache-2.0