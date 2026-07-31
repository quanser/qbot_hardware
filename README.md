# qbot_hardware

ROS 2 packages for hardware drivers and localization on the QBot Platform.

## Overview

This repo provides the low-level ROS 2 interface to the QBot Platform: motor control, sensor drivers, and localization (scan-match and EKF sensor fusion). It runs on the QBot's onboard Jetson and publishes the sensor and pose topics that downstream packages, such as [`qbot_rl_deploy`](<link-to-qbot_rl_deploy-repo>), depend on it for navigation and control.


## Packages in this repo

| Package | Description |
|---|---|
| [`qbot_hardware`](./qbot_hardware/README.md) | Motor control and sensor drivers (wheel encoders, gyro, lidar, etc.) |
| [`qbot_localization`](./qbot_localization/README.md) | Scan-match localization and EKF sensor fusion |

## Prerequisites

- QUARC runtime (required for the QBot driver and `ekf_localization_node`'s RT model)
- ROS2 Humble

## Getting Started

Prior to running any ROS 2 example, source Humble in your terminal session:

```bash
source /opt/ros/humble/setup.bash
```

### Build

From your ROS 2 workspace root:

```bash
colcon build --packages-select qbot_hardware qbot_localization
source install/setup.bash
```

### Running nodes

```bash
ros2 run <package_name> <node_name>
```

Example:
```bash
ros2 run qbot_localization ekf_localization_node
```

Nodes should generally run one per terminal session for development/debugging. For normal operation, use a launch file.

### Running launch files

```bash
ros2 launch <package_name> <launch_file_name>.launch.py
```

Example:
```bash
ros2 launch qbot_localization qbot_localization_launch.py
```

See each package's README for the full list of nodes, topics, parameters, and launch files.

## Known Limitations / Operational Notes

- Running RViz locally on the QBot's onboard Jetson via remote desktop causes severe CPU contention and should not be used during real evaluation runs. Visualize from a separate machine on the same ROS 2 domain instead.

## License

Apache-2.0