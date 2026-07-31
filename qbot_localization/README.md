# qbot_localization

Localization nodes for the QBot Platform: lidar scan-match pose estimation, with an optional QUARC-based EKF sensor fusion alternative.

## Overview

This package provides two **interchangeable** localization nodes. Only one should be run at a time, they share the same default topic and service names so either can be swapped in without changing downstream configuration.

| Node | Approach |
|---|---|
| `scan_match_node` | Lidar scan-matching against a reference pose, published directly as the pose estimate |
| `ekf_localization_node` | Performs scan-matching internally via a QUARC RT model, then fuses it with odometry/IMU in an EKF. |


## Nodes

### `scan_match_node`

Subscribes:
| Topic (param) | Default | Type |
|---|---|---|
| `scan_topic` | `scan` | `sensor_msgs/LaserScan` |

Publishes:
| Topic (param) | Default | Type | Gated by |
|---|---|---|---|
| `pose_topic` | `pose_estimate` | `geometry_msgs/PoseStamped` | always |
| `odom_topic` | `odom` | `nav_msgs/Odometry` | `odom_pub` param |
| tf (`odom` → `base_link`) | — | — | `tf_pub` param |

Services:
| Service (param) | Default | Type |
|---|---|---|
| `reset_service_name` | `/reset_scan_match` | `std_srvs/Trigger` |

Parameters:
| Parameter | Default | Description |
|---|---|---|
| `init_pose` | `[0.0, 0.0, 0.0]` | Initial pose estimate (x, y, theta) |
| `scan_topic` | `scan` | Input lidar scan topic |
| `pose_topic` | `pose_estimate` | Output pose topic |
| `odom_topic` | `odom` | Output odometry topic |
| `calibrate` | `True` | Toggle creating an initial calibration scan |
| `odom_pub` | `True` | Toggle odometry publishing |
| `tf_pub` | `True` | Toggle tf broadcast |
| `reset_service_name` | `/reset_scan_match` | Name of the reset trigger service |

### `ekf_localization_node`

Runtime node name: `ekf_localization_node`

Subscribes:
| Topic (param) | Default | Type |
|---|---|---|
| `scan_topic` | `scan` | `sensor_msgs/LaserScan` |
| `speed_topic` | `/qbot_speed_feedback` | `geometry_msgs/TwistStamped` |
| `imu_topic` | `imu` | `sensor_msgs/Imu` |

Publishes:
| Topic (param) | Default | Type | Gated by |
|---|---|---|---|
| `pose_topic` | `pose_estimate` | `geometry_msgs/PoseStamped` | always |
| `odom_topic` | `odom` | `nav_msgs/Odometry` | `odom_pub` param |
| tf (`odom` → `base_link`)| — | — | `tf_pub` param |

Services:
| Service (param) | Default | Type |
|---|---|---|
| `reset_service_name` | `/reset_scan_match` | `std_srvs/Trigger` |

Parameters:
| Parameter | Default | Description |
|---|---|---|
| `init_pose` | `[0.0, 0.0, 0.0]` | Initial pose estimate (x, y, theta) |
| `scan_topic` | `scan` | Input lidar scan topic |
| `speed_topic` | `/qbot_speed_feedback` | Input wheel speed feedback topic |
| `imu_topic` | `imu` | Input IMU topic |
| `pose_topic` | `pose_estimate` | Output fused pose topic |
| `odom_topic` | `odom` | Output odometry topic |
| `scan_decimation` | `4` | Lidar downsampling factor |
| `odom_pub` | `True` | Toggle odometry publishing |
| `tf_pub` | `True` | Toggle tf broadcast |
| `reset_service_name` | `/reset_scan_match` | Name of the reset trigger service |
| `reset_pulse_ticks` | `50` | Number of ticks the reset service pulses `enable=0` before re-enabling |

Reset behavior: calling the reset service pulses the node's internal enable flag off for `reset_pulse_ticks` control cycles, then re-enables, used to reinitialize localization state at episode boundaries (e.g. by an RL client in `qbot_rl_deploy`).

## Launch

**`qbot_localization_launch.py`** brings up the full EKF localization stack. This is the only launch file in this package; it stages startup in order, waiting for each dependency to be confirmed running before starting the next:

1. QBot driver RT model
2. `ekf_scan_match` RT model (2s after the driver model starts)
3. `qbot_platform_driver_node`, `lidar_node`, `fixed_frame`, and
   `ekf_localization_node` together, 2s after the RT model starts

> Steps 1 and 3 depend on `qbot_hardware` — see [Dependency](#dependency) below.

```bash
ros2 launch qbot_localization qbot_localization_launch.py
```

## Dependency

**`qbot_hardware`** — `qbot_localization_launch.py` directly launches `qbot_platform_driver_interface`, `lidar`, and `fixed_lidar_frame` from this package by node name. It must be built and available in the same workspace to run localization.

Requires `ekf_scan_match.rt-linux_qbot_platform` (QUARC RT model) to be available on the target if running `ekf_localization_node`. This is installed under `share/qbot_localization/rt_models/` at build time.

## License

Apache-2.0