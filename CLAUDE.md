# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 workspace for **AquaRob**, an Unmanned Surface Vehicle (USV) simulator — PFE 2025/2026. The USV (named `wamv`) is simulated in Gazebo Harmonic and controlled via ROS2 Humble nodes in Python.

## Build & Run Commands

All commands must be run from the workspace root (`~/gazebo_maritime_ws`).

```bash
# Source ROS2 + workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Build the package
colcon build --packages-select ros2_maritime
source install/setup.bash  # required after every build

# Launch full simulation (Gazebo + bridge + all nodes)
ros2 launch ros2_maritime display.launch.py

# Run individual nodes
ros2 run ros2_maritime imu_relay
ros2 run ros2_maritime heading
ros2 run ros2_maritime navigator
ros2 run ros2_maritime dynamics_sim
ros2 run ros2_maritime gazebo_pose_updater
```

## Architecture

### Two simulation modes

1. **Gazebo-physics mode** (`display.launch.py`): Gazebo Harmonic simulates the USV dynamics. The `gz-sim-thruster-system` plugin applies thrust commands directly in-sim. `gazebo_pose_updater` is not needed here.

2. **Standalone dynamics mode**: `dynamics_sim` replaces Gazebo physics — it subscribes to thrust commands and publishes `/usv/odometry` based on a simplified 3-DOF maneuvering model (surge/sway/yaw). `gazebo_pose_updater` syncs this simulated pose back into Gazebo for visualization only.

### ROS2 ↔ Gazebo bridge (`bridge_config.yaml`)

`ros_gz_bridge` translates between Gazebo topics and ROS2 topics:
- Sensors (GZ→ROS): `/wamv/sensors/{gps,imu,magnetometer,altimeter}` → `/sensor/{gps,imu}`, `/imu/mag`, `/altimeter/data`
- Actuators (ROS→GZ): `/left_thrust_cmd`, `/right_thrust_cmd` → `/wamv/thrusters/{left,right}/thrust`

### Node data flow

```
Gazebo sensors
    │
    ├─ /sensor/imu ──→ [imu_relay] ──→ /sensor/imu_fixed ──→ [ekf_node (odom+map)]
    │                                                              │
    ├─ /imu/mag ──→ [heading] ──→ /usv/heading_deg ──→ [navigator]
    │                                                         │
    └─ /usv/odometry ←─────────── [dynamics_sim (standalone)] │
              │                                                │
              └─────────────────────────────────────→ [navigator]
                                                           │
                                        /left_thrust_cmd ←─┘
                                        /right_thrust_cmd ←┘
```

### Key nodes

| Node | File | Role |
|------|------|------|
| `imu_relay` | `imu_relay.py` | Sets `frame_id="base_link"` on IMU messages (required by EKF) |
| `heading` | `heading.py` | Magnetometer → heading in degrees (0=North); applies magnetic declination (0.0087 rad) |
| `navigator` | `navigator.py` | Dual PID (yaw + thrust) GPS waypoint follower; converts odometry position to approximate GPS using a fixed origin (36.7538°N, 3.0588°E) |
| `dynamics_sim` | `dynamics_sim.py` | 3-DOF maneuvering model: m=180 kg, Iz=446 kg·m², damping Xu=-50, Yv=-100, Nr=-100 |
| `gazebo_pose_updater` | `gazebo_pose_updater.py` | Pushes ROS odometry back to Gazebo at 10 Hz via `gz service` subprocess call (world: `usv_monde_leger`) |

### Simulation world (`monde_usv_leger.sdf`)

- World origin: 36.7538°N, 3.0588°E (Algiers area)
- USV model `wamv`: static body (physics handled externally), two thrusters at `(−1.5, ±0.8, 0)` m
- Sensors on `base_link`: IMU@100 Hz, GPS@10 Hz, Magnetometer@50 Hz

### Navigator PID parameters (current tuning)

- Yaw PID: Kp=0.2, Ki=0.001, Kd=0.01 (integral clamped ±5, output clamped ±40)
- Thrust PID: Kp=0.15, Ki=0.0005, Kd=0.0 (integral clamped 0–20, output clamped 0–50 N)
- Default waypoint target: 36.7548°N, 3.0588°E; acceptance radius: 2 m
- Run at 10 Hz (`dt=0.1 s`)

### EKF localization (`ekf_filter.yaml`)

Two `robot_localization` EKF nodes run in parallel:
- `ekf_filter_node_odom`: world_frame=`odom`, publishes to `/odometry/local`
- `ekf_filter_node_map`: world_frame=`map`

Both fuse only IMU orientation (yaw) and yaw rate (`imu0_config` enables indices 5 and 11 only).
