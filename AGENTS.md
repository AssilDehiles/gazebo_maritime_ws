# AGENTS.md

This file describes conventions and guidance for AI coding agents working in this repository.

## Project Context

ROS2/Gazebo workspace for **AquaRob**, an Unmanned Surface Vehicle (USV) simulator (PFE 2025/2026).
- ROS2 Humble + Gazebo Harmonic on Ubuntu
- Main package: `ros2_maritime` in `src/ros2_maritime/`
- Simulation world assets: `src/gazebo_maritime/worlds/`
- USV model name in Gazebo: `wamv`

---

## Build & Verification

Before reporting any task complete, verify the build succeeds:

```bash
cd ~/gazebo_maritime_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_maritime
source install/setup.bash
```

There are no automated tests. After building, do a basic smoke-check with:

```bash
ros2 run ros2_maritime <node_name>  # confirm it starts without errors
```

---

## Repository Layout

```
src/
├── gazebo_maritime/
│   └── worlds/monde_usv_leger.sdf      # Gazebo world (Algiers, wamv model)
└── ros2_maritime/
    ├── config/
    │   ├── bridge_config.yaml           # ROS2 ↔ Gazebo topic bridge
    │   └── ekf_filter.yaml             # robot_localization EKF config
    ├── launch/display.launch.py         # Full simulation launcher
    ├── rviz/usv_nav.rviz               # RViz2 config
    └── ros2_maritime/                   # Python nodes
        ├── imu_relay.py
        ├── heading.py
        ├── navigator.py
        ├── dynamics_sim.py
        └── gazebo_pose_updater.py
```

---

## Node Reference

### `imu_relay` — [imu_relay.py](src/ros2_maritime/ros2_maritime/imu_relay.py)

Passes `/sensor/imu` through with `header.frame_id` forced to `"base_link"`.
Required because EKF rejects IMU messages without a recognized frame.

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/sensor/imu` | `sensor_msgs/Imu` |
| Pub | `/sensor/imu_fixed` | `sensor_msgs/Imu` |

---

### `heading` — [heading.py](src/ros2_maritime/ros2_maritime/heading.py)

Converts raw magnetometer field to heading in degrees (0 = North, 90 = East).
Applies magnetic declination (default `0.0087 rad`).

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/imu/mag` | `sensor_msgs/MagneticField` |
| Pub | `/usv/heading_deg` | `std_msgs/Float64` |

Parameter: `magnetic_declination` (float, radians)

---

### `navigator` — [navigator.py](src/ros2_maritime/ros2_maritime/navigator.py)

GPS waypoint follower running at 10 Hz. Uses two PID loops:
- **Yaw PID** — corrects heading error → differential thrust
- **Thrust PID** — scales base thrust from distance to target

Position source: converts `/usv/odometry` local (x, y) metres back to approximate GPS
using the fixed world origin `(36.7538°N, 3.0588°E)`.

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/usv/odometry` | `nav_msgs/Odometry` |
| Sub | `/usv/heading_deg` | `std_msgs/Float64` |
| Pub | `/left_thrust_cmd` | `std_msgs/Float64` |
| Pub | `/right_thrust_cmd` | `std_msgs/Float64` |

Parameters:

| Name | Default | Notes |
|------|---------|-------|
| `target_lat` | `36.7548` | Waypoint latitude |
| `target_lon` | `3.0588` | Waypoint longitude |
| `acceptance_radius` | `2.0` m | Goal tolerance |

PID tuning (hardcoded):

| Loop | Kp | Ki | Kd | Integral clamp | Output clamp |
|------|----|----|----|----------------|--------------|
| Yaw | 0.2 | 0.001 | 0.01 | ±5 | ±40 |
| Thrust | 0.15 | 0.0005 | 0.0 | 0–20 | 0–50 N |

Thrust mixing: `left = base − yaw_correction`, `right = base + yaw_correction`.

---

### `dynamics_sim` — [dynamics_sim.py](src/ros2_maritime/ros2_maritime/dynamics_sim.py)

Standalone 3-DOF maneuvering model (surge `u`, sway `v`, yaw rate `r`) running at 100 Hz.
Replaces Gazebo physics when operating without the thruster plugin.

Physical parameters (hardcoded):

| Symbol | Value | Unit |
|--------|-------|------|
| m | 180 | kg |
| Iz | 446 | kg·m² |
| Xu (surge drag) | −50 | N·s/m |
| Yv (sway drag) | −100 | N·s/m |
| Nr (yaw drag) | −100 | N·m·s/rad |
| L (thruster separation) | 2.0 | m |

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/left_thrust_cmd` | `std_msgs/Float64` |
| Sub | `/right_thrust_cmd` | `std_msgs/Float64` |
| Pub | `/usv/odometry` | `nav_msgs/Odometry` |

---

### `gazebo_pose_updater` — [gazebo_pose_updater.py](src/ros2_maritime/ros2_maritime/gazebo_pose_updater.py)

Syncs ROS odometry back into Gazebo at 10 Hz via `gz service` subprocess calls.
Only needed in standalone dynamics mode — not used when Gazebo physics drives the USV.

Calls: `/world/usv_monde_leger/set_pose` (Gazebo service)

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/usv/odometry` | `nav_msgs/Odometry` |

---

## Topic Map

```
Gazebo (/wamv/sensors/...)
    │
    ├─ imu          GZ→ROS  /sensor/imu     → [imu_relay] → /sensor/imu_fixed
    ├─ magnetometer GZ→ROS  /imu/mag        → [heading]   → /usv/heading_deg
    ├─ gps          GZ→ROS  /sensor/gps     (unused by current nodes)
    └─ altimeter    GZ→ROS  /altimeter/data (unused by current nodes)

[dynamics_sim] → /usv/odometry ─┬→ [navigator] → /left_thrust_cmd  → GZ thruster
                                 │               → /right_thrust_cmd → GZ thruster
                                 └→ [gazebo_pose_updater] (standalone mode only)

[imu_relay] → /sensor/imu_fixed → [ekf_node_odom] → /odometry/local
                                 → [ekf_node_map]
```

---

## Two Simulation Modes

### Gazebo-physics mode (default via `display.launch.py`)

Gazebo Harmonic simulates USV dynamics through the `gz-sim-thruster-system` plugin.
`dynamics_sim` and `gazebo_pose_updater` are **not** launched.

```bash
ros2 launch ros2_maritime display.launch.py
```

### Standalone dynamics mode

`dynamics_sim` computes the USV state; `gazebo_pose_updater` pushes it back into Gazebo
for visualization only. Run manually:

```bash
ros2 run ros2_maritime dynamics_sim
ros2 run ros2_maritime gazebo_pose_updater
```

---

## Coding Conventions

- All nodes are single-file Python classes inheriting from `rclpy.node.Node`.
- No launch arguments currently — parameters are declared with `declare_parameter` and read at startup.
- Log messages are in French (project convention).
- Physical constants and PID gains are hardcoded directly in `__init__`; do not move them to YAML without updating CLAUDE.md.
- The world origin `(36.7538°N, 3.0588°E)` is hardcoded in `navigator.py:49-50`; any GPS ↔ local coordinate conversion must use this same constant.
- `dt` in `navigator.py` (`0.1 s`) must match the timer period (`0.1 s`); keep them in sync.

---

## Known Issues / Active Work

- Navigator heading error oscillates due to PID tuning; yaw gains are under active adjustment.
- EKF fuses only IMU yaw and yaw rate (indices 5 and 11); GPS is not fused into EKF.
- `gazebo_pose_updater` uses `subprocess.Popen` (fire-and-forget) — `gz service` latency can cause pose lag at high update rates.

---

## What Agents Should NOT Change Without Discussion

- World origin constants (`36.7538`, `3.0588`) — changing these breaks GPS↔local conversion across multiple nodes.
- `world_frame` in `ekf_filter.yaml` — the two EKF nodes intentionally use different frames (`odom` vs `map`).
- Gazebo world name `usv_monde_leger` — hardcoded in `gazebo_pose_updater.py` and `display.launch.py`.
- Thruster positions in the SDF — physically constrained by the wamv model geometry.
