# IGVC Simulation - ROS 2 Architecture

In-depth analysis of the ROS 2 architecture and system design.

---

## 1. High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                         IGVC SIMULATION - ROS 2 ARCHITECTURE                       │
└─────────────────────────────────────────────────────────────────────────────────┘

┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐
│   TERMINAL 1     │     │   TERMINAL 2     │     │   TERMINAL 3/4   │
│   Gazebo Stack   │     │  Nav2 + Percep  │     │  RViz / Waypoint  │
└────────┬─────────┘     └────────┬────────┘     └────────┬─────────┘
         │                        │                       │
         ▼                        ▼                       ▼
┌────────────────────────────────────────────────────────────────────────────┐
│                           ROS 2 TOPIC / ACTION FLOW                          │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  GAZEBO                    PERCEPTION                    NAVIGATION2        │
│  ───────                   ───────────                    ───────────       │
│  /odom ──────────────────► EKF ────────────────────────► /fusion/odom       │
│  /imu ───────────────────►   │                                    │         │
│  /hokuyo_scan ───────────────┼──────────────────────────► fused_scan ───────┤
│  /oak/rgb/image_raw ────────► lane_detector ──► /lane_cloud ────────────────┤
│  /oak/stereo/points ─────────► pointcloud_fusion ──► /fused_cloud           │
│  /joint_states               │                        │                     │
│  /robot_description          pointcloud_to_laserscan   │                     │
│       │                      │                        ▼                     │
│       │                      │              local_costmap ◄── fused_scan     │
│       │                      │              global_costmap ◄── lane_cloud    │
│       │                      │                        │                     │
│       │                      │              planner_server                   │
│       │                      │              controller_server               │
│       │                      │              bt_navigator                    │
│       │                      │                        │                     │
│  /cmd_vel ◄───────────────────────────────────────────┘                     │
│       ▲                                                                     │
│       │              navigate_to_pose (action) ◄── navigate_igvc_course.py  │
│                                                                             │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. System Layers

### Layer 1: Simulation (Gazebo)

| Component | Source | Output | Purpose |
|-----------|--------|--------|---------|
| **gzserver** | `orange_igvc.world` | Physics, sensors | Simulate world and robot |
| **spawn_entity** | `/robot_description` | Spawns `orange_robot` | Load robot model |
| **robot_state_publisher** | `orange_robot_simulation.xacro` | TF tree | Publish robot links |
| **diff_drive_controller** | `cmd_vel` | `/odom`, wheel TF | Wheel odometry |
| **Hokuyo plugin** | Gazebo rays | `/hokuyo_scan` | 2D LiDAR |
| **OAK-D Pro plugin** | Gazebo camera | `/oak/rgb/image_raw`, `/oak/stereo/points` | RGB + depth |
| **IMU plugin** | Gazebo IMU | `/imu` | Orientation, angular velocity |

### Layer 2: Localization

| Component | Inputs | Output | Purpose |
|-----------|--------|--------|---------|
| **robot_localization (EKF)** | `/odom`, `/imu` | `/fusion/odom`, `odom→base_footprint` TF | Fuse wheel odom + IMU |

### Layer 3: Perception

| Component | Inputs | Outputs | Purpose |
|-----------|--------|---------|---------|
| **lane_detector** | `/oak/rgb/image_raw`, `/oak/rgb/camera_info` | `/lane_cloud`, `/lane_debug` | White/yellow lane → PointCloud2 |
| **pointcloud_fusion** | `/hokuyo_scan`, `/oak/stereo/points`, `/lane_cloud` | `/fused_cloud` | Merge LiDAR + depth + lanes |
| **pointcloud_to_laserscan** | `/fused_cloud` | `/fused_scan` | PointCloud2 → LaserScan for Nav2 |

### Layer 4: Navigation2

| Component | Inputs | Outputs | Purpose |
|-----------|--------|---------|---------|
| **controller_server** | `/fusion/odom`, costmaps, path | `/cmd_vel` | DWB local planner |
| **planner_server** | Goal, costmap | Global path | NavfnPlanner |
| **bt_navigator** | `navigate_to_pose` action | Orchestrates Nav2 | Behavior tree |
| **local_costmap** | `/fused_scan`, `/lane_cloud` | Obstacle layer | Local obstacles |
| **global_costmap** | `/fused_scan`, `/lane_cloud` | Obstacle layer | Global obstacles |

### Layer 5: Waypoint Navigation

| Component | Inputs | Outputs | Purpose |
|-----------|--------|---------|---------|
| **navigate_igvc_course.py** | Waypoints list | `navigate_to_pose` goals | Sequential waypoint following |

---

## 3. TF Tree (Simplified)

```
odom (EKF publishes odom → base_footprint)
  └── base_footprint
        └── base_link (robot_state_publisher from URDF)
              ├── base_joint
              ├── hokuyo_link (LiDAR at 0.1, 0, 0.645)
              ├── oak_link (camera at 0, 0, 1.0)
              │     ├── oak_rgb_camera_optical_frame
              │     └── oak_left_camera_optical_frame (depth)
              ├── imu_link
              └── wheel links (left_wheel_hinge, right_wheel_hinge)
```

---

## 4. Key Configuration Files

| File | Purpose |
|------|---------|
| `orange_robot_simulation.xacro` | Robot URDF, sensor mounts (camera, LiDAR, IMU) |
| `orange_robot_simulation.gazebo` | Gazebo plugins (diff_drive, joint_state_publisher) |
| `nav2_params_fused.yaml` | Nav2 costmaps, planner, controller, goal tolerance |
| `ekf_node.yaml` | EKF sensor config, process noise |
| `perception.launch.xml` | Lane detector + fusion + pointcloud_to_laserscan params |

---

## 5. Launch Dependencies

```
orange_igvc_simple.launch.xml
├── robot_localization (localization.launch.xml)
│   └── ekf_node
├── gzserver (Gazebo)
├── gzclient (Gazebo GUI)
├── robot_state_publisher
└── spawn_entity

igvc_perception_full.launch.xml
├── perception.launch.xml
│   ├── lane_detector
│   ├── pointcloud_fusion
│   ├── pointcloud_to_laserscan_node
│   └── static_transform_publisher (oak_to_base_link)
├── controller_server
├── planner_server
├── behavior_server
├── bt_navigator
└── lifecycle_manager_navigation
```

---

## 6. Package Dependencies

```
orange_gazebo
├── orange_description (URDF, sensors)
├── orange_sensor_tools (localization)
├── robot_state_publisher
└── gazebo_ros_pkgs

orange_perception
├── cv_bridge (OpenCV ↔ ROS)
├── sensor_msgs
├── pcl_ros
└── pointcloud_to_laserscan (external)

orange_sensor_tools
├── robot_localization (EKF)
├── tf2, tf2_ros
└── sensor_msgs
```

---

## 7. Data Flow Summary

1. **Gazebo** publishes `/odom`, `/imu`, `/hokuyo_scan`, `/oak/rgb/image_raw`, `/oak/stereo/points`
2. **EKF** fuses `/odom` + `/imu` → `/fusion/odom` (used by Nav2)
3. **lane_detector** converts camera image → `/lane_cloud` (PointCloud2 of lane boundaries)
4. **pointcloud_fusion** merges LiDAR + depth + lanes → `/fused_cloud`
5. **pointcloud_to_laserscan** converts `/fused_cloud` → `/fused_scan` (LaserScan)
6. **Nav2 costmaps** use `/fused_scan` and `/lane_cloud` as obstacle sources
7. **Nav2 controller** outputs `/cmd_vel` → Gazebo diff_drive
8. **navigate_igvc_course.py** sends `navigate_to_pose` goals to bt_navigator
