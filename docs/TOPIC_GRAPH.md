# IGVC Simulation - Topic Graph

Visual representation of ROS 2 topics and data flow.

---

## Mermaid Flowchart

```mermaid
flowchart TB
    subgraph GAZEBO["Gazebo Simulation"]
        GZ[gzserver]
        GZ --> ODOM[/odom]
        GZ --> IMU[/imu]
        GZ --> HOKUYO[/hokuyo_scan]
        GZ --> RGB[/oak/rgb/image_raw]
        GZ --> DEPTH[/oak/stereo/points]
        GZ --> JOINT[/joint_states]
    end

    subgraph LOCALIZATION["Localization"]
        EKF[robot_localization EKF]
        ODOM --> EKF
        IMU --> EKF
        EKF --> FUSION[/fusion/odom]
        EKF --> TF_ODOM[odom→base_footprint TF]
    end

    subgraph PERCEPTION["Perception"]
        LANE[lane_detector]
        FUSION_NODE[pointcloud_fusion]
        PCL2LASER[pointcloud_to_laserscan]
        RGB --> LANE
        LANE --> LANE_CLOUD[/lane_cloud]
        HOKUYO --> FUSION_NODE
        DEPTH --> FUSION_NODE
        LANE_CLOUD --> FUSION_NODE
        FUSION_NODE --> FUSED_CLOUD[/fused_cloud]
        FUSED_CLOUD --> PCL2LASER
        PCL2LASER --> FUSED_SCAN[/fused_scan]
    end

    subgraph NAV2["Navigation2"]
        LOCAL_CM[local_costmap]
        GLOBAL_CM[global_costmap]
        PLANNER[planner_server]
        CTRL[controller_server]
        BT[bt_navigator]
        FUSED_SCAN --> LOCAL_CM
        FUSED_SCAN --> GLOBAL_CM
        LANE_CLOUD --> LOCAL_CM
        LANE_CLOUD --> GLOBAL_CM
        FUSION --> CTRL
        LOCAL_CM --> CTRL
        GLOBAL_CM --> PLANNER
        PLANNER --> BT
        CTRL --> BT
        BT --> CMD[/cmd_vel]
    end

    subgraph WAYPOINT["Waypoint Script"]
        NAV_SCRIPT[navigate_igvc_course.py]
        NAV_SCRIPT -->|navigate_to_pose action| BT
    end

    CMD --> GZ
```

---

## Topic Summary Table

| Topic | Type | Publisher | Subscriber(s) | Purpose |
|-------|------|-----------|---------------|---------|
| `/odom` | `nav_msgs/Odometry` | Gazebo diff_drive | EKF | Wheel odometry |
| `/imu` | `sensor_msgs/Imu` | Gazebo IMU plugin | EKF | Orientation, angular velocity |
| `/hokuyo_scan` | `sensor_msgs/LaserScan` | Gazebo Hokuyo | pointcloud_fusion | 2D LiDAR |
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | Gazebo OAK-D | lane_detector | RGB camera |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | Gazebo OAK-D | lane_detector | Camera intrinsics |
| `/oak/stereo/points` | `sensor_msgs/PointCloud2` | Gazebo OAK-D | pointcloud_fusion | Depth point cloud |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo | robot_state_publisher | Joint positions |
| `/fusion/odom` | `nav_msgs/Odometry` | EKF | controller_server | Fused odometry |
| `/lane_cloud` | `sensor_msgs/PointCloud2` | lane_detector | pointcloud_fusion, costmaps | Lane boundaries |
| `/lane_debug` | `sensor_msgs/Image` | lane_detector | (RViz) | Debug visualization |
| `/fused_cloud` | `sensor_msgs/PointCloud2` | pointcloud_fusion | pointcloud_to_laserscan | Merged point cloud |
| `/fused_scan` | `sensor_msgs/LaserScan` | pointcloud_to_laserscan | costmaps | LiDAR + depth + lanes as LaserScan |
| `/cmd_vel` | `geometry_msgs/Twist` | controller_server | Gazebo diff_drive | Velocity commands |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | local_costmap | RViz, controller | Local obstacle map |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | global_costmap | RViz, planner | Global obstacle map |
| `/plan` | `nav_msgs/Path` | planner_server | RViz, controller | Global path |
| `/received_global_plan` | `nav_msgs/Path` | bt_navigator | controller_server | Path to follow |

---

## Action Servers

| Action | Server | Client | Purpose |
|--------|--------|--------|---------|
| `navigate_to_pose` | bt_navigator | navigate_igvc_course.py, send_waypoint.sh | Goal-based navigation |
| `follow_path` | controller_server | bt_navigator | Path following |
| `compute_path_to_pose` | planner_server | bt_navigator | Global path planning |

---

## TF Frames

| Parent | Child | Publisher |
|--------|-------|-----------|
| `odom` | `base_footprint` | EKF (robot_localization) |
| `base_footprint` | `base_link` | robot_state_publisher |
| `base_link` | `hokuyo_link` | robot_state_publisher |
| `base_link` | `oak_link` | robot_state_publisher |
| `base_link` | `imu_link` | robot_state_publisher |
| `oak_link` | `oak_rgb_camera_optical_frame` | robot_state_publisher |
| `oak_link` | `oak_left_camera_optical_frame` | robot_state_publisher |

---

## Quick Topic Checks

```bash
# Verify sensors
ros2 topic hz /hokuyo_scan          # ~10 Hz
ros2 topic hz /oak/rgb/image_raw    # ~5-30 Hz
ros2 topic hz /lane_cloud            # ~1 Hz
ros2 topic hz /fused_scan           # ~10 Hz

# Verify localization
ros2 topic hz /fusion/odom

# Verify commands
ros2 topic echo /cmd_vel
```
