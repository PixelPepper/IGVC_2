# IGVC Simulation - Setup Guide for New Users

Step-by-step setup and first-run instructions.

---

## Prerequisites

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS 2**: Humble
- **Gazebo**: Gazebo Classic (gazebo11)
- **Python**: 3.10+
- **Display**: X11 (or virtual display for headless)

---

## 1. Install Dependencies

### ROS 2 Packages

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-nav2-bringup ros-humble-navigation2
sudo apt install -y ros-humble-robot-localization
sudo apt install -y ros-humble-cv-bridge ros-humble-vision-opencv
sudo apt install -y ros-humble-pcl-ros ros-humble-pcl-conversions
sudo apt install -y ros-humble-pointcloud-to-laserscan
sudo apt install -y ros-humble-robot-state-publisher ros-humble-joint-state-publisher
sudo apt install -y ros-humble-xacro ros-humble-urdf
sudo apt install -y ros-humble-gazebo-plugins
sudo apt install -y ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install -y ros-humble-laser-geometry
sudo apt install -y ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-nav-msgs
```

### Python / System

```bash
sudo apt install -y python3-opencv python3-numpy python3-pip
sudo apt install -y libpcl-dev pcl-tools
```

---

## 2. Clone and Build

```bash
git clone https://github.com/PixelPepper/IGVC_2.git
cd IGVC_2
```

Build the workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Source the project setup script:

```bash
source setup_orange.sh
```

---

## 3. First Run (4 Terminals)

### Terminal 1 – Gazebo

```bash
cd IGVC_2   # or your workspace path
source setup_orange.sh
ros2 launch orange_gazebo orange_igvc_simple.launch.xml
```

**Wait for:** `Successfully spawned entity [orange_robot]` and Gazebo window.

If Gazebo window doesn't open:

```bash
DISPLAY=:0 gzclient &
```

---

### Terminal 2 – Perception + Navigation

```bash
cd IGVC_2
source setup_orange.sh
ros2 launch orange_gazebo igvc_perception_full.launch.xml
```

**Wait for (15–20 seconds):**

- `[lane_detector]: Lane Detector initialized`
- `[lane_detector]: ✓ Detected XXXX lane wall points`
- `[local_costmap]: Subscribed to Topics: fused_scan lane_cloud`
- `[lifecycle_manager_navigation]: Managed nodes are active`

---

### Terminal 3 – RViz (optional)

```bash
cd IGVC_2
source setup_orange.sh
rviz2 -d igvc_perception.rviz
```

Or use the helper script:

```bash
./launch_rviz_perception.sh
```

---

### Terminal 4 – Run Course

```bash
cd IGVC_2
source setup_orange.sh
python3 navigate_igvc_course.py
# or: ./run_igvc_course.sh
```

---

## 4. Verify Setup

```bash
# Sensors
ros2 topic hz /hokuyo_scan          # ~10 Hz
ros2 topic hz /oak/rgb/image_raw    # ~5–30 Hz
ros2 topic hz /lane_cloud           # ~1 Hz
ros2 topic hz /fused_scan           # ~10 Hz

# Nav2
ros2 action list | grep navigate_to_pose

# Odometry
ros2 topic echo /fusion/odom --field pose.pose.position
```

---

## 5. Configuration

| What | File / Location |
|------|-----------------|
| Waypoints | `navigate_igvc_course.py` → `self.waypoints` |
| Nav2 params | `orange_gazebo/config/nav2_params_fused.yaml` |
| Lane detection | `orange_perception` (HSV, morphology) |
| Robot spawn | `orange_igvc_simple.launch.xml` |

---

## 6. Troubleshooting

| Issue | Solution |
|-------|----------|
| Gazebo window doesn't open | `DISPLAY=:0 gzclient &` |
| Lane detection shows 0 points | Check `/oak/rgb/image_raw`, adjust camera |
| Robot stuck at waypoint | Increase `xy_goal_tolerance` in `nav2_params_fused.yaml` |
| Build errors | `rosdep install --from-paths src --ignore-src -r -y` |
| Robot not moving | Check `ros2 action list`, `ros2 topic echo /cmd_vel` |

---

## 7. Further Documentation

- **[ARCHITECTURE.md](ARCHITECTURE.md)** – System design and data flow
- **[TOPIC_GRAPH.md](TOPIC_GRAPH.md)** – Topic diagram and tables
- **[FINAL_SETUP_GUIDE.md](../FINAL_SETUP_GUIDE.md)** – Detailed setup and tuning
