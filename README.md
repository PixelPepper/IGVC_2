# IGVC Simulation - Autonomous Robot Navigation

Autonomous navigation simulation for the IGVC (Intelligent Ground Vehicle Competition) course using ROS 2 Humble, Gazebo, and Navigation2. Features lane detection, LiDAR obstacle avoidance, and waypoint-based path following.

**Clone from GitHub** → `git clone https://github.com/PixelPepper/IGVC_2.git` → Works from any directory (no hardcoded paths).

---

## Features

- **Gazebo simulation** – Orange robot in IGVC-style world with lanes, cones, and obstacles
- **Lane detection** – HSV-based white/yellow lane segmentation via OAK-D Pro camera
- **Navigation2** – Path planning and obstacle avoidance with fused LiDAR + lane boundaries
- **Waypoint navigation** – Sequential waypoint following through the course
- **Robot localization** – EKF fusion of wheel odometry and IMU
- **RViz visualization** – Costmaps, paths, LiDAR, and camera feeds

---

## System Requirements

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS 2**: Humble
- **Gazebo**: Gazebo Classic (gazebo11)
- **Python**: 3.10+
- **Display**: For Gazebo and RViz (X11 or headless with virtual display)

---

## Dependencies

### ROS 2 Packages

Install ROS 2 Humble and common packages:

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

### Optional (if using additional packages)

```bash
# Linefit ground segmentation (orange_sensor_tools)
sudo apt install -y ros-humble-linefit-ground-segmentation-ros

# Velodyne (if using 3D LiDAR)
sudo apt install -y ros-humble-velodyne
```

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/PixelPepper/IGVC_2.git
cd IGVC_2
```

The repository is self-contained (no submodules).

### 2. Workspace path

`setup_orange.sh` auto-detects the workspace from its location—no path edits needed. Clone as `IGVC_2` (default) or any name; just `cd` into your clone directory before sourcing.

### 3. Build the workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Source the setup script

```bash
source setup_orange.sh
```

---

## Quick Start

### Launch simulation (4 terminals)

**Terminal 1 – Gazebo**
```bash
cd IGVC_2   # or your workspace path
source setup_orange.sh
ros2 launch orange_gazebo orange_igvc_simple.launch.py
```

**Terminal 2 – Navigation + Perception**
```bash
cd IGVC_2
source setup_orange.sh
ros2 launch orange_gazebo igvc_perception_full.launch.xml
```

**Terminal 3 – RViz (optional)**
```bash
cd IGVC_2
source setup_orange.sh
rviz2 -d igvc_perception.rviz
```

The saved config keeps **Local Costmap** off by default (avoids `indexed_8bit_image` GLSL errors on some GPUs). Enable it in RViz under Displays if you need the costmap.

If RViz still logs shader errors or fails to render images, use software OpenGL:

```bash
./launch_rviz_perception.sh --software
# or: IGVC_RVIZ_SOFTWARE_GL=1 ./launch_rviz_perception.sh
```

**Terminal 4 – Run course**
```bash
cd IGVC_2
source setup_orange.sh
python3 navigate_igvc_course.py
# or: ./run_igvc_course.sh
```

---

## Architecture

The system uses a layered ROS 2 architecture:

- **Simulation** (Gazebo) → **Localization** (EKF) → **Perception** (lane detection, fusion) → **Navigation2** → **cmd_vel** → Gazebo

Key topics: `/odom`, `/imu`, `/hokuyo_scan`, `/oak/rgb/image_raw` → EKF and perception → `/fused_scan`, `/lane_cloud` → Nav2 costmaps → `/cmd_vel`.

**Documentation:**

- [**docs/ARCHITECTURE.md**](docs/ARCHITECTURE.md) – In-depth architecture, layers, TF tree, config files
- [**docs/TOPIC_GRAPH.md**](docs/TOPIC_GRAPH.md) – Mermaid diagram and topic tables
- [**docs/SETUP_GUIDE.md**](docs/SETUP_GUIDE.md) – Step-by-step setup for new users

---

## Project Structure

```
IGVC_2/
├── src/
│   ├── orange_ros2/           # Core Orange robot packages
│   │   ├── orange_description # Robot URDF, sensors (OAK-D Pro, Hokuyo)
│   │   ├── orange_gazebo      # Gazebo worlds, spawn, launch
│   │   ├── orange_perception  # Lane detection, point cloud fusion
│   │   ├── orange_sensor_tools
│   │   ├── orange_navigation
│   │   └── ...
│   ├── linefit_ground_segmentation_ros2/
│   ├── velodyne/
│   └── ...
├── docs/                      # Architecture and setup documentation
│   ├── ARCHITECTURE.md        # System design, layers, TF tree
│   ├── TOPIC_GRAPH.md         # Topic diagram and tables
│   └── SETUP_GUIDE.md         # New user setup guide
├── navigate_igvc_course.py    # Waypoint navigation script
├── nav2_params_fused.yaml     # (legacy; primary: orange_gazebo/config/)
├── igvc_perception.rviz       # RViz config
├── setup_orange.sh            # Workspace setup
├── FINAL_SETUP_GUIDE.md       # Detailed setup instructions
└── README.md
```

---

## Configuration

- **Waypoints**: Edit `src/orange_ros2/orange_gazebo/config/waypoints/igvc_course_waypoints.yaml`
- **Nav2 params**: `orange_gazebo/config/nav2_params_fused.yaml` (costmaps, inflation, goal tolerance)
- **Lane detection**: `orange_perception` (HSV thresholds, morphological ops)
- **Robot spawn**: `orange_igvc_simple.launch.py` (kills stale Gazebo, then XML stack; or use `.launch.xml` alone — it waits ~2s after clearing port 11345). Ricardo sim: `base_link` aligns with `base_footprint` (no extra yaw) so Nav2 forward matches the mesh. Override `spawn_yaw:=3.14159` if you need the initial body heading from older configs.

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Gazebo window doesn't open | Run `DISPLAY=:0 gzclient &` in a separate terminal |
| RViz `indexed_8bit_image` / sampler GLSL errors | Use `./launch_rviz_perception.sh --software` or enable Local Costmap only after RViz starts (costmap Map display is off by default) |
| Lane detection shows 0 points | Check camera topic `/oak/rgb/image_raw`, adjust camera pitch/position |
| Robot drives over lines despite lane overlay in RViz | Use `igvc_perception_full.launch.xml` (not Nav2 with `nav2_params_no_map` only). `igvc_nav2_full` defaults to `nav2_params_fused.yaml` when you need lane costmaps. |
| Robot stuck at waypoint | Increase `xy_goal_tolerance` in `nav2_params_fused.yaml` |
| Build errors | Run `rosdep install --from-paths src --ignore-src -r -y` |

---

## License

Apache-2.0 (see package `package.xml` files)

---

## Acknowledgments

- [KBKN Autonomous Robotics Lab](https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2) – Original Orange robot repository
- ROS 2 Navigation2, Gazebo, OpenCV
