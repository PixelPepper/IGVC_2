# IGVC Simulation - Autonomous Robot Navigation

Autonomous navigation simulation for the IGVC (Intelligent Ground Vehicle Competition) course using ROS 2 Humble, Gazebo, and Navigation2. Features lane detection, LiDAR obstacle avoidance, and waypoint-based path following.

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

### 2. Workspace path

`setup_orange.sh` auto-detects the workspace from its location—no path edits needed. Just `cd` into your clone directory before sourcing.

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
ros2 launch orange_gazebo orange_igvc_simple.launch.xml
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

**Terminal 4 – Run course**
```bash
cd IGVC_2
source setup_orange.sh
python3 navigate_igvc_course.py
# or: ./run_igvc_course.sh
```

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
├── navigate_igvc_course.py    # Waypoint navigation script
├── nav2_params_fused.yaml     # Nav2 costmap, planner, controller params
├── igvc_perception.rviz       # RViz config
├── setup_orange.sh            # Workspace setup
├── FINAL_SETUP_GUIDE.md       # Detailed setup instructions
└── README.md
```

---

## Configuration

- **Waypoints**: Edit `navigate_igvc_course.py` (`self.waypoints`)
- **Nav2 params**: `nav2_params_fused.yaml` (costmaps, inflation, goal tolerance)
- **Lane detection**: `orange_perception` (HSV thresholds, morphological ops)
- **Robot spawn**: `orange_igvc_simple.launch.xml` (position, yaw)

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Gazebo window doesn't open | Run `DISPLAY=:0 gzclient &` in a separate terminal |
| Lane detection shows 0 points | Check camera topic `/oak/rgb/image_raw`, adjust camera pitch/position |
| Robot stuck at waypoint | Increase `xy_goal_tolerance` in `nav2_params_fused.yaml` |
| Build errors | Run `rosdep install --from-paths src --ignore-src -r -y` |

---

## License

Apache-2.0 (see package `package.xml` files)

---

## Acknowledgments

- [KBKN Autonomous Robotics Lab](https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2) – Original Orange robot repository
- ROS 2 Navigation2, Gazebo, OpenCV
