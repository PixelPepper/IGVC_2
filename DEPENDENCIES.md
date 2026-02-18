# Dependencies

Complete list of dependencies for the IGVC Simulation project.

---

## ROS 2 Distribution

- **ROS 2 Humble Hawksbill** (Ubuntu 22.04)

---

## Core ROS 2 Packages

| Package | Purpose | Install |
|---------|---------|---------|
| `ros-humble-desktop` | ROS 2 base | `sudo apt install ros-humble-desktop` |
| `ros-humble-gazebo-ros-pkgs` | Gazebo integration | `sudo apt install ros-humble-gazebo-ros-pkgs` |
| `ros-humble-nav2-bringup` | Navigation2 stack | `sudo apt install ros-humble-nav2-bringup` |
| `ros-humble-navigation2` | Nav2 packages | `sudo apt install ros-humble-navigation2` |
| `ros-humble-robot-localization` | EKF odometry fusion | `sudo apt install ros-humble-robot-localization` |
| `ros-humble-robot-state-publisher` | URDF → TF | `sudo apt install ros-humble-robot-state-publisher` |
| `ros-humble-joint-state-publisher` | Joint states | `sudo apt install ros-humble-joint-state-publisher` |

---

## Perception & Vision

| Package | Purpose |
|---------|---------|
| `ros-humble-cv-bridge` | OpenCV ↔ ROS images |
| `ros-humble-vision-opencv` | OpenCV bindings |
| `ros-humble-pcl-ros` | Point Cloud Library ROS |
| `ros-humble-pcl-conversions` | PCL ↔ ROS conversions |
| `ros-humble-pointcloud-to-laserscan` | 3D → 2D scan |

---

## Sensors & TF

| Package | Purpose |
|---------|---------|
| `ros-humble-sensor-msgs` | Image, LaserScan, etc. |
| `ros-humble-tf2` | Transform library |
| `ros-humble-tf2-ros` | TF2 ROS interface |
| `ros-humble-tf2-geometry-msgs` | Geometry conversions |
| `ros-humble-laser-geometry` | Laser ↔ PointCloud |
| `ros-humble-geometry-msgs` | Pose, Twist, etc. |
| `ros-humble-nav-msgs` | Path, Odometry |

---

## Robot Description

| Package | Purpose |
|---------|---------|
| `ros-humble-xacro` | XML macros for URDF |
| `ros-humble-urdf` | URDF parsing |
| `ros-humble-gazebo-plugins` | Gazebo ROS plugins |

---

## System Libraries

| Package | Purpose |
|---------|---------|
| `libpcl-dev` | Point Cloud Library |
| `python3-opencv` | OpenCV for Python |
| `python3-numpy` | NumPy |

---

## One-Line Install (Ubuntu 22.04)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-robot-localization \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-urdf \
  ros-humble-gazebo-plugins \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-laser-geometry \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  python3-opencv \
  python3-numpy \
  libpcl-dev
```

---

## Workspace Packages (Built from source)

Built via `colcon build` from this repo:

- `orange_description`
- `orange_gazebo`
- `orange_perception`
- `orange_sensor_tools`
- `orange_navigation`
- `linefit_ground_segmentation_ros`
- And others in `src/`
