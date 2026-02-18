# 🎉 IGVC Enhanced Perception System - Implementation Summary

## What Was Built

A complete sensor fusion and lane detection system for the IGVC autonomous robot, integrating:

1. **OAK-D Pro RGB-D Camera** (emulated in Gazebo)
2. **Lane Segmentation** (computer vision-based)
3. **Multi-Sensor Point Cloud Fusion** (LiDAR + Depth + Lanes)
4. **Navigation2 Integration** (with fused perception)

---

## 📁 New Files Created

### 1. Robot Description
- **`src/orange_ros2/orange_description/xacro/sensors/oak_d_pro.xacro`**
  - Complete OAK-D Pro camera model (RGB + Stereo depth)
  - Gazebo camera and depth sensor plugins
  - Proper TF frames for camera optical frames

### 2. Perception Package
- **`src/orange_ros2/orange_perception/`** (NEW Python package)
  
  **Nodes:**
  - `orange_perception/lane_detector.py` - Lane segmentation using HSV color detection
  - `orange_perception/pointcloud_fusion.py` - Fuses LiDAR, depth, and lane point clouds
  
  **Launch:**
  - `launch/perception.launch.xml` - Launches full perception stack

### 3. Configuration Files
- **`nav2_params_fused.yaml`**
  - Updated Nav2 configuration to use fused scan
  - Optimized for multi-sensor obstacle detection

### 4. Launch Files
- **`src/orange_ros2/orange_gazebo/launch/igvc_perception_full.launch.xml`**
  - Combined perception + navigation launch file

### 5. Documentation
- **`PERCEPTION_SETUP.md`** - Complete guide for perception system
- **`QUICKSTART_PERCEPTION.sh`** - Interactive startup script
- **`IMPLEMENTATION_SUMMARY.md`** - This file

---

## 🔧 Modified Files

### Robot URDF
**`src/orange_ros2/orange_description/xacro/orange_robot_simulation.xacro`**
```xml
<!-- Added OAK-D Pro camera -->
<xacro:include filename="$(find orange_description)/xacro/sensors/oak_d_pro.xacro"/>
<xacro:oak_d_pro name="oak" parent="base_link">
  <origin xyz="0.150 0.0 0.200" rpy="0.0 0.0 0.0"/>
</xacro:oak_d_pro>
```

---

## 🌊 Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    SENSOR LAYER                              │
├─────────────────────────────────────────────────────────────┤
│ Hokuyo 2D LiDAR          →  /hokuyo_scan (LaserScan)       │
│ OAK-D Pro RGB Camera     →  /oak/rgb/image_raw (Image)     │
│ OAK-D Pro Depth Camera   →  /oak/stereo/points (PC2)       │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                 PERCEPTION LAYER                             │
├─────────────────────────────────────────────────────────────┤
│ Lane Detector Node                                          │
│  - Input: /oak/rgb/image_raw                                │
│  - Processing: HSV color segmentation (white/yellow)        │
│  - Output: /lane_cloud (PointCloud2)                        │
│                                                              │
│ Point Cloud Fusion Node                                     │
│  - Inputs: /hokuyo_scan, /oak/stereo/points, /lane_cloud  │
│  - Processing: Merge + weight lanes + downsample           │
│  - Output: /fused_cloud (PointCloud2)                       │
│                                                              │
│ PointCloud to LaserScan Converter                           │
│  - Input: /fused_cloud                                      │
│  - Processing: Project 3D → 2D scan                         │
│  - Output: /fused_scan (LaserScan)                          │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                  NAVIGATION LAYER                            │
├─────────────────────────────────────────────────────────────┤
│ Nav2 Costmap                                                │
│  - Input: /fused_scan                                       │
│  - Processing: Obstacle detection + inflation               │
│  - Output: /local_costmap/costmap (OccupancyGrid)          │
│                                                              │
│ Nav2 Planner                                                │
│  - Computes global path avoiding all obstacles             │
│                                                              │
│ Nav2 Controller                                             │
│  - Follows path while avoiding dynamic obstacles           │
│  - Output: /cmd_vel (robot motion commands)                │
└─────────────────────────────────────────────────────────────┘
```

---

## 🎯 Features Implemented

### 1. OAK-D Pro Camera Emulation
✅ RGB camera (1920x1080 @ 30Hz)
✅ Stereo depth (640x400 @ 30Hz)
✅ Point cloud generation
✅ Proper TF frames
✅ Gazebo sensor plugins

### 2. Lane Segmentation
✅ HSV color-based detection (white + yellow)
✅ Morphological cleanup (remove noise)
✅ Inverse perspective mapping (2D→3D)
✅ Configurable parameters
✅ Debug visualization option

### 3. Point Cloud Fusion
✅ Multi-source fusion (LiDAR + Depth + Lanes)
✅ Weighted lane boundaries (10x weight default)
✅ Intelligent downsampling
✅ Range filtering
✅ Configurable fusion rate

### 4. Navigation Integration
✅ Updated Nav2 to use fused perception
✅ Obstacle layer using fused scan
✅ Proper frame transforms
✅ DWB controller tuned for lane following

---

## 📊 Performance Metrics

| Component | Rate | Resolution | Notes |
|-----------|------|------------|-------|
| Hokuyo LiDAR | 10 Hz | 720 points | Most reliable |
| OAK RGB | 30 Hz | 1920x1080 | High res color |
| OAK Depth | 30 Hz | 640x400 | 3D obstacles |
| Lane Detection | 30 Hz | Variable | Depends on lanes visible |
| Point Cloud Fusion | 10 Hz | ~3000 points | Configurable |
| Nav2 Costmap | 5 Hz | 10x10m @ 5cm | Local rolling window |

---

## 🔑 Key Parameters

### Lane Detection
```python
min_lane_area = 100        # Minimum contour area (pixels²)
max_distance = 10.0        # Maximum detection distance (meters)
debug_viz = False          # Enable debug visualization
```

### Point Cloud Fusion
```python
output_frame = 'base_link' # Output frame
publish_rate = 10.0        # Fusion frequency (Hz)
lane_weight = 10.0         # Weight multiplier for lanes
max_range = 10.0           # Maximum sensor range (meters)
```

### Nav2 Costmap (Fused)
```yaml
observation_sources: fused_scan
fused_scan:
  topic: /fused_scan
  data_type: "LaserScan"
  obstacle_max_range: 9.5
  raytrace_max_range: 10.0
```

---

## 🚀 How to Use

### Basic Usage
```bash
# 1. Clean
pkill -9 gzserver; pkill -9 gzclient; sleep 2

# 2. Terminal 1: Simulation
cd ~/IGVC_SIM
source setup_orange.sh
ros2 launch orange_gazebo orange_igvc_simple.launch.xml

# 3. Terminal 2: Perception + Nav2
source setup_orange.sh
ros2 launch orange_gazebo igvc_perception_full.launch.xml

# 4. Terminal 3: Autonomous driving
source setup_orange.sh
./run_igvc_course.sh
```

### Quick Start
```bash
cd ~/IGVC_SIM
./QUICKSTART_PERCEPTION.sh
# Then follow the printed instructions
```

---

## 🔍 Monitoring & Debug

### Check Sensor Topics
```bash
ros2 topic list | grep -E "oak|lane|fused"
ros2 topic hz /lane_cloud
ros2 topic hz /fused_cloud
ros2 topic echo /fused_scan --field ranges[0:10]
```

### View Camera & Lane Detection
```bash
# View RGB camera
ros2 run rqt_image_view rqt_image_view /oak/rgb/image_raw

# Enable and view lane debug
ros2 param set /lane_detector debug_viz true
ros2 run rqt_image_view rqt_image_view /lane_debug
```

### View Point Clouds in RViz
Add these displays in RViz:
- `/hokuyo_scan` (LaserScan) - LiDAR only
- `/fused_scan` (LaserScan) - All sensors combined
- `/lane_cloud` (PointCloud2) - Detected lanes
- `/oak/stereo/points` (PointCloud2) - Depth camera
- `/fused_cloud` (PointCloud2) - Complete fusion

---

## 🎛️ Tuning Guide

### Make Robot More Cautious Around Lanes
```bash
# Increase lane weight (avoids crossing more)
ros2 param set /pointcloud_fusion lane_weight 15.0
```

### Make Robot Less Sensitive to Lanes
```bash
# Decrease lane weight (crosses more easily)
ros2 param set /pointcloud_fusion lane_weight 3.0
```

### Adjust Lane Detection Sensitivity
```bash
# Detect only larger lane segments
ros2 param set /lane_detector min_lane_area 200

# Detect lanes further away
ros2 param set /lane_detector max_distance 15.0
```

### Adjust Fusion Rate (if performance slow)
```bash
# Lower fusion rate
ros2 param set /pointcloud_fusion publish_rate 5.0
```

---

## 🐛 Troubleshooting

### Camera Not Working
- **Check**: `ros2 topic list | grep oak`
- **Solution**: Rebuild workspace with `colcon build --packages-select orange_description`

### No Lane Detection
- **Check**: `ros2 topic hz /lane_cloud` (should be >0 if lanes visible)
- **Enable debug**: `ros2 param set /lane_detector debug_viz true`
- **View**: `ros2 run rqt_image_view rqt_image_view /lane_debug`
- **Note**: Lanes must be white or yellow in simulation

### Fusion Not Publishing
- **Check inputs**:
  ```bash
  ros2 topic hz /hokuyo_scan
  ros2 topic hz /oak/stereo/points
  ros2 topic hz /lane_cloud
  ```
- **Check output**: `ros2 topic hz /fused_cloud`
- **View logs**: Check Terminal 2 for fusion node messages

### Robot Still Crosses Lanes
- **Increase lane weight**: `ros2 param set /pointcloud_fusion lane_weight 20.0`
- **Check costmap**: In RViz, verify `/local_costmap/costmap` shows lane obstacles
- **Verify fusion**: `ros2 topic echo /fused_scan --once` should show obstacles at lane positions

---

## 📚 References

### Related Packages
- [depthai-ros](https://github.com/luxonis/depthai-ros) - Real OAK camera ROS driver
- [Navigation2](https://navigation.ros.org/) - ROS2 navigation framework
- [pointcloud_to_laserscan](http://wiki.ros.org/pointcloud_to_laserscan) - PC to scan converter

### Documentation Files
- `README.md` - Main project overview
- `PERCEPTION_SETUP.md` - Detailed perception guide
- `QUICKSTART.md` - Basic Nav2 guide (no perception)

---

## ✅ Testing Checklist

- [ ] Gazebo launches with robot + camera
- [ ] Camera publishes RGB images
- [ ] Camera publishes depth point cloud
- [ ] Lane detector publishes lane cloud
- [ ] Point cloud fusion publishes fused cloud
- [ ] Fused scan appears in topic list
- [ ] Nav2 servers all start successfully
- [ ] RViz shows fused costmap with obstacles
- [ ] Robot navigates to sent waypoint
- [ ] Robot avoids physical obstacles
- [ ] Robot stays within lane boundaries
- [ ] Robot completes full course autonomously

---

## 🎓 Learning Resources

Want to understand the code better?

1. **Lane Detection**: See `lane_detector.py` - Simple HSV thresholding
2. **Point Cloud Fusion**: See `pointcloud_fusion.py` - Numpy array stacking
3. **Gazebo Sensors**: See `oak_d_pro.xacro` - Camera and depth plugins
4. **Nav2 Integration**: See `nav2_params_fused.yaml` - Costmap configuration

---

## 🚀 Future Enhancements

Potential improvements:
1. **Deep learning lane detection** (instead of HSV)
2. **Semantic segmentation** (classify obstacles)
3. **Kalman filtering** (smooth lane tracking)
4. **Multi-camera fusion** (front + sides)
5. **Dynamic obstacle tracking** (predict moving objects)
6. **Real OAK-D Pro integration** (when deploying to hardware)

---

## 📝 Notes

- All perception code is in Python for easy modification
- HSV color thresholds are tuned for Gazebo lighting
- Real-world deployment will need different thresholds
- Point cloud fusion uses simple concatenation (no registration needed in simulation)
- Lane weight is intentionally high to strongly avoid crossing

---

**System Status**: ✅ **COMPLETE AND READY TO TEST**

Run `./QUICKSTART_PERCEPTION.sh` to get started!
