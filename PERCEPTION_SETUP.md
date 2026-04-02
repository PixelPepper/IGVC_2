# 🎥 OAK-D Pro + Lane Segmentation + Point Cloud Fusion

## Overview

This enhanced setup adds:
- **OAK-D Pro depth camera** emulation (RGB + Stereo depth)
- **Lane segmentation** using computer vision (HSV color detection)
- **Point cloud fusion** (LiDAR + Depth + Lane detection)
- **Nav2 integration** with fused perception

## Architecture

```
Hardware Sensors:
├─ Hokuyo 2D LiDAR    → /hokuyo_scan
├─ OAK-D Pro RGB      → /oak/rgb/image_raw
└─ OAK-D Pro Depth    → /oak/stereo/points

Perception Pipeline:
├─ Lane Detector      → /lane_cloud (detected lane boundaries as points)
├─ Point Cloud Fusion → /fused_cloud (LiDAR + Depth + Lanes)
└─ PC to LaserScan    → /fused_scan (for Nav2 costmap)

Navigation:
└─ Nav2 Stack         → Uses /fused_scan for obstacle avoidance
```

## Complete Startup Sequence

### Step 0: Clean Everything
```bash
pkill -9 gzserver; pkill -9 gzclient; pkill -9 python3
sleep 2
```

### Step 1: Launch Simulation (Terminal 1)
```bash
cd ~/IGVC_2   # or your clone directory
source setup_orange.sh
ros2 launch orange_gazebo orange_igvc_simple.launch.xml
```

**Wait for**: Robot spawns successfully with OAK-D Pro camera visible

---

### Step 2: Launch Perception + Nav2 (Terminal 2)
```bash
cd ~/IGVC_2   # or your clone directory
source setup_orange.sh
ros2 launch orange_gazebo igvc_perception_full.launch.xml
```

**This launches**:
1. Lane Detector (detects white/yellow lanes)
2. Point Cloud Fusion (combines all sensors)
3. PointCloud to LaserScan converter
4. Full Nav2 stack with fused perception

**Wait for** (~15-20 seconds):
- `[lane_detector]: Lane Detector initialized`
- `[pointcloud_fusion]: Point Cloud Fusion initialized`
- `[lifecycle_manager_navigation]: Server controller_server connected with bond`
- `[lifecycle_manager_navigation]: Server planner_server connected with bond`

---

### Step 3: Launch RViz (Terminal 3 - Optional)
```bash
cd ~/IGVC_2   # or your clone directory
source setup_orange.sh
ros2 launch orange_bringup rviz2.launch.xml
```

**Configure RViz**:
1. Fixed Frame: `odom`
2. Add displays:
   - Map → `/local_costmap/costmap`
   - Path → `/plan`
   - LaserScan → `/fused_scan` (shows combined perception)
   - PointCloud2 → `/lane_cloud` (shows detected lanes)
   - PointCloud2 → `/oak/stereo/points` (shows depth camera)
   - Image → `/oak/rgb/image_raw` (shows camera view)
   - Image → `/lane_debug` (shows lane detection overlay)

---

### Step 4: Start Autonomous Navigation
```bash
cd ~/IGVC_2   # or your clone directory
./run_igvc_course.sh
```

---

## What's Happening

### Lane Detection
- Detects **white** and **yellow** lanes using HSV color segmentation
- Converts lane pixels to 3D points using inverse perspective mapping
- Publishes as point cloud → `/lane_cloud`
- These points get **extra weight** in the fused costmap (to avoid crossing lanes)

### Point Cloud Fusion
Combines:
1. **Hokuyo LiDAR** (2D, most reliable for obstacles)
2. **OAK-D Pro depth** (3D, for volumetric obstacles)
3. **Lane detections** (weighted 5x to strongly avoid crossing)

Result: `/fused_cloud` contains all obstacle information

### Navigation
- Nav2 uses `/fused_scan` (converted from fused point cloud)
- Robot **avoids**:
  - Physical obstacles (barrels, cones)
  - Lane boundaries (heavily weighted)
  - Depth obstacles (poles, barriers)

---

## Monitoring & Debugging

### Check All Sensors
```bash
source ~/IGVC_2/setup_orange.sh   # or your clone path

# Check camera is publishing
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/stereo/points

# Check lane detection
ros2 topic hz /lane_cloud

# Check fusion
ros2 topic hz /fused_cloud
ros2 topic hz /fused_scan
```

### View Camera Feed
```bash
source ~/IGVC_2/setup_orange.sh   # or your clone path
ros2 run rqt_image_view rqt_image_view /oak/rgb/image_raw
```

### View Lane Detection Debug
```bash
# Enable debug visualization
ros2 param set /lane_detector debug_viz true

# View in rqt
ros2 run rqt_image_view rqt_image_view /lane_debug
```

### Check Point Cloud Fusion
```bash
source ~/IGVC_2/setup_orange.sh   # or your clone path

# See number of points being fused
ros2 topic echo /fused_cloud --field width

# Check fusion rate
ros2 topic hz /fused_cloud
```

---

## Configuration Files

- **Robot with Camera**: `src/orange_ros2/orange_description/xacro/orange_robot_simulation.xacro`
- **Camera Sensor**: `src/orange_ros2/orange_description/xacro/sensors/oak_d_pro.xacro`
- **Lane Detector**: `src/orange_ros2/orange_perception/orange_perception/lane_detector.py`
- **Point Cloud Fusion**: `src/orange_ros2/orange_perception/orange_perception/pointcloud_fusion.py`
- **Nav2 Config**: `nav2_params_fused.yaml`

---

## Tuning Parameters

### Lane Detection Sensitivity
Edit in Terminal 2 while running:
```bash
# Increase minimum lane area (fewer false positives)
ros2 param set /lane_detector min_lane_area 200

# Increase max detection distance
ros2 param set /lane_detector max_distance 15.0
```

### Lane Weight in Fusion
```bash
# Make lanes MORE important (harder to cross)
ros2 param set /pointcloud_fusion lane_weight 10.0

# Make lanes LESS important
ros2 param set /pointcloud_fusion lane_weight 2.0
```

### Costmap Settings
Edit `nav2_params_fused.yaml`:
```yaml
obstacle_layer:
  fused_scan:
    obstacle_max_range: 9.5  # How far to see obstacles
    raytrace_max_range: 10.0  # How far to clear space
```

---

## Troubleshooting

### Camera Not Publishing
```bash
# Check if camera exists in simulation
ros2 topic list | grep oak

# Check Gazebo sensor
gz topic -l | grep camera
```

### No Lane Detection
- Lanes must be white or yellow
- Check camera is pointing forward
- Enable debug viz: `ros2 param set /lane_detector debug_viz true`

### Fusion Not Working
```bash
# Check all inputs are arriving
ros2 topic hz /hokuyo_scan
ros2 topic hz /oak/stereo/points
ros2 topic hz /lane_cloud

# Check fusion output
ros2 topic hz /fused_cloud
```

### Robot Ignores Lanes
- Increase `lane_weight` parameter
- Check `/lane_cloud` is publishing points
- Verify `/fused_scan` includes lane obstacles

---

## Performance Notes

- **Lane detection**: ~30 Hz on camera input
- **Point cloud fusion**: 10 Hz (configurable)
- **Depth camera**: 30 Hz, 640x400 resolution
- **RGB camera**: 30 Hz, 1920x1080 resolution

Lower rates if performance is slow:
```bash
ros2 param set /pointcloud_fusion publish_rate 5.0
```

---

## Next Steps

Want to improve further?
1. **Add obstacle classification** (cones vs barrels vs poles)
2. **Implement Kalman filtering** for lane tracking
3. **Add semantic segmentation** with deep learning
4. **Multi-camera setup** (front + sides)
5. **Dynamic obstacle tracking** (moving objects)

