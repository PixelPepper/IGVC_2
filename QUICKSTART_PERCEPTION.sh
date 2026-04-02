#!/bin/bash
# Quick Start for IGVC with Full Perception Stack
# OAK-D Pro + Lane Segmentation + Point Cloud Fusion

cat << 'EOF'
╔══════════════════════════════════════════════════════════════╗
║   IGVC Autonomous Navigation with Enhanced Perception       ║
║   OAK-D Pro Camera + Lane Detection + Sensor Fusion         ║
╚══════════════════════════════════════════════════════════════╝

STEP 1: CLEAN (Run this now)
EOF

echo "Killing all processes..."
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 python3 2>/dev/null
sleep 2

cat << 'EOF'

✓ Clean complete!

═══════════════════════════════════════════════════════════════
STEP 2: TERMINAL 1 - Launch Gazebo Simulation
═══════════════════════════════════════════════════════════════
cd $WORKSPACE
source setup_orange.sh
ros2 launch orange_gazebo orange_igvc_simple.launch.xml

Wait for: "Successfully spawned entity [orange_robot]"

═══════════════════════════════════════════════════════════════
STEP 3: TERMINAL 2 - Launch Perception + Nav2
═══════════════════════════════════════════════════════════════
cd $WORKSPACE
source setup_orange.sh
ros2 launch orange_gazebo igvc_perception_full.launch.xml

Wait 15-20 seconds for:
 ✓ Lane Detector initialized
 ✓ Point Cloud Fusion initialized
 ✓ Nav2 servers connected

═══════════════════════════════════════════════════════════════
STEP 4: TERMINAL 3 - Launch RViz (Optional)
═══════════════════════════════════════════════════════════════
cd $WORKSPACE
source setup_orange.sh
ros2 launch orange_bringup rviz2.launch.xml

RViz Config:
 1. Fixed Frame → odom
 2. Add → Map → /local_costmap/costmap
 3. Add → Path → /plan
 4. Add → LaserScan → /fused_scan
 5. Add → PointCloud2 → /lane_cloud
 6. Add → Image → /oak/rgb/image_raw

═══════════════════════════════════════════════════════════════
STEP 5: TERMINAL 4 - Start Autonomous Course
═══════════════════════════════════════════════════════════════
cd $WORKSPACE
./run_igvc_course.sh

Watch the robot:
 ✓ Detect lanes (white/yellow)
 ✓ Avoid obstacles (LiDAR + Depth)
 ✓ Navigate autonomously through course
 ✓ Stay within lane boundaries

═══════════════════════════════════════════════════════════════
MONITORING COMMANDS
═══════════════════════════════════════════════════════════════

Check all sensors:
  ros2 topic hz /hokuyo_scan        # LiDAR
  ros2 topic hz /oak/rgb/image_raw  # Camera
  ros2 topic hz /oak/stereo/points  # Depth
  ros2 topic hz /lane_cloud         # Detected lanes
  ros2 topic hz /fused_cloud        # Fused sensors

View camera:
  ros2 run rqt_image_view rqt_image_view /oak/rgb/image_raw

View lane detection:
  ros2 param set /lane_detector debug_viz true
  ros2 run rqt_image_view rqt_image_view /lane_debug

═══════════════════════════════════════════════════════════════
TUNING PARAMETERS
═══════════════════════════════════════════════════════════════

Make lanes MORE important (avoid crossing):
  ros2 param set /pointcloud_fusion lane_weight 10.0

Make lanes LESS important:
  ros2 param set /pointcloud_fusion lane_weight 2.0

Adjust lane detection sensitivity:
  ros2 param set /lane_detector min_lane_area 200
  ros2 param set /lane_detector max_distance 15.0

═══════════════════════════════════════════════════════════════

📚 Full Documentation:
   - PERCEPTION_SETUP.md (complete guide)
   - README.md (overview)

🤖 Ready to test! Open 4 terminals and follow the steps above.

EOF
