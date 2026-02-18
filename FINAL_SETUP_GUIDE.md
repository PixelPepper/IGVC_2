# 🏁 Complete IGVC Autonomous Navigation Setup

## 🆕 Latest Updates (2026-02-11)

### Lane Detection System - FULLY OPERATIONAL! ✅

**What Changed:**
- **Direct Nav2 Integration**: Lane boundaries now fed directly to Nav2's costmap (no complex fusion)
- **Aggressive Lane Detection**: Enhanced HSV + grayscale thresholding for reliable white/yellow line detection
- **Vertical Lane Walls**: Detected lanes create 3D "walls" (0-1.0m height) that Nav2 treats as physical barriers
- **OAK-D Pro Camera**: Mounted at 1.0m height, 30° pitch, looking down at ground ahead
- **Real-time Monitoring**: Terminal 2 shows `✓ Detected XXXX lane wall points` every second

**Expected Behavior:**
- Robot detects ~2400-2600 lane boundary points per frame
- Nav2 sees lanes as HIGH-PRIORITY obstacles
- Robot will stop/turn before crossing white/yellow lines
- Smooth navigation while staying within lane boundaries

---

## Your IGVC Course Waypoints
The robot will navigate through these waypoints in order:
1. **Waypoint 1**: x=-15.90, y=30.53
2. **Waypoint 2**: x=-6.69, y=30.59
3. **Waypoint 3**: x=4.74, y=30.75
4. **Waypoint 4**: x=14.03, y=30.43
5. **Return Home**: x=0.00, y=0.00

---

## 🚀 Complete Startup Sequence

### Step 0: Clean Everything

```bash
pkill -9 gzserver; pkill -9 gzclient; pkill -9 python3; pkill -9 xterm
sleep 2
```

---

### Step 1: TERMINAL 1 - Launch Gazebo

```bash
cd ~/IGVC_SIM
source setup_orange.sh
ros2 launch orange_gazebo orange_igvc_simple.launch.xml
```

**✅ Wait for:**
- `Successfully spawned entity [orange_robot]`
- Gazebo window opens showing the IGVC world

**If Gazebo window doesn't open**, run in another terminal:
```bash
DISPLAY=:0 gzclient &
```

---

### Step 2: TERMINAL 2 - Launch Perception + Navigation

**NEW terminal:**

```bash
cd ~/IGVC_SIM
source setup_orange.sh
ros2 launch orange_gazebo igvc_perception_full.launch.xml
```

**✅ Wait for these CRITICAL messages (15-20 seconds):**
```
[lane_detector]: Lane Detector initialized
[lane_detector]: Camera intrinsics received
[lane_detector]: 📸 Receiving images: 640x480
[lane_detector]: ✓ Detected 2466 lane wall points    <-- LANES DETECTED!
[pointcloud_fusion]: Point Cloud Fusion inicd ~/IGVC_SIM
source setup_orange.sh
ros2 launch orange_gazebo igvc_perception_full.launch.xmltialized
[local_costmap]: Subscribed to Topics: fused_scan lane_cloud    <-- BOTH sensors!
[lifecycle_manager_navigation]: Managed nodes are active
```

**🔍 IMPORTANT - Lane Detection Check:**
- You MUST see `✓ Detected XXXX lane wall points` messages repeating
- If you see `⚠ NO LANES DETECTED!` instead, the camera needs adjustment
- Typical lane point count: 2400-2600 points

**Ignore:**
- "Robot is out of bounds" warnings (normal at startup)
- "Message Filter dropping" messages (timing, harmless)
- QoS incompatibility warnings (harmless)

---

### Step 3: TERMINAL 3 - Test Single Waypoint

**NEW terminal:**

```bash
cd ~/IGVC_SIM
source setup_orange.sh

# Test with first waypoint
./send_waypoint.sh -15.897553 30.531566
```

**Watch in Gazebo:**
- Robot should start moving toward waypoint
- Avoids obstacles
- Stays in lanes
- Terminal 2 shows navigation progress

---

### Step 4: TERMINAL 4 - Full Course Navigation

**After test works, run full course in NEW terminal:**

```bash
cd ~/IGVC_SIM
./run_igvc_course.sh
```

**Robot will automatically:**
- Navigate to Waypoint 1: (-15.90, 30.53)
- Navigate to Waypoint 2: (-6.69, 30.59)
- Navigate to Waypoint 3: (4.74, 30.75)
- Navigate to Waypoint 4: (14.03, 30.43)
- Return to Start: (0.00, 0.00)

**Each waypoint completion shows:**
```
✓ Waypoint X reached!
Sending waypoint X+1/5: [name]
Target: x=XX.XX, y=YY.YY
Goal accepted! Robot navigating...
```

---

### Step 5: RViz Visualization (OPTIONAL)

**OPTION A - Pre-configured RViz (RECOMMENDED):**

```bash
cd ~/IGVC_SIM
./launch_rviz_perception.sh
```

This launches RViz with all displays pre-configured!

**OPTION B - Manual RViz Launch:**

```bash
cd ~/IGVC_SIM
source setup_orange.sh
ros2 launch orange_bringup rviz2.launch.xml
```

Then manually configure:

1. **Fixed Frame** → `odom` (top of left panel)

2. **Add displays** (click "Add" button):
   - **Map** → `/local_costmap/costmap`
   - **Map** → `/global_costmap/costmap`
   - **Path** → `/plan`
   - **LaserScan** → `/fused_scan` (Size: 0.05, Style: Spheres, Color: Red)
   - **PointCloud2** → `/lane_cloud` (Size: 0.05, Color: Green)
   - **PointCloud2** → `/oak/stereo/points` (Size: 0.03, Color: Blue)
   - **Image** → `/oak/rgb/image_raw`
   - **Image** → `/lane_debug` (lane detection visualization)
   - **RobotModel** (should already be there)

---

## 🧠 How the Perception System Works

### Multi-Sensor Fusion Architecture

1. **Hokuyo LiDAR** (`/hokuyo_scan`)
   - 2D laser scanner detecting obstacles
   - Converted to point cloud and published as `/fused_scan`
   - Primary obstacle detection sensor

2. **OAK-D Pro Camera** (mounted at 1.0m, 30° downward pitch)
   - RGB images: `/oak/rgb/image_raw` (640x480)
   - Depth point cloud: `/oak/stereo/points` (currently not used)
   - Positioned on tall pole to see ground lanes ahead

3. **Lane Detection Node** (`lane_detector`)
   - Processes RGB camera images
   - Uses aggressive HSV + grayscale thresholding for white/yellow lanes
   - Creates 3D "wall" of points at detected lane boundaries
   - Publishes to: `/lane_cloud` (PointCloud2)
   - **CRITICAL**: Must detect ~2400-2600 points for lanes to work!

4. **Navigation2 (Nav2) Costmap Integration**
   - Nav2's obstacle layer subscribes to BOTH:
     - `/fused_scan` (LiDAR data as LaserScan)
     - `/lane_cloud` (Lane boundaries as PointCloud2)
   - Nav2 handles all coordinate frame transforms internally
   - Lane points are marked as obstacles (cannot cross)
   - LiDAR points provide obstacle detection + path clearing

### Why This Works
- **Direct sensor integration**: No complex fusion node, Nav2 does TF transforms
- **Vertical lane walls**: Lane points stacked 0-1.0m height so Nav2 sees them as barriers
- **Aggressive detection**: Multiple thresholding methods ensure white lines are found
- **Thick morphological dilation**: Makes lane boundaries very visible in costmap

---

## 📊 What You'll See

### In Gazebo Window
- Orange robot moving through the course
- Robot avoiding barrels, cones, and barriers
- Robot staying within white/yellow lane boundaries
- Smooth path following

### In Terminal 2 (Nav2)
- Navigation progress messages
- Costmap updates
- Obstacle detection logs
- Planning success/failure messages

### In Terminal 3/4 (Waypoint Sender)
- Waypoint progress: "Sending waypoint X/5..."
- Distance remaining updates
- Success messages: "✓ Waypoint X reached!"

### In RViz (if running)
- **Green path line** showing planned route
- **Costmap** showing obstacles (red) and free space (gray)
- **Red/white dots** from fused sensors (LiDAR + camera + lanes)
- **Green points** showing detected lane boundaries
- **Robot model** moving in real-time

---

## 🔧 Monitoring & Debugging

### Check All Sensors Working
```bash
cd ~/IGVC_SIM
source setup_orange.sh

# Should all show positive Hz rates
ros2 topic hz /hokuyo_scan          # LiDAR: ~10 Hz
ros2 topic hz /oak/rgb/image_raw    # Camera: ~5-30 Hz
ros2 topic hz /lane_cloud           # Lanes: ~1 Hz (CRITICAL!)
ros2 topic hz /fused_scan           # Fused: ~10 Hz

# Check lane detection points count
ros2 topic echo /lane_cloud --field width  # Should be ~2400-2600
```

### View Camera Feed
```bash
ros2 run rqt_image_view rqt_image_view /oak/rgb/image_raw
```

### Enable Lane Detection Debug
```bash
ros2 param set /lane_detector debug_viz true
ros2 run rqt_image_view rqt_image_view /lane_debug
```

### Check Robot Position
```bash
ros2 topic echo /odom --field pose.pose.position
```

### Check Nav2 Status
```bash
ros2 node list | grep -E "controller|planner|navigator"
```

---

## 🎛️ Tuning Parameters

### Adjust Lane Detection Sensitivity
```bash
# Detect smaller lane segments (more sensitive)
ros2 param set /lane_detector min_lane_area 50

# Detect larger lane segments only (less sensitive)
ros2 param set /lane_detector min_lane_area 500

# Change detection range
ros2 param set /lane_detector max_distance 15.0  # (default: 10.0)
```

### Enable/Disable Lane Detection Debug Visualization
```bash
# Enable debug images on /lane_debug topic
ros2 param set /lane_detector debug_viz true

# View debug:
ros2 run rqt_image_view rqt_image_view /lane_debug
```

### Slow Down Robot
Edit `nav2_params_fused.yaml` and change:
```yaml
max_vel_x: 0.5  # (default is 0.8)
max_vel_theta: 0.5  # (default is 1.0)
```
Then restart Terminal 2.

### Increase Obstacle Avoidance Safety
Edit `nav2_params_fused.yaml`:
```yaml
inflation_radius: 1.0  # (default: 0.8) - larger = more cautious
robot_radius: 0.4      # (default: 0.3) - larger = wider safety margin
```
Then restart Terminal 2.

---

## 🐛 Troubleshooting

### Robot Not Moving
**Check:**
```bash
# Nav2 nodes running?
ros2 node list | grep navigator

# Action server available?
ros2 action list | grep navigate_to_pose

# Robot getting commands?
ros2 topic echo /cmd_vel
```

### Robot Crosses Lanes

**First, verify lanes are being detected:**
```bash
# Check Terminal 2 for this message:
[lane_detector]: ✓ Detected 2466 lane wall points

# Or check the topic:
ros2 topic hz /lane_cloud
```

**If lanes ARE detected but robot still crosses:**
- Lanes are now sent DIRECTLY to Nav2's costmap (no fusion needed)
- Check Nav2 is subscribed: Look for `Subscribed to Topics: fused_scan lane_cloud`
- The system should already be configured with high lane priority

**If lanes NOT detected (⚠ NO LANES DETECTED!):**
```bash
# Check camera is publishing:
ros2 topic hz /oak/rgb/image_raw

# View camera feed:
ros2 run rqt_image_view rqt_image_view /oak/rgb/image_raw

# View lane detection debug:
ros2 run rqt_image_view rqt_image_view /lane_debug
```

**Camera may need adjustment** if not seeing ground properly (mounted at 1.0m, 30° pitch)

### "Goal off global costmap" Errors
- This happens with the global costmap trying to use a rolling window
- Local costmap still works for navigation
- Robot should still move (using local planning only)

### Gazebo Window Freezes
```bash
pkill -9 gzclient
DISPLAY=:0 gzclient &
```

### Lane Detection Working But Not in Costmap

**Verify Nav2 is subscribed to lane_cloud:**
```bash
# In Terminal 2, look for:
[local_costmap]: Subscribed to Topics: fused_scan lane_cloud
[global_costmap]: Subscribed to Topics: fused_scan lane_cloud
```

If you only see `fused_scan`, the config didn't load properly:
```bash
# Stop Terminal 2 (Ctrl+C), then restart:
cd ~/IGVC_SIM
source setup_orange.sh
ros2 launch orange_gazebo igvc_perception_full.launch.xml
```

### Want to Stop Mid-Course
- In Terminal 4, press **Ctrl+C**
- Robot will stop immediately

---

## 📝 Quick Reference

**Start Everything:**
```bash
# Terminal 1
cd ~/IGVC_SIM && source setup_orange.sh
ros2 launch orange_gazebo orange_igvc_simple.launch.xml

# Terminal 2
cd ~/IGVC_SIM && source setup_orange.sh
ros2 launch orange_gazebo igvc_perception_full.launch.xml

# Terminal 3 (after 20 seconds)
cd ~/IGVC_SIM && ./run_igvc_course.sh
```

**Stop Everything:**
```bash
pkill -9 gzserver; pkill -9 gzclient; pkill -9 python3
```

**Restart:**
- Run Step 0 (clean)
- Run Terminal 1
- Run Terminal 2
- Run Terminal 3

---

## ✅ Expected Behavior

1. **Robot starts at origin** (0, 0)
2. **Navigates to Waypoint 1** (-15.90, 30.53)
   - Detects and avoids obstacles
   - Stays within lane boundaries
3. **Continues to Waypoints 2-4** along the course
4. **Returns to origin** (0, 0)

**Total distance**: ~80+ meters through the course

**Time**: Depends on obstacles, typically 5-10 minutes for full course

---

## 🩺 Quick Health Check

Before starting navigation, verify all systems are operational:

**✅ Lane Detection Health Check (Terminal 2):**
```
[lane_detector]: ✓ Detected 2466 lane wall points    <-- GOOD!
```

**❌ Lane Detection FAILED:**
```
[lane_detector]: ⚠ NO LANES DETECTED!    <-- BAD! Camera issue!
```

**✅ Nav2 Sensor Integration:**
```
[local_costmap]: Subscribed to Topics: fused_scan lane_cloud    <-- BOTH sensors!
```

**✅ All Servers Active:**
```
[lifecycle_manager_navigation]: Managed nodes are active
```

**Quick Sensor Test (New Terminal):**
```bash
cd ~/IGVC_SIM
source setup_orange.sh

# All should return data
ros2 topic hz /hokuyo_scan      # ~10 Hz
ros2 topic hz /lane_cloud       # ~1 Hz
ros2 topic hz /fused_scan       # ~10 Hz

# Lane points count
ros2 topic echo /lane_cloud --field width  # Should be 2400-2600
```

---

## 🎉 You're Ready!

All waypoints are configured. Follow the steps above to start!

**Pre-Flight Checklist:**
- ✅ Gazebo running (Terminal 1) - robot visible
- ✅ Nav2 + Perception running (Terminal 2) - managed nodes active
- ✅ Lane detection working - seeing `✓ Detected XXXX` messages
- ✅ Nav2 subscribed to both sensors - `fused_scan lane_cloud`
- 🎯 **Ready to navigate!**

**Run the Course:**
```bash
cd ~/IGVC_SIM
./run_igvc_course.sh
```

Watch the robot complete your IGVC course autonomously! 🤖🏁

**Expected Course Duration**: 5-10 minutes for all 5 waypoints (~80+ meters total)
