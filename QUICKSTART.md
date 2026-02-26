# IGVC Autonomous Navigation - Quick Start Guide

## Full Stack Launch (Step-by-Step)

### Terminal 1: Launch Gazebo Simulation
```bash
cd ~/IGVC_2   # or your clone directory
source setup_orange.sh
ros2 launch orange_gazebo orange_igvc_simple.launch.xml
```

**What this does:**
- Starts Gazebo with the IGVC world (obstacles, lanes, etc.)
- Spawns the Orange robot with Hokuyo LiDAR
- Sets up robot localization (EKF for sensor fusion)

**Wait for:** Gazebo GUI to fully open and robot to appear

---

### Terminal 2: Launch Autonomous Navigation
```bash
cd ~/IGVC_2   # or your clone directory
source setup_orange.sh
ros2 launch waypoint_navigation waypoint_nav.launch.xml \
  use_sim_time:=true \
  waypoints_file:=$(ros2 pkg prefix orange_gazebo)/share/orange_gazebo/config/waypoints/igvc_waypoints.yaml \
  world_frame:=odom \
  tandem_scan:=/hokuyo_scan
```

**What this does:**
- Opens xterm window showing navigation status
- Loads waypoints from `igvc_waypoints.yaml`
- Starts autonomous waypoint following
- Enables obstacle detection and avoidance

**Expected:** Robot starts moving towards first waypoint

---

### Terminal 3: Launch RViz2 (Optional - for visualization)
```bash
cd ~/IGVC_2   # or your clone directory
source setup_orange.sh
ros2 launch orange_bringup rviz2.launch.xml
```

**What this shows:**
- Robot 3D model
- LiDAR scan data (red points)
- TF transforms (coordinate frames)
- Odometry path

---

## System Overview

```
┌─────────────────┐
│  Gazebo World   │  ← Physics simulation, sensors
└────────┬────────┘
         │
    ┌────▼────┐
    │  Robot  │  ← Orange robot with Hokuyo LiDAR
    └────┬────┘
         │
    ┌────▼────────────────┐
    │  Sensor Data        │  ← /hokuyo_scan, /odom, /imu
    └────┬────────────────┘
         │
    ┌────▼──────────────────────┐
    │  Waypoint Navigation      │  ← Path planning & obstacle avoidance
    └────┬──────────────────────┘
         │
    ┌────▼────┐
    │ /cmd_vel│  ← Velocity commands to robot
    └─────────┘
```

---

## Monitoring the System

### Check Robot is Moving
```bash
source ~/IGVC_2/setup_orange.sh   # or your clone path
ros2 topic echo /cmd_vel
```
**Should see:** linear.x and angular.z values changing

### Check Robot Position
```bash
ros2 topic echo /odom --field pose.pose.position
```
**Should see:** x and y values increasing as robot moves

### Check LiDAR Data
```bash
ros2 topic echo /hokuyo_scan --field ranges[0:10]
```
**Should see:** Distance measurements (close values = obstacles detected)

### List All Topics
```bash
ros2 topic list
```

### Check Node Graph
```bash
ros2 node list
```

---

## Adjusting Waypoints

Edit `/home/tech/IGVC_SIM/igvc_waypoints.yaml`:

```yaml
waypoints:
- point: {x: 2.0, y: 0.0, z: 0.0, vel: 0.8, rad: 0.5, stop: false}
- point: {x: 4.0, y: 0.5, z: 0.0, vel: 0.8, rad: 0.5, stop: false}
# Add more waypoints...
```

**Parameters:**
- `x, y`: Position in meters (odom frame)
- `vel`: Max velocity (m/s) - lower for tight sections
- `rad`: Acceptance radius - how close to waypoint before moving to next
- `stop`: Whether to stop at this waypoint

**After editing:** Restart Terminal 2 (Ctrl+C, then relaunch)

---

## Manual Control (If Needed)

To drive manually and test/explore the course:

```bash
cd ~/IGVC_2   # or your clone directory
source setup_orange.sh
ros2 launch orange_teleop teleop_keyboard.launch.xml
```

**Controls:**
- Arrow keys or WASD to move
- Watch `/odom` to note positions for waypoints

---

## Troubleshooting

### Robot Not Moving
1. Check xterm window for errors
2. Verify waypoints are reasonable (start at 0,0)
3. Check `/cmd_vel` topic for velocity commands
4. Make sure `world_frame:=odom` matches your setup

### Robot Going in Circles
- Waypoints might be behind the robot
- Adjust waypoints to be in front of starting position
- Use manual control to explore and find good waypoint positions

### Gazebo Crashes
```bash
# Kill all Gazebo processes
pkill -9 gzserver; pkill -9 gzclient
# Relaunch from Terminal 1
```

### "Address already in use" Error
```bash
# Kill existing Gazebo
pkill -9 gzserver; pkill -9 gzclient
# Wait 2 seconds, then relaunch
```

---

## Stopping Everything

1. Press **Ctrl+C** in each terminal
2. Clean up any remaining processes:
```bash
pkill -9 gzserver
pkill -9 gzclient
pkill -9 xterm
```

---

## Advanced: SLAM + Navigation

To create a map while navigating:

**Terminal 2a:** Instead of waypoint navigation, run SLAM:
```bash
source ~/IGVC_2/setup_orange.sh   # or your clone path
ros2 launch orange_slam slam_toolbox.launch.xml use_sim_time:=true
```

**Terminal 2b:** Manual teleop to drive around:
```bash
source ~/IGVC_2/setup_orange.sh   # or your clone path
ros2 launch orange_teleop teleop_keyboard.launch.xml
```

**Save map:**
```bash
cd ~/IGVC_2   # or your clone directory
ros2 run nav2_map_server map_saver_cli -f my_igvc_map
```

**Use map for navigation:**
```bash
ros2 launch orange_navigation waypoint_navigation.launch.xml \
  use_sim_time:=true \
  map:=~/IGVC_SIM/my_igvc_map.yaml
```

---

## Files Reference

- **Waypoints:** `install/orange_gazebo/share/orange_gazebo/config/waypoints/igvc_waypoints.yaml`
- **Setup Script:** `setup_orange.sh` (in workspace root)
- **Launch Files:** `install/orange_gazebo/share/orange_gazebo/launch/`
- **README:** `README.md`

---

## Quick Launch Reference

```bash
# See launch guide
~/IGVC_SIM/launch_igvc_full.sh

# Source workspace
source ~/IGVC_2/setup_orange.sh   # or your clone path

# Launch simulation
ros2 launch orange_gazebo orange_igvc_simple.launch.xml

# Launch navigation
ros2 launch waypoint_navigation waypoint_nav.launch.xml \
  use_sim_time:=true \
  waypoints_file:=$(ros2 pkg prefix orange_gazebo)/share/orange_gazebo/config/waypoints/igvc_waypoints.yaml \
  world_frame:=odom \
  tandem_scan:=/hokuyo_scan

# Launch RViz2
ros2 launch orange_bringup rviz2.launch.xml
```

---

## Success Checklist

- [ ] Gazebo opens with IGVC world
- [ ] Orange robot visible in Gazebo
- [ ] Xterm window shows waypoint navigation
- [ ] Robot moves towards waypoints
- [ ] LiDAR detects obstacles (check RViz2)
- [ ] Robot avoids obstacles while navigating

---

**Need Help?** Check the xterm window output for detailed navigation status and error messages.
