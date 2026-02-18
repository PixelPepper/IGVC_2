#!/bin/bash
# Orange ROS2 Workspace Setup Script
# Works from any directory - detects workspace from script location

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR" && pwd)"

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source the workspace overlay
source "$WORKSPACE_ROOT/install/setup.bash"

# Set Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$WORKSPACE_ROOT/install/orange_gazebo/share/orange_gazebo/models

# Set Gazebo network settings for offline mode
export GAZEBO_IP=127.0.0.1
export GAZEBO_MASTER_URI=http://localhost:11345

echo "✓ Orange ROS2 workspace activated!"
echo ""
echo "Available launch commands:"
echo "  ros2 launch orange_gazebo empty_world.launch.xml          # Empty world"
echo "  ros2 launch orange_gazebo orange_world.launch.xml         # Orange world"
echo "  ros2 launch orange_gazebo orange_igvc.launch.xml          # IGVC world"
echo "  ros2 launch orange_gazebo orange_hosei.launch.xml         # Hosei world"
echo ""
echo "  ros2 launch orange_teleop teleop_keyboard.launch.xml      # Keyboard control"
echo "  ros2 launch orange_bringup rviz2.launch.xml               # RViz visualization"
echo ""
