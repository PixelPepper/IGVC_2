#!/bin/bash
# Run IGVC Course Navigation
# This script sends the robot through the entire course automatically
# Works regardless of clone location (e.g. IGVC_2, IGVC_SIM, ~/projects/IGVC_2)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source setup_orange.sh

echo "================================"
echo "IGVC Course Navigation"
echo "================================"
echo ""
echo "Make sure you have:"
echo "  Terminal 1: Gazebo (orange_igvc_simple.launch.py) ✓"
echo "  Terminal 2: Perception + Nav2 with lane costmaps (igvc_perception_full.launch.xml) ✓"
echo "  Terminal 3: RViz (optional) ✓"
echo ""
echo "  LiDAR-only (no /lane_cloud): use igvc_nav2_full.launch.xml with"
echo "    params_file:=\$(ros2 pkg prefix orange_gazebo)/share/orange_gazebo/config/nav2_params_no_map.yaml"
echo ""
echo "Starting course navigation in 3 seconds..."
echo "Press Ctrl+C to stop at any time"
echo ""
sleep 3

python3 navigate_igvc_course.py
