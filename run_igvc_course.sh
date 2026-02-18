#!/bin/bash
# Run IGVC Course Navigation
# This script sends the robot through the entire course automatically

cd ~/IGVC_SIM
source setup_orange.sh

echo "================================"
echo "IGVC Course Navigation"
echo "================================"
echo ""
echo "Make sure you have:"
echo "  Terminal 1: Gazebo (orange_igvc_simple.launch.xml) ✓"
echo "  Terminal 2: Nav2 (igvc_nav2_full.launch.xml) ✓"
echo "  Terminal 3: RViz (optional) ✓"
echo ""
echo "Starting course navigation in 3 seconds..."
echo "Press Ctrl+C to stop at any time"
echo ""
sleep 3

python3 navigate_igvc_course.py
