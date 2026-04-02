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
echo "  Terminal 1: Gazebo (orange_igvc_simple.launch.xml) ✓"
echo "  Terminal 2: Nav2 (igvc_nav2_full.launch.xml) ✓"
echo "  Terminal 3: RViz (optional) ✓"
echo ""
echo "Starting course navigation in 3 seconds..."
echo "Press Ctrl+C to stop at any time"
echo ""
sleep 3

python3 navigate_igvc_course.py
