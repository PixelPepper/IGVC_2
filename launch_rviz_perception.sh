#!/bin/bash
# Launch RViz with pre-configured IGVC Perception displays

cd ~/IGVC_SIM
source setup_orange.sh

echo "Launching RViz with IGVC Perception configuration..."
echo ""
echo "Displays included:"
echo "  ✓ Robot Model"
echo "  ✓ Grid"
echo "  ✓ Local Costmap (/local_costmap/costmap)"
echo "  ✓ Planned Path (/plan)"
echo "  ✓ Fused Scan - All Sensors (/fused_scan)"
echo "  ✓ Lane Detection (/lane_cloud) - Green"
echo "  ✓ Depth Camera (/oak/stereo/points) - Blue"
echo "  ✓ Camera View (/oak/rgb/image_raw)"
echo ""
echo "Fixed Frame: odom"
echo ""

rviz2 -d ~/IGVC_SIM/igvc_perception.rviz
