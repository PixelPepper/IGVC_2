#!/bin/bash
# Launch RViz with pre-configured IGVC Perception displays
# Works regardless of clone location (e.g. IGVC_2, IGVC_SIM)
#
# Usage:
#   ./launch_rviz_perception.sh              # default (GPU OpenGL)
#   ./launch_rviz_perception.sh --software   # software GL (workaround for
#                                            # indexed_8bit_image GLSL errors)
#   IGVC_RVIZ_SOFTWARE_GL=1 ./launch_rviz_perception.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source setup_orange.sh

USE_SOFTWARE_GL=0
for arg in "$@"; do
  case "$arg" in
    --software|--sw|software)
      USE_SOFTWARE_GL=1
      ;;
  esac
done
if [[ "${IGVC_RVIZ_SOFTWARE_GL:-0}" == "1" ]]; then
  USE_SOFTWARE_GL=1
fi

if [[ "$USE_SOFTWARE_GL" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  echo "Using software OpenGL (LIBGL_ALWAYS_SOFTWARE=1) — slower but avoids some GPU shader bugs."
  echo ""
fi

echo "Launching RViz with IGVC Perception configuration..."
echo ""
echo "Displays included:"
echo "  ✓ Robot Model"
echo "  ✓ Grid"
echo "  ○ Local Costmap (/local_costmap/costmap) — off by default (enable in RViz if needed)"
echo "  ✓ Planned Path (/plan)"
echo "  ✓ Fused Scan - All Sensors (/fused_scan)"
echo "  ✓ Lane Detection (/lane_cloud) - Green"
echo "  ✓ Depth Camera (/oak/stereo/points) - Blue"
echo "  ✓ Camera View (/oak/rgb/image_raw)"
echo ""
echo "Fixed Frame: odom"
echo ""

rviz2 -d "$SCRIPT_DIR/igvc_perception.rviz"
