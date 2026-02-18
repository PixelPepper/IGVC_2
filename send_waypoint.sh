#!/bin/bash
# Send a single waypoint to the robot
# Usage: ./send_waypoint.sh <x> <y> [yaw]

if [ $# -lt 2 ]; then
    echo "Usage: $0 <x> <y> [yaw]"
    echo "Example: $0 5.0 2.0 0.0"
    exit 1
fi

X=$1
Y=$2
YAW=${3:-0.0}

cd ~/IGVC_SIM
source setup_orange.sh

echo "Sending waypoint: x=$X, y=$Y, yaw=$YAW"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'odom'},
    pose: {
      position: {x: $X, y: $Y, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}" --feedback
