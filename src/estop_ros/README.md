# estop_ros [![ROS 2 Industrial CI](https://github.com/KBKN-Autonomous-Robotics-Lab/estop_ros/actions/workflows/ci.yml/badge.svg)](https://github.com/KBKN-Autonomous-Robotics-Lab/estop_ros/actions/workflows/ci.yml)
This ROS 2 package overrides the `geometry_msgs/Twist` topic in response to an emergency stop (E-STOP) signal received via serial connection.
## Setup ⚙
```
$ cd ~/<path_to_your_workspace>/src
$ git clone https://github.com/KBKN-Autonomous-Robotics-Lab/estop_ros.git
$ wstool merge estop_ros/estop_ros.rosinstall
$ wstool update
$ rosdep install -r -y -i --from-paths .
$ cd ~/<path_to_your_workspace> && colcon build
```
## Run 🚀
```
$ ros2 launch estop_ros estop_control.launch.xml
```
