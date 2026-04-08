# SPDX-License-Identifier: Apache-2.0
"""IGVC simple sim: stop any stale Gazebo (frees port 11345), then run the XML stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('orange_gazebo')
    kill_stale = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            # TERM then KILL so port 11345 is released before the next gzserver starts.
            'killall -q gzserver gzclient 2>/dev/null || true; sleep 0.5; '
            'killall -9 -q gzserver gzclient 2>/dev/null || true; sleep 1',
        ],
        output='screen',
        name='kill_stale_gazebo',
    )
    sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'orange_igvc_simple.launch.xml'),
        ),
        launch_arguments=[
            # XML also supports standalone kill+delay; avoid double-kill and extra wait when using this entry point.
            ('kill_stale_gazebo', 'false'),
            ('gazebo_stack_delay', '0'),
        ],
    )
    return LaunchDescription(
        [
            kill_stale,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=kill_stale,
                    on_exit=[sim],
                ),
            ),
        ],
    )
