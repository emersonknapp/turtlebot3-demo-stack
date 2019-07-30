# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch all of the robot-side ROS2 nodes for the turtlebot3 demo stack."""

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def external_py_launch(
    package_name: str, launchfile_name: str, launch_args: dict = {},
) -> IncludeLaunchDescription:
    pkg_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource([
        pkg_share, '/launch/', launchfile_name]),
        launch_arguments=launch_args.items())


def generate_launch_description():
    lidar = Node(
        package='hls_lfcd_lds_driver', node_executable='hlds_laser_publisher',
        output='screen')
    # Ideally frame of lidar would be configurable instead of needing extra static tf link
    lidar_frame_link = Node(
        package='tf2_ros', node_executable='static_transform_publisher',
        arguments='0 0 0 0 0 0 1.0 base_scan laser'.split(' '),
        output='screen')
    indicator_led = Node(
        package='rpi_indicator_led', node_executable='basic_blinker', output='screen')
    joy_teleop = external_py_launch('turtlebot3_joy_interpreter', 'xbox360.launch.py')
    turtlebot3_bringup = external_py_launch(
        'turtlebot3_bringup', 'robot.launch.py', {'use_sim_time': 'false'})

    return launch.LaunchDescription([
        lidar,
        lidar_frame_link,
        indicator_led,
        joy_teleop,
        turtlebot3_bringup,
    ])
