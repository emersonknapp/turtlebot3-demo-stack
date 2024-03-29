# Copyright 2019 Amazon.com Inc or its Affiliates
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

"""Launch a joy driver + interpreter combo to talk to an XBox360 controller.

The driver talks to /dev/input/js0 and publishes /joy.
The interpreter listens to /joy and publishes to /cmd_vel.
"""

import launch
from launch_ros.actions import Node


def generate_launch_description():
    joy = Node(package='joy', node_executable='joy_node', output='screen')
    joy_interp = Node(
        package='turtlebot3_joy_interpreter', node_executable='xbox360_interpreter',
        output='screen')
    return launch.LaunchDescription([
        joy,
        joy_interp,
    ])
