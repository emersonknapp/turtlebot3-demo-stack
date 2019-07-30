#!/usr/bin/env python3
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

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

LEFT_HORIZONTAL_STICK = 0
LEFT_VERTICAL_STICK = 1
RIGHT_HORIZONTAL_STICK = 3
RIGHT_VERTICAL_STICK = 4
RIGHT_TRIGGER = 5
R_BUTTON = 5

MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84


class InterpreterNode(Node):
    def __init__(self):
        super().__init__('joyinterpreter')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_cb
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel')

    def joy_cb(self, msg):
        lin_vel = msg.axes[LEFT_VERTICAL_STICK] * 1.0
        ang_vel = msg.axes[RIGHT_HORIZONTAL_STICK] * 1.0
        drive_pressed = msg.buttons[R_BUTTON]

        cmd = Twist()
        cmd.linear.y = 0.
        cmd.linear.z = 0.
        cmd.angular.x = 0.
        cmd.angular.y = 0.

        if not drive_pressed:
            cmd.linear.x = 0.
            cmd.angular.z = 0.
        else:
            cmd.linear.x = lin_vel * MAX_LIN_VEL
            cmd.angular.z = ang_vel * MAX_ANG_VEL

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = InterpreterNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
