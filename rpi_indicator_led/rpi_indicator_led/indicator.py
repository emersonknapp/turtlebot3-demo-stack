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

from gpiozero import PWMLED

import rclpy
from rclpy.node import Node

OUTPUT_PIN = 18


class IndicatorNode(Node):
    def __init__(self, led):
        super().__init__('indicatorled')
        self.led = led
        self.startup_pulse()
        self.create_timer = self.create_timer(2.0, self.blink)

    def startup_pulse(self):
        self.led.pulse(fade_in_time=1, fade_out_time=0, n=1, background=False)
        self.led.value = 1

    def blink(self):
        self.led.blink(on_time=0.2, off_time=0.1, n=2, background=False)
        self.led.value = 1


def main(args=None):
    led = PWMLED(OUTPUT_PIN)
    rclpy.init(args=args)
    node = IndicatorNode(led)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    led.value = 0


if __name__ == '__main__':
    main()
