# Turtlebot3 On-Robot ROS2 Demo Software Stack
This package provides a top-level launchfile - intended to be run on a Turtlebot3's Raspberry Pi compute.

It provides an end-to-end demonstration of basic headless robot functionality.
* Lidar driver talks to the LDS and publishes to /scan (hls_lfcd_lds_driver)
* The robot description and TF tree (turtlebot3_bringup)
* A basic joystick driver+interpreter combo to produce movement commands from an XBox360 controller connected to the Turtlebot3 (turtlebot3_joy_interpreter)
* A node for an indicator LED, run on the Pi's GPIO. This optional component allows a completely headless robot to give basic indication that the stack is running. (rpi_indicator_led)

To retrieve sources that do not exist in a fresh ROS2 installation, you can use `../on-robot-stack.repos` as input to `vcs`.

To build, you can use this package as a target

```
colcon build --packages-up-to turtlebot3_on_robot_stack
```
