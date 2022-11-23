This package provides a rosnode `diff_drive_node.py` that works with `turtlesim`. In turtlesim, the turtle can be moved by sending `geometry_msgs.Twist` commands to `cmd_vel`. The `diff_drive_node.py` in this package converts the turtle's velocities to wheel velocities and publishes it as `nav_wheel_vel`. It also subscribes to `cmd_wheel_vel` to take wheen velocities as input and calculates the forward dynamics to get the corresponding robot velocity and publishes it as `cmd_vel`.

1. ```rosrun turtlesim turtlesim```
2. ```rosrun for_diff_robot diff_drive_node.py```
3. ```rostopic echo /turtle1/nav_wheel_vel```
4. Publish wheel velocities to `cmd_wheel_vel`