# Cartesian Controller Utilities

This package turns the 3DConnexion (*spacenav_node*) from the
[joystick_drivers](https://github.com/ros-drivers/joystick_drivers) into a
force and motion control input device for teleoperation with the `cartesian_controllers`:

- **Force control**: Re-publish `geometry_msgs/Twist` messages as `geometry_msgs/WrenchStamped` messages to intuitively control the `target_wrench`.
- **Motion control**: Turn `geometry_msgs/Twist` messages into a `geometry_msgs/PoseStamped` to smoothly steer the robot via its `target_frame`.
- **Button commands**: Define custom commands for the space mouse buttons with ROS command line instructions. You can trigger events, such as calling ROS services and publish to topics.

## Install
We need some additional system dependencies
```bash
sudo apt install libspnav-dev spacenavd screen
```

and these Python dependencies for the scripts:
```bash
pip3 install numpy numpy-quaternion
```

We also need a custom version of the `joystick_drivers` (one that correctly reports the button events).
Until pending PRs are merged upstream there, you'll find the package [here](https://github.com/stefanscherzinger/joystick_drivers).
Navigate into your `src` folder in your workspace,
```bash
git clone -b ros2 https://github.com/stefanscherzinger/joystick_drivers.git
```
and build the workspace as usual.

## Force control
Start the simulation and the force controller as described [here](../cartesian_force_controller/README.md).
In a sourced terminal, call
```bash
ros2 launch cartesian_controller_utilities spacenav_to_wrench.launch.py
```
You can now steer the robot's end-effector with the space mouse.
There's a parameter to change the coordinate frame for control: In rqt open the *Dynamic Reconfigure* plugin under *Configuration*,
navigate to *my_cartesian_force_controller* and untick the box for `hand_frame_control`. You now steer the robot in the controller's `robot_base_link` coordinates.


## Motion control
Start the simulation and the compliance controller as described [here](../cartesian_compliance_controller/README.md).
In a sourced terminal, call
```bash
ros2 launch cartesian_controller_utilities spacenav_to_pose.launch.py
```
You can now steer the robot's `target_frame` with the space mouse.
The publisher makes sure that we always start from the current end-effector pose.
We use the `cartesian_compliance_controller` in this example, because it's
suitable for teleoperation that involves contact with the environment and is
thus more versatile than the `cartesian_motion_controller`.
You can take this example and adjust the setting for your own robot manipulator.


## Button commands
The space mouse's buttons are extremely handy for triggering custom events during teleoperation, such as opening and closing grippers.
You can [program both buttons](etc/button_cmds.yaml) with everything that's a valid (ROS) command on the command line.
In this example, we record the `spacenav/twist` topic into a rosbag and use the two buttons for *starting* and *stoping*, respectively.
In a source terminal, call
```bash
ros2 launch cartesian_controller_utilities spacenav_buttons.launch.py
```
Now press the space mouse's buttons. You should see output in the terminal, similar to this
```bash
[buttons.py-2] start recording
[buttons.py-2] stop recording
```
Per default, the rosbags will end up in the terminal's current folder.
A practical use case could be recording the `target_frame` during teleoperation for motion replay with the [cartesian_motion_controller](../cartesian_motion_controller/README.md).


## Disable GUI control in Gazebo
When working with the 3DConnexion as a force control input device, you probably don't want to control gazebo's camera at the same time. Remember your kernel drivers still treat your force control thing as a space mouse, which is used by gazebo by default. To switch this behavior off add the following lines to the *~/.gazebo/gui.ini* file:
```
[spacenav]
deadband_x = 0.1
deadband_y = 0.1
deadband_z = 0.1
deadband_rx = 0.1
deadband_ry = 0.1
deadband_rz = 0.1
topic=~/spacenav/remapped_joy_topic_to_something_not_used
```
More information can be found [here](http://answers.gazebosim.org/question/14225/how-can-i-turn-off-the-space-navigators-control-of-the-camera/).
